//! Subcommands. Linux only (V4L2 camera, CH343 serial).

use crate::bus::Bus;
use crate::config::{Config, N_MOTORS, WRIST};
use crate::control::{fit_acceleration, min_jerk, AxesMap, BallPid, BALL_ACCEL_PER_RAD};
use crate::estimator::{AlphaBeta, Homography};
use crate::kinematics::WristModel;
use crate::vision::{self, capture, Detection};
use anyhow::{bail, Context, Result};
use nalgebra::Vector2;
use st3215::protocol::RamRegister;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};
use tokio::io::{AsyncBufReadExt, BufReader};

const ARM_IDS: [u8; 7] = [1, 2, 3, 4, 5, 6, 7];

// ---------------------------------------------------------------------------------------------------------------
// Arm: the bus plus the last commanded goal of M1..M7.

struct Arm {
    bus: Bus,
    cmd: [u16; 7],
}

impl Arm {
    async fn open(cfg: &Config) -> Result<(Self, [u16; N_MOTORS])> {
        let mut bus = Bus::open(&cfg.bus.port, cfg.bus.timeout_ms)?;
        let present = bus.hold_present(cfg.poses.grip).await?;
        let mut cmd = [0u16; 7];
        cmd.copy_from_slice(&present[..7]);
        let mut all = [0u16; N_MOTORS];
        all.copy_from_slice(&present);
        Ok((Self { bus, cmd }, all))
    }

    async fn send(&mut self, goals: [u16; 7]) -> Result<()> {
        let g: Vec<(u8, u16)> = ARM_IDS.iter().zip(goals).map(|(&id, t)| (id, t)).collect();
        self.bus.sync_write_goals(&g).await?;
        self.cmd = goals;
        Ok(())
    }

    async fn send_wrist(&mut self, wrist: [u16; 3]) -> Result<()> {
        let g: Vec<(u8, u16)> = WRIST.iter().zip(wrist).map(|(&j, t)| (j as u8 + 1, t)).collect();
        self.bus.sync_write_goals(&g).await?;
        for (k, &j) in WRIST.iter().enumerate() {
            self.cmd[j] = wrist[k];
        }
        Ok(())
    }

    /// Minimum-jerk move of M1..M7 from the last command to `target`, at 50 Hz.
    async fn move_to(&mut self, target: &[u16], secs: f64) -> Result<()> {
        let start = self.cmd;
        let mut goal = [0u16; 7];
        let t0 = Instant::now();
        let mut tick = tokio::time::interval(Duration::from_millis(20));
        loop {
            tick.tick().await;
            let s = min_jerk(t0.elapsed().as_secs_f64() / secs.max(0.05));
            for j in 0..7 {
                goal[j] = (start[j] as f64 + (target[j] as f64 - start[j] as f64) * s).round() as u16;
            }
            self.send(goal).await?;
            if s >= 1.0 {
                return Ok(());
            }
        }
    }
}

fn wrist_of(p: &[u16]) -> [u16; 3] {
    [p[WRIST[0]], p[WRIST[1]], p[WRIST[2]]]
}

fn joint_rad(cfg: &Config, ticks: &[u16]) -> [f64; 7] {
    let mut q = [0.0; 7];
    for j in 0..7 {
        q[j] = cfg.tick_to_rad(j, ticks[j] as f64);
    }
    q
}

/// Wrist solver wrapper: blade-frame tilt in -> clamped wrist ticks out.
struct WristCtl {
    model: WristModel,
    level_rad: [f64; 3],
    level_ticks: [u16; 3],
    range: [u16; 3],
    cmd_rad: [f64; 3],
    null_gain: f64,
    backlash: [crate::control::Backlash; 3],
}

impl WristCtl {
    fn new(cfg: &Config, level: &[u16; N_MOTORS]) -> Self {
        let q = joint_rad(cfg, level);
        let level_rad = [q[4], q[5], q[6]];
        Self {
            model: WristModel::new(&q, cfg.joints.wrist_weight, cfg.control.damping),
            level_rad,
            level_ticks: wrist_of(level),
            range: cfg.joints.wrist_range_ticks,
            cmd_rad: level_rad,
            null_gain: cfg.control.null_space_gain,
            backlash: Default::default(),
        }
    }

    fn reset(&mut self) {
        self.cmd_rad = self.level_rad;
        for b in &mut self.backlash {
            b.reset();
        }
    }

    /// Solver tilt (u, v) in rad -> wrist goal ticks, clamped around the level pose.
    fn ticks_for(&mut self, cfg: &Config, uv: Vector2<f64>) -> [u16; 3] {
        let target = uv.map(f64::sin);
        let q = self.model.solve(&self.cmd_rad, target, &self.level_rad, self.null_gain);
        let mut out = [0u16; 3];
        for k in 0..3 {
            let j = WRIST[k];
            let lo = self.level_ticks[k] as f64 - self.range[k] as f64;
            let hi = self.level_ticks[k] as f64 + self.range[k] as f64;
            let t = cfg.rad_to_tick(j, q[k]).clamp(lo.max(0.0), hi.min(4095.0));
            // The model keeps the uncompensated angle; only the goal sent to the servo gets the slop offset.
            self.cmd_rad[k] = cfg.tick_to_rad(j, t);
            let sent = self.backlash[k].apply(t, cfg.control.backlash_ticks[k]);
            out[k] = sent.clamp(lo.max(0.0), hi.min(4095.0)).round() as u16;
        }
        out
    }
}

async fn prompt(msg: &str) -> Result<String> {
    print!("{msg}");
    std::io::stdout().flush()?;
    let mut line = String::new();
    BufReader::new(tokio::io::stdin()).read_line(&mut line).await?;
    Ok(line.trim().to_string())
}

fn homography(cfg: &Config) -> Result<Homography> {
    cfg.camera
        .homography
        .map(|h| Homography::from_row_major(&h))
        .context("no homography in config: run `paddle-balance calib-camera` first")
}

/// Wait for the next camera detection, or None after `timeout`.
async fn next_det(rx: &mut tokio::sync::watch::Receiver<Detection>, timeout: Duration) -> Result<Option<Detection>> {
    match tokio::time::timeout(timeout, rx.changed()).await {
        Ok(Ok(())) => Ok(Some(*rx.borrow_and_update())),
        Ok(Err(_)) => bail!("camera thread stopped"),
        Err(_) => Ok(None),
    }
}

/// Collect `n` ball centroids (pixels) and return the per-axis median, plus the mean HSV.
async fn median_ball_px(rx: &mut tokio::sync::watch::Receiver<Detection>, n: usize) -> Result<(Vector2<f64>, [f64; 3])> {
    let (mut xs, mut ys, mut hsv) = (Vec::new(), Vec::new(), [0.0; 3]);
    let deadline = Instant::now() + Duration::from_secs(5);
    while xs.len() < n {
        if Instant::now() > deadline {
            bail!("ball not seen in 5 s (got {} of {n} frames)", xs.len());
        }
        if let Some(Detection { blob: Some(b), .. }) = next_det(rx, Duration::from_millis(500)).await? {
            xs.push(b.px.x);
            ys.push(b.px.y);
            for k in 0..3 {
                hsv[k] += b.hsv[k] / n as f64;
            }
        }
    }
    let med = |v: &mut Vec<f64>| {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v[v.len() / 2]
    };
    Ok((Vector2::new(med(&mut xs), med(&mut ys)), hsv))
}

// ---------------------------------------------------------------------------------------------------------------

pub async fn scan(cfg_path: &Path, save_rest: bool) -> Result<()> {
    let mut cfg = Config::load(cfg_path)?;
    let mut bus = Bus::open(&cfg.bus.port, cfg.bus.timeout_ms)?;
    println!("port {}", cfg.bus.port);
    println!(" id  ping  pos   speed  load  volt  temp  curr_mA  torque");
    let mut pos = [0u16; N_MOTORS];
    for id in 1..=8u8 {
        if !bus.ping(id).await {
            println!(" {id:>2}  --");
            continue;
        }
        let s = bus.read_state(id).await?;
        let tq = bus.read_torque_enabled(id).await.unwrap_or(false);
        pos[id as usize - 1] = s.position;
        println!(
            " {id:>2}  ok   {:>4}  {:>5}  {:>4}  {:>4.1}  {:>4}  {:>7.0}  {}",
            s.position,
            s.speed,
            s.load,
            s.voltage_dv as f64 / 10.0,
            s.temperature_c,
            s.current_ma,
            if tq { "on" } else { "OFF" }
        );
    }
    println!("rest = {:?}", pos);
    if let Some(level) = cfg.poses.level {
        println!("level = {:?}", level);
    }
    if save_rest {
        cfg.poses.rest = Some(pos);
        cfg.save(cfg_path)?;
    }
    Ok(())
}

pub async fn jog(cfg_path: &Path) -> Result<()> {
    let mut cfg = Config::load(cfg_path)?;
    let (mut arm, _present) = Arm::open(&cfg).await?;
    println!(
        "jog: '<motor 1-7> <+/-ticks>' (e.g. '6 +10'), 'show', 'goto level|rest', 'save level|rest', 'q'.\n\
         Put a bubble level (or a phone level app) on the blade and adjust until it reads 0 in both directions.\n\
         M8 (gripper) is held where it is (use `grip` to tighten it)."
    );
    let mut lines = BufReader::new(tokio::io::stdin()).lines();
    loop {
        print!("jog> ");
        std::io::stdout().flush()?;
        let Some(line) = lines.next_line().await? else { break };
        let parts: Vec<&str> = line.split_whitespace().collect();
        match parts.as_slice() {
            [] => {}
            ["q"] | ["quit"] | ["exit"] => break,
            ["show"] => {
                let p = arm.bus.read_positions(&[1, 2, 3, 4, 5, 6, 7, 8]).await?;
                println!("present {:?}\ngoal    {:?}", p, arm.cmd);
            }
            ["save", which] => {
                let p = arm.bus.read_positions(&[1, 2, 3, 4, 5, 6, 7, 8]).await?;
                let mut pose = [0u16; N_MOTORS];
                pose.copy_from_slice(&p);
                // Use the commanded goal for M1..M7: it is what the servos are holding, without sag.
                pose[..7].copy_from_slice(&arm.cmd);
                match *which {
                    "level" => cfg.poses.level = Some(pose),
                    "rest" => cfg.poses.rest = Some(pose),
                    _ => {
                        println!("save level|rest");
                        continue;
                    }
                }
                cfg.save(cfg_path)?;
                println!("{which} = {:?}", pose);
            }
            ["goto", which] => {
                let pose = match *which {
                    "level" => cfg.poses.level,
                    "rest" => cfg.poses.rest,
                    _ => None,
                };
                match pose {
                    Some(p) => arm.move_to(&p, cfg.poses.move_time_s).await?,
                    None => println!("no {which} pose saved"),
                }
            }
            [m, d] => {
                let (Ok(m), Ok(d)) = (m.parse::<usize>(), d.trim_start_matches('+').parse::<i32>()) else {
                    println!("?");
                    continue;
                };
                if !(1..=7).contains(&m) || d.abs() > 200 {
                    println!("motor 1-7, step at most 200 ticks");
                    continue;
                }
                let mut g = arm.cmd;
                g[m - 1] = (g[m - 1] as i32 + d).clamp(0, 4095) as u16;
                arm.move_to(&g, 0.2 + d.abs() as f64 / 400.0).await?;
                println!("M{m} -> {}", g[m - 1]);
            }
            _ => println!("?"),
        }
    }
    Ok(())
}

pub async fn vision_test(cfg_path: &Path, seconds: f64, snapshot: Option<PathBuf>) -> Result<()> {
    let cfg = Config::load(cfg_path)?;
    let h = cfg.camera.homography.map(|h| Homography::from_row_major(&h));
    let (stx, srx) = std::sync::mpsc::channel();
    let mut cam = capture::start(&cfg.camera, h, snapshot.as_ref().map(|_| stx))?;
    let t0 = Instant::now();
    let (mut frames, mut found, mut last_print, mut max_ms) = (0u32, 0u32, Instant::now(), 0f64);
    let mut snap_frame: Option<Vec<u8>> = None;
    while t0.elapsed().as_secs_f64() < seconds {
        let Some(d) = next_det(&mut cam.rx, Duration::from_secs(1)).await? else {
            println!("no frame for 1 s");
            continue;
        };
        frames += 1;
        max_ms = max_ms.max(d.detect_ms);
        if snap_frame.is_none() {
            snap_frame = srx.try_recv().ok();
            if let (Some(f), Some(path)) = (&snap_frame, &snapshot) {
                vision::save_ppm(path, f, cfg.camera.width as usize, cfg.camera.height as usize, d.blob.map(|b| b.px))?;
                println!("saved {}", path.display());
            }
        }
        if let Some(b) = d.blob {
            found += 1;
            if last_print.elapsed() > Duration::from_millis(500) {
                last_print = Instant::now();
                let mm = h.map(|h| h.apply(b.px));
                println!(
                    "ball px ({:.0},{:.0}) area {} blobs {} hsv ({:.0},{:.0},{:.0}) {} detect {:.1} ms",
                    b.px.x,
                    b.px.y,
                    b.area,
                    d.n_blobs,
                    b.hsv[0],
                    b.hsv[1],
                    b.hsv[2],
                    mm.map(|m| format!("blade ({:.1},{:.1}) mm", m.x, m.y)).unwrap_or_default(),
                    d.detect_ms
                );
            }
        } else if last_print.elapsed() > Duration::from_millis(500) {
            last_print = Instant::now();
            println!("no ball (blobs {})", d.n_blobs);
        }
    }
    let el = t0.elapsed().as_secs_f64();
    println!("{frames} frames in {el:.1} s = {:.1} fps, ball found {:.0}%, detect max {max_ms:.1} ms", frames as f64 / el, 100.0 * found as f64 / frames.max(1) as f64);
    Ok(())
}

pub async fn calib_camera(cfg_path: &Path, offset_mm: f64) -> Result<()> {
    let mut cfg = Config::load(cfg_path)?;
    let mut cam = capture::start(&cfg.camera, None, None)?;
    println!(
        "Camera calibration. The paddle must be level (run `jog`, `goto level` first).\n\
         Mark 5 spots on the blade: the centre, and {offset_mm} mm from it in 4 directions at right angles\n\
         (+x, -x opposite each other; +y, -y opposite each other, +y a quarter turn from +x).\n\
         A small ring of tape at each spot keeps the ball still. Keep hands out of the picture."
    );
    let marks = [
        ("centre", Vector2::new(0.0, 0.0)),
        ("+x", Vector2::new(offset_mm, 0.0)),
        ("-x", Vector2::new(-offset_mm, 0.0)),
        ("+y", Vector2::new(0.0, offset_mm)),
        ("-y", Vector2::new(0.0, -offset_mm)),
    ];
    let (mut px, mut mm) = (Vec::new(), Vec::new());
    for (name, pos) in marks {
        prompt(&format!("Put the ball on the {name} mark, then press Enter... ")).await?;
        let (p, hsv) = median_ball_px(&mut cam.rx, 20).await?;
        println!("  {name}: pixel ({:.1}, {:.1}), mean HSV ({:.0}, {:.0}, {:.0})", p.x, p.y, hsv[0], hsv[1], hsv[2]);
        px.push(p);
        mm.push(pos);
    }
    let h = Homography::fit(&px, &mm)?;
    let mut worst = 0f64;
    for (p, m) in px.iter().zip(&mm) {
        worst = worst.max((h.apply(*p) - m).norm());
    }
    println!("homography fitted, worst reprojection error {worst:.2} mm");
    if worst > 5.0 {
        println!("warning: large error; check the marks and that the ball sat still");
    }
    cfg.camera.homography = Some(h.to_row_major());
    // A new image map invalidates the axes calibration.
    cfg.camera.axes = None;
    cfg.save(cfg_path)?;
    println!("next: `paddle-balance calib-axes`");
    Ok(())
}

pub async fn calib_axes(cfg_path: &Path, tilt_deg: f64) -> Result<()> {
    let mut cfg = Config::load(cfg_path)?;
    let level = cfg.level()?;
    let h = homography(&cfg)?;
    let (mut arm, _) = Arm::open(&cfg).await?;
    arm.move_to(&level, cfg.poses.move_time_s).await?;
    let mut wrist = WristCtl::new(&cfg, &level);
    let mut cam = capture::start(&cfg.camera, Some(h), None)?;
    let tilt = tilt_deg.to_radians();
    let mut cols = [Vector2::zeros(); 2];

    for k in 0..2 {
        loop {
            prompt(&format!("Axis {}: put the ball at the blade centre, let go, press Enter... ", ["u", "v"][k])).await?;
            // Baseline drift at level.
            let base = record_mm(&mut cam.rx, &h, 0.3).await?;
            let a0 = fit_acceleration(&base).unwrap_or_else(Vector2::zeros);
            let mut uv = Vector2::zeros();
            uv[k] = tilt;
            arm.send_wrist(wrist.ticks_for(&cfg, uv)).await?;
            let samples = record_mm(&mut cam.rx, &h, 0.45).await?;
            arm.move_to(&level, 0.5).await?;
            wrist.reset();
            // Skip the servo response time, fit the steady roll.
            let late: Vec<_> = samples.iter().copied().filter(|(t, _)| *t >= 0.15).collect();
            match fit_acceleration(&late) {
                Some(a) if late.len() >= 6 => {
                    let a = a - a0;
                    cols[k] = a / (BALL_ACCEL_PER_RAD * tilt);
                    println!(
                        "  axis {}: ball accel ({:.0}, {:.0}) mm/s^2, {:.2} of the model, direction {:.0} deg",
                        ["u", "v"][k],
                        a.x,
                        a.y,
                        cols[k].norm(),
                        a.y.atan2(a.x).to_degrees()
                    );
                    break;
                }
                _ => println!("  ball not tracked well enough ({} frames); again", late.len()),
            }
        }
    }
    let map = AxesMap(nalgebra::Matrix2::from_columns(&cols));
    let ang = map.axes_angle_deg();
    println!("angle between axes {ang:.0} deg (expect ~90)");
    if !(60.0..=120.0).contains(&ang) {
        println!("warning: axes far from orthogonal; check joints.sign/zero in the config, or repeat");
    }
    cfg.camera.axes = Some(map.to_row_major());
    cfg.save(cfg_path)?;
    arm.move_to(&level, 0.5).await?;
    Ok(())
}

/// Move M5, M6, M7 one at a time by `ticks` and measure which way the ball rolls, then compare with the model.
/// Pairwise angles between the joints' effects do not depend on how the camera axes are oriented, so a joint whose
/// sign is wrong in the model shows up as an angle near 180 - expected.
pub async fn probe_joints(cfg_path: &Path, ticks: i32, apply: bool) -> Result<()> {
    let mut cfg = Config::load(cfg_path)?;
    let level = cfg.level()?;
    let h = homography(&cfg)?;
    let (mut arm, _) = Arm::open(&cfg).await?;
    arm.move_to(&level, cfg.poses.move_time_s).await?;
    let mut cam = capture::start(&cfg.camera, Some(h), None)?;

    let q = joint_rad(&cfg, &level);
    let model = WristModel::new(&q, cfg.joints.wrist_weight, cfg.control.damping);
    let jac = model.jacobian(&[q[4], q[5], q[6]]);
    let rad_per_tick = std::f64::consts::TAU / 4096.0;

    let mut measured = [Vector2::zeros(); 3];
    for k in 0..3 {
        let j = WRIST[k];
        loop {
            prompt(&format!("M{}: put the ball at the blade centre, let go, press Enter... ", j + 1)).await?;
            let base = record_mm(&mut cam.rx, &h, 0.3).await?;
            let a0 = fit_acceleration(&base).unwrap_or_else(Vector2::zeros);
            let mut g = level;
            g[j] = (g[j] as i32 + ticks).clamp(0, 4095) as u16;
            arm.send(g[..7].try_into().unwrap()).await?;
            let samples = record_mm(&mut cam.rx, &h, 0.45).await?;
            arm.move_to(&level, 0.5).await?;
            let late: Vec<_> = samples.iter().copied().filter(|(t, _)| *t >= 0.15).collect();
            match fit_acceleration(&late) {
                Some(a) if late.len() >= 6 => {
                    measured[k] = (a - a0) / ticks as f64; // mm/s^2 per tick
                    let model_mag = BALL_ACCEL_PER_RAD * jac.column(k).norm() * rad_per_tick;
                    println!(
                        "  M{}: ball accel {:.1} mm/s^2 per tick, direction {:.0} deg, {:.2} of the model",
                        j + 1,
                        measured[k].norm(),
                        measured[k].y.atan2(measured[k].x).to_degrees(),
                        measured[k].norm() / model_mag
                    );
                    break;
                }
                _ => println!("  ball not tracked well enough ({} frames); again", late.len()),
            }
        }
    }
    arm.move_to(&level, 0.5).await?;

    let angle = |a: Vector2<f64>, b: Vector2<f64>| (a.dot(&b) / (a.norm() * b.norm())).clamp(-1.0, 1.0).acos().to_degrees();
    // Model effect per tick of each joint in the (u, v) tilt plane, with the configured signs.
    let model_col = |k: usize, flip: f64| -> Vector2<f64> {
        let c = jac.column(k);
        Vector2::new(c[0], c[1]) * cfg.joints.sign[WRIST[k]] * flip
    };
    println!("\n  pair     model   measured");
    for (a, b) in [(0, 1), (0, 2), (1, 2)] {
        println!(
            "  M{}-M{}   {:>5.0}   {:>5.0}",
            WRIST[a] + 1,
            WRIST[b] + 1,
            angle(model_col(a, 1.0), model_col(b, 1.0)),
            angle(measured[a], measured[b])
        );
    }
    // Which sign flips (M6 as reference) make the model agree with the measurement best?
    let mut best = (f64::MAX, [1.0, 1.0, 1.0]);
    for f5 in [1.0, -1.0] {
        for f7 in [1.0, -1.0] {
            let f = [f5, 1.0, f7];
            let err: f64 = [(0, 1), (0, 2), (1, 2)]
                .iter()
                .map(|&(a, b)| (angle(model_col(a, f[a]), model_col(b, f[b])) - angle(measured[a], measured[b])).powi(2))
                .sum();
            if err < best.0 {
                best = (err, f);
            }
        }
    }
    let flips: Vec<String> = (0..3).filter(|&k| best.1[k] < 0.0).map(|k| format!("M{}", WRIST[k] + 1)).collect();
    if flips.is_empty() {
        println!("\n  signs agree with the measurement (rms angle error {:.0} deg)", (best.0 / 3.0).sqrt());
    } else {
        println!(
            "\n  best fit: flip the sign of {} (rms angle error {:.0} deg)",
            flips.join(", "),
            (best.0 / 3.0).sqrt()
        );
        if apply {
            for k in 0..3 {
                cfg.joints.sign[WRIST[k]] *= best.1[k];
            }
            cfg.camera.axes = None; // the axes map depends on the signs
            cfg.save(cfg_path)?;
            println!("  applied; joints.sign = {:?}. Re-run calib-axes.", cfg.joints.sign);
        } else {
            println!("  re-run with --apply to change joints.sign, then re-run calib-axes");
        }
    }
    Ok(())
}

/// Ball positions in blade mm over `secs`, time measured from the call.
async fn record_mm(
    rx: &mut tokio::sync::watch::Receiver<Detection>,
    h: &Homography,
    secs: f64,
) -> Result<Vec<(f64, Vector2<f64>)>> {
    let t0 = Instant::now();
    let mut out = Vec::new();
    while t0.elapsed().as_secs_f64() < secs {
        if let Some(Detection { blob: Some(b), t, .. }) = next_det(rx, Duration::from_millis(100)).await? {
            out.push((t.saturating_duration_since(t0).as_secs_f64(), h.apply(b.px)));
        }
    }
    Ok(out)
}

pub async fn grip(cfg_path: &Path) -> Result<()> {
    crate::grip_tui::run(cfg_path).await
}

// ---------------------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
enum State {
    WaitBall,
    Balance,
}

pub async fn run(cfg_path: &Path, log_dir: &Path) -> Result<()> {
    let cfg = Config::load(cfg_path)?;
    let level = cfg.level()?;
    let h = homography(&cfg)?;
    let axes = AxesMap::from_row_major(&cfg.camera.axes.context("no axes calibration: run `paddle-balance calib-axes`")?);
    let axes = if cfg.camera.axes_normalize { axes.normalized() } else { axes };
    println!(
        "axes: u -> {:.0} deg, v -> {:.0} deg, {:.0} deg apart{}",
        axes.0[(1, 0)].atan2(axes.0[(0, 0)]).to_degrees(),
        axes.0[(1, 1)].atan2(axes.0[(0, 1)]).to_degrees(),
        axes.axes_angle_deg(),
        if cfg.camera.axes_normalize { " (normalised)" } else { "" }
    );

    let (mut arm, start_pose) = Arm::open(&cfg).await?;
    if let Some(rest) = cfg.poses.rest {
        for j in 0..7 {
            let d = (start_pose[j] as i32 - rest[j] as i32).abs();
            if d > cfg.safety.start_tolerance_ticks as i32 {
                bail!(
                    "M{} is {d} ticks from the rest pose (limit {}). Wrong port, or the arm is not at rest.",
                    j + 1,
                    cfg.safety.start_tolerance_ticks
                );
            }
        }
    }
    let wrist_ids: Vec<(u8, u8)> = WRIST.iter().map(|&j| (j as u8 + 1, cfg.bus.wrist_acc)).collect();
    arm.bus.sync_write_u8(RamRegister::Acc, &wrist_ids).await?;

    let mut cam = capture::start(&cfg.camera, Some(h), None)?;
    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    println!("moving to level...");
    arm.move_to(&level, cfg.poses.move_time_s).await?;
    println!("level. Put the ball on the paddle. Ctrl-C to stop and return to rest.");

    std::fs::create_dir_all(log_dir)?;
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_secs();
    let log_path = log_dir.join(format!("run_{stamp}.csv"));
    let mut log = std::io::BufWriter::new(std::fs::File::create(&log_path)?);
    writeln!(log, "t,state,seq,found,px,py,area,blobs,bx,by,vx,vy,tilt_x,tilt_y,u,v,g5,g6,g7,p5,p6,p7,i5,i6,i7,latency_ms,detect_ms")?;

    let mut wrist = WristCtl::new(&cfg, &level);
    let mut pid = BallPid::new(cfg.control.clone());
    let mut filt = AlphaBeta::new(cfg.control.alpha, cfg.control.beta);
    let mut state = State::WaitBall;
    let (mut seen, mut last_seen, mut last_t) = (0u32, Instant::now(), None::<Instant>);
    let mut uv = Vector2::zeros();
    let mut present = [0u16; 3];
    let mut current = [0f64; 3];
    let mut charge_mas = [0f64; 3];
    let mut bus_errors = 0;
    let mut no_frame_since: Option<Instant> = None;
    let t0 = Instant::now();
    let (mut balance_start, mut longest) = (None::<Instant>, 0f64);
    let mut last_status = Instant::now();

    let result: Result<()> = async {
        loop {
            let det = tokio::select! {
                _ = &mut ctrl_c => { println!("\nCtrl-C"); return Ok(()); }
                d = next_det(&mut cam.rx, Duration::from_secs_f64(cfg.safety.no_frame_s)) => d?,
            };
            let Some(det) = det else {
                // No frame: hold the last goal. Give up after 1 s.
                let since = *no_frame_since.get_or_insert_with(Instant::now);
                if since.elapsed() > Duration::from_secs(1) {
                    bail!("camera delivered no frame for 1 s");
                }
                continue;
            };
            no_frame_since = None;
            let dt = last_t.map_or(1.0 / cfg.camera.fps as f64, |l| det.t.saturating_duration_since(l).as_secs_f64());
            last_t = Some(det.t);

            let ball = det.blob.map(|b| h.apply(b.px));
            let accel = axes.accel_mm_s2(uv);
            if let Some(p) = ball {
                last_seen = det.t;
                seen += 1;
                filt.update(p, dt, if state == State::Balance { accel } else { Vector2::zeros() });
            } else {
                seen = 0;
            }

            match state {
                State::WaitBall => {
                    if seen >= cfg.safety.ball_confirm_frames {
                        println!("ball detected, balancing");
                        state = State::Balance;
                        balance_start = Some(Instant::now());
                        pid.reset();
                    }
                }
                State::Balance => {
                    let lost = det.t.saturating_duration_since(last_seen).as_secs_f64() > cfg.safety.ball_lost_s;
                    let edge = filt.pos.norm() > cfg.safety.edge_radius_mm;
                    if lost || edge {
                        let held = balance_start.map_or(0.0, |s| s.elapsed().as_secs_f64());
                        longest = longest.max(held);
                        println!("{} after {held:.1} s; back to level, waiting for the ball", if lost { "ball lost" } else { "ball at the edge" });
                        arm.move_to(&level, 0.7).await?;
                        wrist.reset();
                        pid.reset();
                        filt.reset();
                        uv = Vector2::zeros();
                        seen = 0;
                        state = State::WaitBall;
                        continue;
                    }
                    let ahead = cfg.control.predict_ms / 1e3;
                    let p = filt.predict(ahead, accel);
                    let v = filt.vel + accel * ahead;
                    let blade_tilt = pid.step(p, v, dt);
                    uv = axes.blade_to_uv(blade_tilt);
                    let goal = wrist.ticks_for(&cfg, uv);
                    let cur = wrist_of(&arm.cmd);
                    let step = (0..3).map(|k| (goal[k] as i32 - cur[k] as i32).abs()).max().unwrap_or(0);
                    if step >= cfg.control.min_goal_step_ticks as i32 {
                        arm.send_wrist(goal).await?;
                    }
                }
            }
            let latency_ms = det.t.elapsed().as_secs_f64() * 1e3;

            // Read the wrist servos: watchdog + energy metric.
            let mut ok = true;
            for k in 0..3 {
                match arm.bus.read_state(WRIST[k] as u8 + 1).await {
                    Ok(s) => {
                        present[k] = s.position;
                        current[k] = s.current_ma;
                        charge_mas[k] += s.current_ma * dt;
                    }
                    Err(e) => {
                        ok = false;
                        log::warn!("{e:#}");
                    }
                }
            }
            bus_errors = if ok { 0 } else { bus_errors + 1 };
            if bus_errors >= 3 {
                bail!("servo reads failed 3 ticks in a row");
            }

            let (px, area) = det.blob.map_or((Vector2::zeros(), 0), |b| (b.px, b.area));
            let b = ball.unwrap_or_else(Vector2::zeros);
            let g = wrist_of(&arm.cmd);
            writeln!(
                log,
                "{:.4},{:?},{},{},{:.1},{:.1},{},{},{:.2},{:.2},{:.1},{:.1},{:.5},{:.5},{:.5},{:.5},{},{},{},{},{},{},{:.0},{:.0},{:.0},{:.1},{:.1}",
                t0.elapsed().as_secs_f64(), state, det.seq, det.blob.is_some() as u8, px.x, px.y, area, det.n_blobs,
                b.x, b.y, filt.vel.x, filt.vel.y, pid.out.x, pid.out.y, uv.x, uv.y,
                g[0], g[1], g[2], present[0], present[1], present[2], current[0], current[1], current[2],
                latency_ms, det.detect_ms
            )?;
            if last_status.elapsed() > Duration::from_secs(2) {
                last_status = Instant::now();
                println!(
                    "{:?}: ball ({:.0}, {:.0}) mm  tilt ({:.1}, {:.1}) deg  wrist mA ({:.0}, {:.0}, {:.0})  latency {latency_ms:.0} ms",
                    state, filt.pos.x, filt.pos.y, pid.out.x.to_degrees(), pid.out.y.to_degrees(), current[0], current[1], current[2]
                );
            }
        }
    }
    .await;

    if state == State::Balance {
        longest = longest.max(balance_start.map_or(0.0, |s| s.elapsed().as_secs_f64()));
    }
    log.flush()?;
    if let Err(e) = &result {
        eprintln!("stopping: {e:#}");
    }
    println!("returning to rest...");
    arm.move_to(&start_pose, cfg.poses.move_time_s).await?;
    let el = t0.elapsed().as_secs_f64();
    println!(
        "longest balance {longest:.1} s; mean wrist current M5 {:.0} mA, M6 {:.0} mA, M7 {:.0} mA; log {}",
        charge_mas[0] / el,
        charge_mas[1] / el,
        charge_mas[2] / el,
        log_path.display()
    );
    result
}
