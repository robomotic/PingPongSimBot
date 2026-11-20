//! The balancing engine, independent of where frames come from and where goals go.
//!
//! - [`Balancer`]: per camera frame -> next [`Action`] + [`Telemetry`] (state machine, filter, PID, wrist solver).
//! - [`Plant`]: the robot side (camera + servos). [`RealPlant`] drives the arm; `sim::SimPlant` simulates it.
//! - [`run_loop`]: rest -> level -> wait for ball -> balance, with live commands ([`Cmd`]) and events ([`Event`]).
//!   Both the CLI `run` and the TUI use it.

use crate::arm::{joint_rad, wrist_of};
use crate::bus::MotorState;
use crate::config::{Config, ControlConfig, N_MOTORS, WRIST};
use crate::control::{AxesMap, Backlash, BallPid};
use crate::estimator::{AlphaBeta, Homography};
use crate::kinematics::WristModel;
use crate::vision::Detection;
use anyhow::{bail, Context, Result};
use nalgebra::Vector2;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------------------------------------------
// Wrist solver wrapper.

/// Blade tilt in -> clamped wrist ticks out (with slop compensation on the sent goal).
#[derive(Clone)]
pub struct WristCtl {
    pub model: WristModel,
    level_rad: [f64; 3],
    level_ticks: [u16; 3],
    range: [u16; 3],
    cmd_rad: [f64; 3],
    backlash: [Backlash; 3],
}

impl WristCtl {
    pub fn new(cfg: &Config, level: &[u16; N_MOTORS]) -> Self {
        let q = joint_rad(cfg, level);
        let level_rad = [q[4], q[5], q[6]];
        Self {
            model: WristModel::new(&q, cfg.joints.wrist_weight, cfg.control.damping),
            level_rad,
            level_ticks: wrist_of(level),
            range: cfg.joints.wrist_range_ticks,
            cmd_rad: level_rad,
            backlash: Default::default(),
        }
    }

    pub fn reset(&mut self) {
        self.cmd_rad = self.level_rad;
        for b in &mut self.backlash {
            b.reset();
        }
    }

    /// Solver tilt (u, v) in rad -> wrist goal ticks, clamped around the level pose.
    pub fn ticks_for(&mut self, cfg: &Config, uv: Vector2<f64>) -> [u16; 3] {
        let target = uv.map(f64::sin);
        let q = self.model.solve(&self.cmd_rad, target, &self.level_rad, cfg.control.null_space_gain);
        let mut out = [0u16; 3];
        for k in 0..3 {
            let j = WRIST[k];
            let lo = (self.level_ticks[k] as f64 - self.range[k] as f64).max(0.0);
            let hi = (self.level_ticks[k] as f64 + self.range[k] as f64).min(4095.0);
            let t = cfg.rad_to_tick(j, q[k]).clamp(lo, hi);
            // The model keeps the uncompensated angle; only the goal sent to the servo gets the slop offset.
            self.cmd_rad[k] = cfg.tick_to_rad(j, t);
            let sent = self.backlash[k].apply(t, cfg.control.backlash_ticks[k]);
            out[k] = sent.clamp(lo, hi).round() as u16;
        }
        out
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Balancer.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    WaitBall,
    Balance,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    /// Keep the current goals.
    Hold,
    SendWrist([u16; 3]),
    /// Smoothly back to level; call [`Balancer::returned_to_level`] afterwards.
    ReturnToLevel(String),
}

#[derive(Debug, Clone)]
pub struct Telemetry {
    pub t: Instant,
    pub seq: u64,
    pub phase: Phase,
    pub paused: bool,
    /// Raw ball position on the blade (mm), if seen this frame.
    pub ball: Option<Vector2<f64>>,
    pub ball_px: Option<Vector2<f64>>,
    pub area: u32,
    pub n_blobs: u32,
    pub detect_ms: f64,
    /// Filtered position / velocity (mm, mm/s).
    pub pos: Vector2<f64>,
    pub vel: Vector2<f64>,
    /// Commanded blade tilt (rad, blade frame) and solver tilt (u, v).
    pub tilt: Vector2<f64>,
    pub uv: Vector2<f64>,
    /// Seconds in the current balance (0 when waiting) and the longest so far.
    pub balance_s: f64,
    pub longest_s: f64,
    /// One-off message for this frame ("ball detected", "ball lost after 12.3 s", ...).
    pub event: Option<String>,
}

pub struct Balancer {
    pub cfg: Config,
    h: Homography,
    axes: AxesMap,
    pub wrist: WristCtl,
    pid: BallPid,
    filt: AlphaBeta,
    phase: Phase,
    paused: bool,
    seen: u32,
    last_seen: Option<Instant>,
    last_t: Option<Instant>,
    uv: Vector2<f64>,
    balance_start: Option<Instant>,
    longest: f64,
}

/// Image map and axes from the config (axes normalised if configured).
pub fn calibration(cfg: &Config) -> Result<(Homography, AxesMap)> {
    let h = cfg
        .camera
        .homography
        .map(|h| Homography::from_row_major(&h))
        .context("no homography in config: run `calib-camera` first")?;
    let axes = AxesMap::from_row_major(&cfg.camera.axes.context("no axes calibration: run `calib-axes` first")?);
    Ok((h, if cfg.camera.axes_normalize { axes.normalized() } else { axes }))
}

impl Balancer {
    pub fn new(cfg: Config) -> Result<Self> {
        let level = cfg.level()?;
        let (h, axes) = calibration(&cfg)?;
        Ok(Self {
            wrist: WristCtl::new(&cfg, &level),
            pid: BallPid::new(cfg.control.clone()),
            filt: AlphaBeta::new(cfg.control.alpha, cfg.control.beta),
            cfg,
            h,
            axes,
            phase: Phase::WaitBall,
            paused: false,
            seen: 0,
            last_seen: None,
            last_t: None,
            uv: Vector2::zeros(),
            balance_start: None,
            longest: 0.0,
        })
    }

    pub fn axes(&self) -> &AxesMap {
        &self.axes
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn longest(&self) -> f64 {
        let cur = if self.phase == Phase::Balance { self.balance_start.map_or(0.0, |s| s.elapsed().as_secs_f64()) } else { 0.0 };
        self.longest.max(cur)
    }

    /// Live tuning: new gains take effect on the next frame; the filter and PID keep their state.
    pub fn set_control(&mut self, c: ControlConfig) {
        self.pid.set_cfg(c.clone());
        self.filt.set_gains(c.alpha, c.beta);
        self.cfg.control = c;
    }

    pub fn set_paused(&mut self, p: bool) {
        self.paused = p;
    }

    /// Call after the plant finished a [`Action::ReturnToLevel`].
    pub fn returned_to_level(&mut self) {
        self.wrist.reset();
        self.pid.reset();
        self.filt.reset();
        self.uv = Vector2::zeros();
        self.seen = 0;
        self.phase = Phase::WaitBall;
    }

    fn end_balance(&mut self) -> f64 {
        let held = self.balance_start.take().map_or(0.0, |s| s.elapsed().as_secs_f64());
        self.longest = self.longest.max(held);
        held
    }

    /// `current_wrist` = goals the servos were last sent (to skip tiny changes).
    pub fn on_frame(&mut self, det: &Detection, current_wrist: [u16; 3]) -> (Action, Telemetry) {
        let c = &self.cfg;
        let dt = self.last_t.map_or(1.0 / c.camera.fps as f64, |l| det.t.saturating_duration_since(l).as_secs_f64());
        self.last_t = Some(det.t);
        let ball = det.blob.map(|b| self.h.apply(b.px));
        let accel = self.axes.accel_mm_s2(self.uv);
        if let Some(p) = ball {
            self.last_seen = Some(det.t);
            self.seen += 1;
            self.filt.update(p, dt, if self.phase == Phase::Balance { accel } else { Vector2::zeros() });
        } else {
            self.seen = 0;
        }

        let mut event = None;
        let mut action = Action::Hold;
        match self.phase {
            Phase::WaitBall => {
                if !self.paused && self.seen >= c.safety.ball_confirm_frames {
                    self.phase = Phase::Balance;
                    self.balance_start = Some(det.t);
                    self.pid.reset();
                    event = Some("ball detected, balancing".to_string());
                }
            }
            Phase::Balance => {
                let lost = self.last_seen.map_or(true, |s| det.t.saturating_duration_since(s).as_secs_f64() > c.safety.ball_lost_s);
                let edge = self.filt.pos.norm() > c.safety.edge_radius_mm;
                if lost || edge || self.paused {
                    let why = if self.paused { "paused" } else if lost { "ball lost" } else { "ball at the edge" };
                    let held = self.end_balance();
                    let msg = format!("{why} after {held:.1} s; back to level");
                    event = Some(msg.clone());
                    action = Action::ReturnToLevel(msg);
                } else {
                    let ahead = c.control.predict_ms / 1e3;
                    let p = self.filt.predict(ahead, accel);
                    let v = self.filt.vel + accel * ahead;
                    let blade_tilt = self.pid.step(p, v, dt);
                    self.uv = self.axes.blade_to_uv(blade_tilt);
                    let goal = self.wrist.ticks_for(&self.cfg, self.uv);
                    let step = (0..3).map(|k| (goal[k] as i32 - current_wrist[k] as i32).abs()).max().unwrap_or(0);
                    if step >= self.cfg.control.min_goal_step_ticks as i32 {
                        action = Action::SendWrist(goal);
                    }
                }
            }
        }
        let tele = Telemetry {
            t: det.t,
            seq: det.seq,
            phase: self.phase,
            paused: self.paused,
            ball,
            ball_px: det.blob.map(|b| b.px),
            area: det.blob.map_or(0, |b| b.area),
            n_blobs: det.n_blobs,
            detect_ms: det.detect_ms,
            pos: self.filt.pos,
            vel: self.filt.vel,
            tilt: self.pid.out,
            uv: self.uv,
            balance_s: self.balance_start.map_or(0.0, |s| det.t.saturating_duration_since(s).as_secs_f64()),
            longest_s: self.longest,
            event,
        };
        (action, tele)
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Plant: where frames come from and goals go.

#[allow(async_fn_in_trait)]
pub trait Plant {
    /// Short label for logs and the UI ("arm", "sim").
    fn kind(&self) -> &'static str;
    /// Pose of M1..M8 when the run started (returned to at the end).
    fn start_pose(&self) -> [u16; N_MOTORS];
    /// Last goals sent to M1..M7.
    fn cmd(&self) -> [u16; 7];
    /// Next camera detection, or None after `timeout`.
    async fn next_frame(&mut self, timeout: Duration) -> Result<Option<Detection>>;
    async fn send_wrist(&mut self, goals: [u16; 3]) -> Result<()>;
    async fn move_to(&mut self, target: &[u16], secs: f64) -> Result<()>;
    /// State of M5, M6, M7.
    async fn read_wrist(&mut self) -> Result<[MotorState; 3]>;
    /// Simulation only: put a ball on the paddle.
    fn place_ball(&mut self) {}
}

/// The real arm and camera. Linux only (V4L2).
#[cfg(target_os = "linux")]
pub struct RealPlant {
    arm: crate::arm::Arm,
    cam: crate::vision::Camera,
    start: [u16; N_MOTORS],
}

#[cfg(target_os = "linux")]
impl RealPlant {
    /// Opens the bus (all joints hold), checks the start pose against `rest`, sets the wrist acceleration and
    /// starts the camera. Nothing moves.
    pub async fn open(cfg: &Config) -> Result<Self> {
        Self::open_with(cfg, true, true).await
    }

    /// `check_rest`: refuse unless the arm is near the rest pose. `roi`: detect only on the blade (needs the camera map;
    /// the axes may still be missing, e.g. during axis calibration).
    pub async fn open_with(cfg: &Config, check_rest: bool, roi: bool) -> Result<Self> {
        use st3215::protocol::RamRegister;
        let h = if roi { cfg.camera.homography.map(|h| Homography::from_row_major(&h)) } else { None };
        let (mut arm, start) = crate::arm::Arm::open(cfg).await?;
        if let Some(rest) = cfg.poses.rest.filter(|_| check_rest) {
            for j in 0..7 {
                let d = (start[j] as i32 - rest[j] as i32).abs();
                if d > cfg.safety.start_tolerance_ticks as i32 {
                    bail!(
                        "M{} is {d} ticks from the rest pose (limit {}). Wrong port, or the arm is not at rest.",
                        j + 1,
                        cfg.safety.start_tolerance_ticks
                    );
                }
            }
        }
        let acc: Vec<(u8, u8)> = WRIST.iter().map(|&j| (j as u8 + 1, cfg.bus.wrist_acc)).collect();
        arm.bus.sync_write_u8(RamRegister::Acc, &acc).await?;
        let cam = crate::vision::capture::start(&cfg.camera, h, None)?;
        Ok(Self { arm, cam, start })
    }
}

#[cfg(target_os = "linux")]
impl Plant for RealPlant {
    fn kind(&self) -> &'static str {
        "arm"
    }
    fn start_pose(&self) -> [u16; N_MOTORS] {
        self.start
    }
    fn cmd(&self) -> [u16; 7] {
        self.arm.cmd
    }
    async fn next_frame(&mut self, timeout: Duration) -> Result<Option<Detection>> {
        match tokio::time::timeout(timeout, self.cam.rx.changed()).await {
            Ok(Ok(())) => Ok(Some(*self.cam.rx.borrow_and_update())),
            Ok(Err(_)) => bail!("camera thread stopped"),
            Err(_) => Ok(None),
        }
    }
    async fn send_wrist(&mut self, goals: [u16; 3]) -> Result<()> {
        self.arm.send_wrist(goals).await
    }
    async fn move_to(&mut self, target: &[u16], secs: f64) -> Result<()> {
        self.arm.move_to(target, secs).await
    }
    async fn read_wrist(&mut self) -> Result<[MotorState; 3]> {
        let mut out = [MotorState::default(); 3];
        for k in 0..3 {
            out[k] = self.arm.bus.read_state(WRIST[k] as u8 + 1).await?;
        }
        Ok(out)
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Run log (CSV). The active gains are on every row, so live tuning is visible in the analysis.

pub const LOG_HEADER: &str = "t,state,seq,found,px,py,area,blobs,bx,by,vx,vy,tilt_x,tilt_y,u,v,g5,g6,g7,p5,p6,p7,i5,i6,i7,latency_ms,detect_ms,kp,kd,bl5,bl6,bl7,deadband_mm";

pub struct RunLog {
    pub path: PathBuf,
    w: std::io::BufWriter<std::fs::File>,
    t0: Instant,
}

impl RunLog {
    pub fn create(dir: &Path, prefix: &str) -> Result<Self> {
        std::fs::create_dir_all(dir)?;
        let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_secs();
        let path = dir.join(format!("{prefix}_{stamp}.csv"));
        let mut w = std::io::BufWriter::new(std::fs::File::create(&path)?);
        writeln!(w, "{LOG_HEADER}")?;
        Ok(Self { path, w, t0: Instant::now() })
    }

    pub fn write(&mut self, s: &Snapshot) -> Result<()> {
        let t = &s.tele;
        let px = t.ball_px.unwrap_or_else(Vector2::zeros);
        let b = t.ball.unwrap_or_else(Vector2::zeros);
        let c = &s.control;
        writeln!(
            self.w,
            "{:.4},{:?},{},{},{:.1},{:.1},{},{},{:.2},{:.2},{:.1},{:.1},{:.5},{:.5},{:.5},{:.5},{},{},{},{},{},{},{:.0},{:.0},{:.0},{:.1},{:.1},{},{},{},{},{},{}",
            t.t.saturating_duration_since(self.t0).as_secs_f64(),
            t.phase, t.seq, t.ball.is_some() as u8, px.x, px.y, t.area, t.n_blobs,
            b.x, b.y, t.vel.x, t.vel.y, t.tilt.x, t.tilt.y, t.uv.x, t.uv.y,
            s.goal[0], s.goal[1], s.goal[2], s.present[0], s.present[1], s.present[2],
            s.current[0], s.current[1], s.current[2], s.latency_ms, t.detect_ms,
            c.kp, c.kd, c.backlash_ticks[0], c.backlash_ticks[1], c.backlash_ticks[2], c.deadband_mm
        )?;
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.w.flush()?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Run loop.

/// Commands into a running loop (from the TUI, or Ctrl-C in the CLI).
#[derive(Debug, Clone)]
pub enum Cmd {
    SetControl(ControlConfig),
    Pause(bool),
    PlaceBall,
    Stop,
}

/// One frame of everything the UI and the log need.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub tele: Telemetry,
    pub goal: [u16; 3],
    pub present: [u16; 3],
    pub current: [f64; 3],
    pub latency_ms: f64,
    pub control: ControlConfig,
}

#[derive(Debug, Clone)]
pub struct Summary {
    pub longest_s: f64,
    pub duration_s: f64,
    pub mean_current_ma: [f64; 3],
    pub log: PathBuf,
    pub error: Option<String>,
}

#[derive(Debug, Clone)]
pub enum Event {
    Status(String),
    Frame(Box<Snapshot>),
    Finished(Summary),
}

/// Move to level, balance until [`Cmd::Stop`] or an error, then return to the start pose.
/// Every frame is logged and reported through `emit`.
pub async fn run_loop<P: Plant>(
    plant: &mut P,
    mut bal: Balancer,
    log_dir: &Path,
    cmds: &mut tokio::sync::mpsc::UnboundedReceiver<Cmd>,
    mut emit: impl FnMut(Event),
) -> Result<Summary> {
    let level = bal.cfg.level()?;
    let mut log = RunLog::create(log_dir, if plant.kind() == "sim" { "sim" } else { "run" })?;
    emit(Event::Status(format!("moving to level ({:.0} s)...", bal.cfg.poses.move_time_s)));
    plant.move_to(&level, bal.cfg.poses.move_time_s).await?;
    emit(Event::Status("level: put the ball on the paddle".into()));

    let t0 = Instant::now();
    let mut charge = [0f64; 3];
    let mut last_frame = Instant::now();
    let mut bus_errors = 0;
    let mut no_frame_since: Option<Instant> = None;
    let mut wrist_state = [MotorState::default(); 3];
    let no_frame = Duration::from_secs_f64(bal.cfg.safety.no_frame_s);

    let result: Result<()> = async {
        loop {
            let det = tokio::select! {
                c = cmds.recv() => {
                    match c {
                        None | Some(Cmd::Stop) => return Ok(()),
                        Some(Cmd::SetControl(c)) => { bal.set_control(c); emit(Event::Status("gains updated".into())); }
                        Some(Cmd::Pause(p)) => { bal.set_paused(p); emit(Event::Status(if p { "paused: holding level".into() } else { "resumed: put the ball on the paddle".into() })); }
                        Some(Cmd::PlaceBall) => plant.place_ball(),
                    }
                    continue;
                }
                d = plant.next_frame(no_frame) => d?,
            };
            let Some(det) = det else {
                let since = *no_frame_since.get_or_insert_with(Instant::now);
                if since.elapsed() > Duration::from_secs(1) {
                    bail!("camera delivered no frame for 1 s");
                }
                continue;
            };
            no_frame_since = None;
            let dt = last_frame.elapsed().as_secs_f64();
            last_frame = Instant::now();

            let (action, tele) = bal.on_frame(&det, wrist_of(&plant.cmd()));
            match &action {
                Action::Hold => {}
                Action::SendWrist(g) => plant.send_wrist(*g).await?,
                Action::ReturnToLevel(_) => {}
            }
            let latency_ms = det.t.elapsed().as_secs_f64() * 1e3;
            match plant.read_wrist().await {
                Ok(s) => {
                    wrist_state = s;
                    bus_errors = 0;
                    for k in 0..3 {
                        charge[k] += s[k].current_ma * dt;
                    }
                }
                Err(e) => {
                    bus_errors += 1;
                    log::warn!("{e:#}");
                    if bus_errors >= 3 {
                        bail!("servo reads failed 3 frames in a row");
                    }
                }
            }
            let snap = Snapshot {
                tele,
                goal: wrist_of(&plant.cmd()),
                present: [wrist_state[0].position, wrist_state[1].position, wrist_state[2].position],
                current: [wrist_state[0].current_ma, wrist_state[1].current_ma, wrist_state[2].current_ma],
                latency_ms,
                control: bal.cfg.control.clone(),
            };
            log.write(&snap)?;
            if let Some(e) = &snap.tele.event {
                emit(Event::Status(e.clone()));
            }
            emit(Event::Frame(Box::new(snap)));
            if let Action::ReturnToLevel(_) = action {
                plant.move_to(&level, 0.7).await?;
                bal.returned_to_level();
                emit(Event::Status("level: put the ball on the paddle".into()));
            }
        }
    }
    .await;

    log.flush()?;
    let longest = bal.longest();
    let duration = t0.elapsed().as_secs_f64();
    emit(Event::Status("returning to the start pose...".into()));
    let back = plant.move_to(&plant.start_pose(), bal.cfg.poses.move_time_s).await;
    let summary = Summary {
        longest_s: longest,
        duration_s: duration,
        mean_current_ma: [charge[0] / duration, charge[1] / duration, charge[2] / duration],
        log: log.path.clone(),
        error: result.as_ref().err().map(|e| format!("{e:#}")).or_else(|| back.as_ref().err().map(|e| format!("{e:#}"))),
    };
    emit(Event::Finished(summary.clone()));
    Ok(summary)
}
