//! "Throw back": one fast move from the level pose to a taught end pose (blade about upright, face toward the camera
//! side). The servos run the move themselves: each moving joint gets its goal, a speed limit and an acceleration
//! (ST3215 Goal Speed / Acc registers), scaled by how far it moves so that all joints start and arrive together.
//! Nothing is streamed during the move, so the bus and camera latency don't matter.
//!
//! Checked in MuJoCo first (tools/throw_sim.py): at full speed the ball leaves at ~2 m/s, 47° up, peaks ~55 cm above
//! the table and lands ~50 cm on the camera side. Below ~75 % speed it hits the camera or rolls into the jaws, so
//! slow previews are for the arm alone (no ball).

use crate::config::Config;
use anyhow::{bail, Context, Result};
use nalgebra::Vector2;
use std::io::Write;
use std::path::Path;

/// One throw, ready for the bus.
#[derive(Debug, Clone, PartialEq)]
pub struct ThrowPlan {
    /// Goals of M1..M7 (joints that don't move keep their start goal).
    pub goals: [u16; 7],
    /// Which joints move.
    pub moving: [bool; 7],
    /// Goal Speed register (steps/s) per joint.
    pub speed: [u16; 7],
    /// Acc register (unit 100 steps/s^2) per joint.
    pub acc: [u8; 7],
    /// Fraction of full speed this plan was made for.
    pub speed_frac: f64,
    /// Expected duration of the move (s), from the trapezoidal profile of the slowest joint.
    pub duration_s: f64,
}

/// Duration of a trapezoidal (or triangular) move over `dist` with speed limit `v` and acceleration `a`.
pub fn profile_time(dist: f64, v: f64, a: f64) -> f64 {
    if dist <= 0.0 || v <= 0.0 || a <= 0.0 {
        return 0.0;
    }
    let ramp = v * v / a; // distance spent speeding up + slowing down
    if dist >= ramp {
        dist / v + v / a
    } else {
        2.0 * (dist / a).sqrt()
    }
}

/// Plan a throw from `from` (normally the level pose) at `speed_frac` of full speed. Speeds scale linearly with
/// `speed_frac` and accelerations with its square, so a slower plan follows the same path, just slower.
pub fn plan(cfg: &Config, from: &[u16], speed_frac: f64) -> Result<ThrowPlan> {
    let t = &cfg.throw;
    let end = t.end.context("no throw end pose: teach it on the Pose screen (T)")?;
    let level = cfg.level()?;
    if !(0.05..=1.0).contains(&speed_frac) {
        bail!("throw speed {speed_frac} is outside 0.05..1.0");
    }
    for j in 0..7 {
        let d = (end[j] as i32 - level[j] as i32).abs();
        if d > t.limit_ticks[j] as i32 {
            bail!(
                "throw end pose: M{} is {d} ticks from level (limit {}, throw.limit_ticks)",
                j + 1,
                t.limit_ticks[j]
            );
        }
    }
    let mut goals = [0u16; 7];
    goals.copy_from_slice(&from[..7]);
    let mut moving = [false; 7];
    let mut dist = [0f64; 7];
    for j in 0..7 {
        let d = end[j] as i32 - from[j] as i32;
        if d.unsigned_abs() >= t.min_joint_ticks as u32 {
            moving[j] = true;
            goals[j] = end[j];
            dist[j] = d.unsigned_abs() as f64;
        }
    }
    let longest = dist.iter().cloned().fold(0.0, f64::max);
    if longest == 0.0 {
        bail!("the arm is already at the throw end pose");
    }
    let v_full = t.max_speed_steps as f64 * speed_frac;
    let a_full = t.acc as f64 * speed_frac * speed_frac;
    let mut speed = [0u16; 7];
    let mut acc = [0u8; 7];
    let mut duration_s: f64 = 0.0;
    for j in 0..7 {
        if !moving[j] {
            continue;
        }
        let k = dist[j] / longest;
        speed[j] = (v_full * k).round().clamp(1.0, 4000.0) as u16;
        acc[j] = (a_full * k).round().clamp(1.0, 254.0) as u8;
        duration_s = duration_s.max(profile_time(dist[j], speed[j] as f64, acc[j] as f64 * 100.0));
    }
    Ok(ThrowPlan { goals, moving, speed, acc, speed_frac, duration_s })
}

/// One read of the moving joints during a throw (joints that don't move are 0).
#[derive(Debug, Clone, Copy, Default)]
pub struct ServoSample {
    /// Seconds since the goals were sent.
    pub t: f64,
    pub pos: [u16; 7],
    pub load: [i16; 7],
}

/// One camera frame during a throw. The camera is fixed to the gripper, so while the ball is on the blade its
/// position maps to blade mm with the usual homography.
#[derive(Debug, Clone, Copy)]
pub struct CamSample {
    pub t: f64,
    pub px: Option<Vector2<f64>>,
    pub area: u32,
}

/// What happened during a throw move.
#[derive(Debug, Clone, Default)]
pub struct ThrowReport {
    pub move_s: f64,
    /// Peak |load| per joint (0..1000 = 0..100 %).
    pub peak_load: [i16; 7],
    /// Largest |goal - present| at the end, ticks.
    pub final_error: [i32; 7],
    pub aborted: Option<String>,
    /// Servo reads during the move and for `throw.record_after_s` after it.
    pub servo: Vec<ServoSample>,
    /// Camera frames over the same time (real arm only).
    pub cam: Vec<CamSample>,
    /// Torque limit during the throw: (value before, value read back after raising it).
    pub torque_limit: Option<(u16, u16)>,
}

pub const THROW_LOG_HEADER: &str = "t,src,pos1,pos2,pos3,pos4,pos5,pos6,pos7,load1,load2,load3,load4,load5,load6,load7,\
goal1,goal2,goal3,goal4,goal5,goal6,goal7,px,py,bx,by,area";

/// Write a throw log: one `servo` row per bus read and one `cam` row per camera frame, both against the time the
/// goals were sent. `to_mm` maps a pixel to blade mm (valid while the ball is on the blade).
pub fn write_log(path: &Path, plan: &ThrowPlan, rep: &ThrowReport, to_mm: impl Fn(Vector2<f64>) -> Vector2<f64>) -> Result<()> {
    let mut w = std::io::BufWriter::new(std::fs::File::create(path).with_context(|| format!("creating {}", path.display()))?);
    writeln!(w, "{THROW_LOG_HEADER}")?;
    let goals: Vec<String> = plan.goals.iter().zip(plan.moving).map(|(g, m)| if m { g.to_string() } else { String::new() }).collect();
    let goals = goals.join(",");
    let mut rows: Vec<(f64, String)> = Vec::with_capacity(rep.servo.len() + rep.cam.len());
    for s in &rep.servo {
        let pos: Vec<String> = (0..7).map(|j| if plan.moving[j] { s.pos[j].to_string() } else { String::new() }).collect();
        let load: Vec<String> = (0..7).map(|j| if plan.moving[j] { s.load[j].to_string() } else { String::new() }).collect();
        rows.push((s.t, format!("{:.4},servo,{},{},{goals},,,,,", s.t, pos.join(","), load.join(","))));
    }
    for c in &rep.cam {
        let blank = ",".repeat(21); // 7 positions + 7 loads + 7 goals
        let ball = match c.px {
            Some(p) => {
                let b = to_mm(p);
                format!("{:.1},{:.1},{:.2},{:.2},{}", p.x, p.y, b.x, b.y, c.area)
            }
            None => ",,,,0".to_string(),
        };
        rows.push((c.t, format!("{:.4},cam,{blank}{ball}", c.t)));
    }
    rows.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    for (_, r) in rows {
        writeln!(w, "{r}")?;
    }
    w.flush()?;
    Ok(())
}

impl ThrowReport {
    pub fn summary(&self) -> String {
        let peak = self.peak_load.iter().enumerate().max_by_key(|(_, l)| **l).map(|(j, l)| (j + 1, *l)).unwrap_or((0, 0));
        match &self.aborted {
            Some(why) => format!("throw ABORTED after {:.2} s: {why}", self.move_s),
            None => format!(
                "throw done in {:.2} s; peak load M{} {:.0} %{}",
                self.move_s,
                peak.0,
                peak.1 as f64 / 10.0,
                match self.torque_limit {
                    Some((old, now)) => format!("; torque limit {:.0} % (was {:.0} %)", now as f64 / 10.0, old as f64 / 10.0),
                    None => String::new(),
                }
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config::example()
    }

    #[test]
    fn plan_moves_only_the_joints_that_change_and_arrives_together() {
        let c = cfg();
        let level = c.level().unwrap();
        let p = plan(&c, &level, 1.0).unwrap();
        // Taught end: M3 -179, M4 -54, M6 -655; the rest is hand noise (< 20 ticks).
        assert_eq!(p.moving, [false, false, true, true, false, true, false]);
        assert_eq!(p.goals[0], level[0]);
        assert_eq!(p.goals[5], c.throw.end.unwrap()[5]);
        // M6 moves furthest: full speed and acceleration.
        assert_eq!(p.speed[5], c.throw.max_speed_steps);
        assert_eq!(p.acc[5], c.throw.acc);
        // Speeds in proportion to distance.
        let r = p.speed[2] as f64 / p.speed[5] as f64;
        assert!((r - 179.0 / 655.0).abs() < 0.01, "{r}");
        // Matches the MuJoCo move time (0.33 s) roughly.
        assert!(p.duration_s > 0.25 && p.duration_s < 0.45, "{}", p.duration_s);
    }

    #[test]
    fn slow_plan_follows_the_same_path_more_slowly() {
        let c = cfg();
        let level = c.level().unwrap();
        let fast = plan(&c, &level, 1.0).unwrap();
        let slow = plan(&c, &level, 0.25).unwrap();
        assert_eq!(fast.goals, slow.goals);
        assert!(slow.duration_s > 3.0 * fast.duration_s);
        assert!(slow.acc.iter().zip(&slow.moving).all(|(a, m)| !m || *a >= 1));
    }

    #[test]
    fn plan_refuses_an_end_pose_outside_the_box() {
        let mut c = cfg();
        let mut end = c.throw.end.unwrap();
        end[1] += 300; // shoulder
        c.throw.end = Some(end);
        let level = c.level().unwrap();
        let e = plan(&c, &level, 1.0).unwrap_err().to_string();
        assert!(e.contains("M2"), "{e}");
    }

    #[test]
    fn throw_log_has_one_row_per_sample_and_fixed_columns() {
        let c = cfg();
        let level = c.level().unwrap();
        let p = plan(&c, &level, 1.0).unwrap();
        let mut rep = ThrowReport::default();
        rep.servo.push(ServoSample { t: 0.01, pos: p.goals, load: [100; 7] });
        rep.cam.push(CamSample { t: 0.02, px: Some(Vector2::new(320.0, 240.0)), area: 2900 });
        rep.cam.push(CamSample { t: 0.05, px: None, area: 0 });
        let path = std::env::temp_dir().join(format!("throw_test_{}.csv", std::process::id()));
        write_log(&path, &p, &rep, |px| px / 10.0).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        std::fs::remove_file(&path).ok();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 4);
        let n = lines[0].split(',').count();
        assert!(lines.iter().all(|l| l.split(',').count() == n), "{text}");
        assert!(lines[1].contains(",servo,") && lines[2].contains(",cam,") && lines[2].contains("32.00,24.00"));
    }

    #[test]
    fn profile_times() {
        // Triangular: 100 steps at a = 100 -> 2 s.
        assert!((profile_time(100.0, 1000.0, 100.0) - 2.0).abs() < 1e-9);
        // Trapezoidal: 1000 steps, v 100, a 100 -> 10 + 1 s.
        assert!((profile_time(1000.0, 100.0, 100.0) - 11.0).abs() < 1e-9);
    }
}
