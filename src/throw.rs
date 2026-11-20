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

/// What happened during a throw move.
#[derive(Debug, Clone, Default)]
pub struct ThrowReport {
    pub move_s: f64,
    /// Peak |load| per joint (0..1000 = 0..100 %).
    pub peak_load: [i16; 7],
    /// Largest |goal - present| at the end, ticks.
    pub final_error: [i32; 7],
    pub aborted: Option<String>,
}

impl ThrowReport {
    pub fn summary(&self) -> String {
        let peak = self.peak_load.iter().enumerate().max_by_key(|(_, l)| **l).map(|(j, l)| (j + 1, *l)).unwrap_or((0, 0));
        match &self.aborted {
            Some(why) => format!("throw ABORTED after {:.2} s: {why}", self.move_s),
            None => format!(
                "throw done in {:.2} s; peak load M{} {:.0} %",
                self.move_s,
                peak.0,
                peak.1 as f64 / 10.0
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
    fn profile_times() {
        // Triangular: 100 steps at a = 100 -> 2 s.
        assert!((profile_time(100.0, 1000.0, 100.0) - 2.0).abs() < 1e-9);
        // Trapezoidal: 1000 steps, v 100, a 100 -> 10 + 1 s.
        assert!((profile_time(1000.0, 100.0, 100.0) - 11.0).abs() < 1e-9);
    }
}
