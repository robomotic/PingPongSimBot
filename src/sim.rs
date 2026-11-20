//! Simulated arm + camera for trying the controller and the TUI without the robot.
//!
//! The plant is fitted to the first real run (data/run_1795119100.csv, see tools/tune_sim.py):
//! - goals reach the servos 3 frames late, and each servo follows with a first-order lag (tau 0.1 s);
//! - each servo has a dead zone (ticks): it only moves when the goal is more than half of it away
//!   (M6 ~20 ticks: the measured tracking error);
//! - the blade tilt comes from the servos' positions through the same wrist kinematics the controller uses,
//!   and the ball accelerates at `eff` x the ideal 0.6 g theta along the calibrated axes;
//! - the ball is seen through the inverse of the calibrated homography, with pixel noise.
//! Frames are produced in real time at `fps`.

use crate::arm::{joint_rad, wrist_of};
use crate::balance::{calibration, Plant};
use crate::bus::MotorState;
use crate::config::{Config, N_MOTORS, WRIST};
use crate::control::{AxesMap, BALL_ACCEL_PER_RAD};
use crate::estimator::Homography;
use crate::kinematics::WristModel;
use crate::vision::{Blob, Detection};
use crate::config::ThrowConfig;
use crate::throw::{ThrowPlan, ThrowReport};
use anyhow::{Context, Result};
use nalgebra::Vector2;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct SimParams {
    pub fps: f64,
    pub delay_frames: usize,
    pub servo_tau_s: f64,
    pub dead_zone_ticks: [f64; 3],
    /// Fraction of the ideal ball acceleration that the real blade achieves.
    pub eff: f64,
    pub noise_px: f64,
    /// Blade radius (mm): the ball falls off beyond it.
    pub blade_radius_mm: f64,
}

impl Default for SimParams {
    fn default() -> Self {
        Self {
            fps: 29.0,
            delay_frames: 3,
            servo_tau_s: 0.1,
            dead_zone_ticks: [3.0, 20.0, 6.0],
            eff: 0.5,
            noise_px: 0.3,
            blade_radius_mm: 75.0,
        }
    }
}

/// Small deterministic RNG (xorshift64*), so the simulator needs no extra crate.
#[derive(Debug, Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn normal(&mut self) -> f64 {
        let (u1, u2) = (self.uniform().max(1e-12), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

pub struct SimPlant {
    cfg: Config,
    pub params: SimParams,
    model: WristModel,
    axes: AxesMap,
    to_px: Homography,
    cmd: [u16; 7],
    start: [u16; N_MOTORS],
    queue: VecDeque<[f64; 3]>,
    target: [f64; 3],
    present: [f64; 3],
    ball: Option<(Vector2<f64>, Vector2<f64>)>,
    rng: Rng,
    next_at: Instant,
    seq: u64,
}

impl SimPlant {
    /// The simulated arm starts at the rest pose (or level) with no ball.
    pub fn new(cfg: &Config, params: SimParams, seed: u64) -> Result<Self> {
        let level = cfg.level()?;
        let (h, axes) = calibration(cfg)?;
        let start = cfg.poses.rest.unwrap_or(level);
        let q = joint_rad(cfg, &level);
        let mut cmd = [0u16; 7];
        cmd.copy_from_slice(&start[..7]);
        let w = wrist_of(&start).map(|t| t as f64);
        Ok(Self {
            model: WristModel::new(&q, cfg.joints.wrist_weight, cfg.control.damping),
            axes,
            to_px: h.inverse().context("homography is not invertible")?,
            cfg: cfg.clone(),
            params,
            cmd,
            start,
            queue: VecDeque::new(),
            target: w,
            present: w,
            ball: None,
            rng: Rng::new(seed),
            next_at: Instant::now(),
            seq: 0,
        })
    }

    pub fn ball(&self) -> Option<Vector2<f64>> {
        self.ball.map(|(p, _)| p)
    }

    /// Blade tilt (in-plane gravity, sin) from the servos' present positions.
    fn blade_accel(&self) -> Vector2<f64> {
        let mut wrist = [0.0; 3];
        for k in 0..3 {
            wrist[k] = self.cfg.tick_to_rad(WRIST[k], self.present[k]);
        }
        let uv = self.model.tilt(&wrist);
        self.axes.0 * uv * BALL_ACCEL_PER_RAD * self.params.eff
    }

    /// Advance one camera frame.
    pub fn step(&mut self) {
        let p = &self.params;
        let dt = 1.0 / p.fps;
        // Goals reach the servos `delay_frames` late.
        self.queue.push_back(wrist_of(&self.cmd).map(|t| t as f64));
        while self.queue.len() > p.delay_frames {
            self.target = self.queue.pop_front().unwrap();
        }
        // Dead zone + first-order lag.
        for k in 0..3 {
            let err = self.target[k] - self.present[k];
            let half = p.dead_zone_ticks[k] / 2.0;
            if err.abs() > half {
                let aim = self.target[k] - err.signum() * half;
                self.present[k] += (aim - self.present[k]) * (dt / p.servo_tau_s).min(1.0);
            }
        }
        // Ball physics, 10 substeps.
        if let Some((mut pos, mut vel)) = self.ball {
            let a = self.blade_accel();
            for _ in 0..10 {
                vel += a * (dt / 10.0);
                pos += vel * (dt / 10.0);
            }
            self.ball = if pos.norm() > p.blade_radius_mm { None } else { Some((pos, vel)) };
        }
    }

    fn detection(&mut self) -> Detection {
        self.seq += 1;
        let blob = self.ball.map(|(p, _)| {
            let px = self.to_px.apply(p);
            let n = self.params.noise_px;
            Blob {
                px: px + Vector2::new(self.rng.normal() * n, self.rng.normal() * n),
                area: 2900,
                hsv: [15.0, 150.0, 130.0],
            }
        });
        Detection { seq: self.seq, t: Instant::now(), blob, n_blobs: blob.is_some() as u32, detect_ms: 0.2 }
    }
}

impl Plant for SimPlant {
    fn kind(&self) -> &'static str {
        "sim"
    }
    fn start_pose(&self) -> [u16; N_MOTORS] {
        self.start
    }
    fn cmd(&self) -> [u16; 7] {
        self.cmd
    }
    async fn next_frame(&mut self, _timeout: Duration) -> Result<Option<Detection>> {
        let period = Duration::from_secs_f64(1.0 / self.params.fps);
        let now = Instant::now();
        if self.next_at < now {
            self.next_at = now;
        }
        tokio::time::sleep_until(self.next_at.into()).await;
        self.next_at += period;
        self.step();
        Ok(Some(self.detection()))
    }
    async fn send_wrist(&mut self, goals: [u16; 3]) -> Result<()> {
        for (k, &j) in WRIST.iter().enumerate() {
            self.cmd[j] = goals[k];
        }
        Ok(())
    }
    async fn move_to(&mut self, target: &[u16], secs: f64) -> Result<()> {
        tokio::time::sleep(Duration::from_secs_f64(secs.min(1.5))).await;
        self.cmd.copy_from_slice(&target[..7]);
        let w = wrist_of(&self.cmd).map(|t| t as f64);
        self.queue.clear();
        self.target = w;
        self.present = w;
        if let Some((p, _)) = self.ball {
            self.ball = Some((p, Vector2::zeros()));
        }
        self.next_at = Instant::now();
        Ok(())
    }
    async fn read_wrist(&mut self) -> Result<[MotorState; 3]> {
        let mut out = [MotorState::default(); 3];
        for k in 0..3 {
            let err = (self.target[k] - self.present[k]).abs();
            out[k] = MotorState {
                position: self.present[k].round() as u16,
                current_ma: 2.0 + 0.6 * err + if k == 1 { 8.0 } else { 0.0 },
                ..Default::default()
            };
        }
        Ok(out)
    }
    async fn throw(&mut self, plan: &ThrowPlan, _cfg: &ThrowConfig) -> Result<ThrowReport> {
        // The simulated blade can't fling the ball (see tools/throw_sim.py for the physics): the move just takes
        // the planned time and the ball is gone.
        tokio::time::sleep(Duration::from_secs_f64(plan.duration_s)).await;
        for j in 0..7 {
            if plan.moving[j] {
                self.cmd[j] = plan.goals[j];
            }
        }
        let w = wrist_of(&self.cmd).map(|t| t as f64);
        self.queue.clear();
        self.target = w;
        self.present = w;
        self.ball = None;
        self.next_at = Instant::now();
        let mut peak = [0i16; 7];
        for j in 0..7 {
            if plan.moving[j] {
                peak[j] = (150.0 + 300.0 * plan.speed_frac) as i16;
            }
        }
        Ok(ThrowReport { move_s: plan.duration_s, peak_load: peak, ..Default::default() })
    }
    fn place_ball(&mut self) {
        let r = 15.0 + 15.0 * self.rng.uniform();
        let a = 2.0 * std::f64::consts::PI * self.rng.uniform();
        self.ball = Some((Vector2::new(r * a.cos(), r * a.sin()), Vector2::zeros()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::balance::{Action, Balancer};

    /// The real controller, on the simulated plant, keeps the ball on the blade for 20 s.
    #[test]
    fn controller_balances_the_simulated_ball() {
        let cfg = Config::example();
        let mut sim = SimPlant::new(&cfg, SimParams::default(), 7).unwrap();
        let level = cfg.level().unwrap();
        sim.cmd.copy_from_slice(&level[..7]);
        let w = wrist_of(&level).map(|t| t as f64);
        sim.target = w;
        sim.present = w;
        sim.place_ball();
        let mut bal = Balancer::new(cfg.clone()).unwrap();
        let t0 = Instant::now();
        let dt = Duration::from_secs_f64(1.0 / sim.params.fps);
        let mut offsets = Vec::new();
        for i in 0..(29 * 20) {
            sim.step();
            let mut det = sim.detection();
            det.t = t0 + dt * i;
            let (action, tele) = bal.on_frame(&det, wrist_of(&sim.cmd));
            match action {
                Action::SendWrist(g) => {
                    for (k, &j) in WRIST.iter().enumerate() {
                        sim.cmd[j] = g[k];
                    }
                }
                Action::ReturnToLevel(m) => panic!("lost the ball at frame {i}: {m}"),
                Action::Hold => {}
            }
            if i > 29 * 5 {
                offsets.push(tele.pos.norm());
            }
        }
        assert!(sim.ball().is_some(), "ball fell off");
        offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = offsets[offsets.len() / 2];
        assert!(median < 30.0, "median offset {median:.1} mm");
    }
}
