//! Ball-position PID in blade coordinates, and the map between blade axes (camera) and solver tilt axes (u, v).

use crate::config::ControlConfig;
use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};

/// Hollow-shell ball rolling without slip: a = g sin(theta) / (1 + 2/3) = 0.6 g theta. In mm/s^2 per rad.
pub const BALL_ACCEL_PER_RAD: f64 = 0.6 * 9810.0;

/// `calib-axes` result: column k = ball acceleration (mm/s^2, blade frame) per BALL_ACCEL_PER_RAD and per rad of
/// tilt along solver axis k. Ideally a rotation (or reflection); its scale absorbs rolling friction and map errors.
#[derive(Debug, Clone, Copy)]
pub struct AxesMap(pub Matrix2<f64>);

impl AxesMap {
    pub fn from_row_major(a: &[f64; 4]) -> Self {
        Self(Matrix2::new(a[0], a[1], a[2], a[3]))
    }

    pub fn to_row_major(&self) -> [f64; 4] {
        [self.0[(0, 0)], self.0[(0, 1)], self.0[(1, 0)], self.0[(1, 1)]]
    }

    /// Same directions, unit length.
    pub fn normalized(&self) -> Self {
        let mut m = self.0;
        for k in 0..2 {
            let n = m.column(k).norm();
            if n > 1e-9 {
                m.set_column(k, &(m.column(k) / n));
            }
        }
        Self(m)
    }

    /// Solver tilt (u, v) that produces the wanted blade-frame "tilt" (acceleration / BALL_ACCEL_PER_RAD).
    pub fn blade_to_uv(&self, blade: Vector2<f64>) -> Vector2<f64> {
        self.0.try_inverse().map(|inv| inv * blade).unwrap_or_else(Vector2::zeros)
    }

    /// Expected ball acceleration in blade mm/s^2 for a solver tilt (u, v).
    pub fn accel_mm_s2(&self, uv: Vector2<f64>) -> Vector2<f64> {
        self.0 * uv * BALL_ACCEL_PER_RAD
    }

    /// Angle between the two measured axes, in degrees (90 = orthogonal, as the model expects).
    pub fn axes_angle_deg(&self) -> f64 {
        let (a, b) = (self.0.column(0), self.0.column(1));
        (a.dot(&b) / (a.norm() * b.norm())).clamp(-1.0, 1.0).acos().to_degrees()
    }
}

#[derive(Debug, Clone)]
pub struct BallPid {
    cfg: ControlConfig,
    integral: Vector2<f64>,
    /// Last commanded blade tilt (rad, blade frame).
    pub out: Vector2<f64>,
}

impl BallPid {
    pub fn new(cfg: ControlConfig) -> Self {
        Self { cfg, integral: Vector2::zeros(), out: Vector2::zeros() }
    }

    pub fn reset(&mut self) {
        self.integral = Vector2::zeros();
        self.out = Vector2::zeros();
    }

    /// New gains/limits; the integral and the last output are kept (live tuning).
    pub fn set_cfg(&mut self, cfg: ControlConfig) {
        self.cfg = cfg;
    }

    /// Returns the wanted blade tilt (rad, blade frame; positive = ball accelerates along +axis).
    /// `pos` in mm from the blade centre, `vel` in mm/s.
    pub fn step(&mut self, pos: Vector2<f64>, vel: Vector2<f64>, dt: f64) -> Vector2<f64> {
        let c = &self.cfg;
        // Energy saver: ball centred and nearly still -> keep the present tilt, send nothing new.
        if pos.norm() < c.deadband_mm && vel.norm() < c.deadband_speed_mm_s {
            return self.out;
        }
        let mut raw = -(pos * c.kp + vel * c.kd + self.integral * c.ki);
        let max = c.max_tilt_deg.to_radians();
        let saturated = raw.norm() > max;
        if saturated {
            raw *= max / raw.norm();
        } else {
            // Anti-windup: integrate only while not saturated.
            self.integral += pos * dt;
            if c.ki > 0.0 {
                let lim = c.i_limit_rad / c.ki;
                if self.integral.norm() > lim {
                    self.integral *= lim / self.integral.norm();
                }
            }
        }
        let step_max = c.max_tilt_rate_deg_s.to_radians() * dt;
        let d = raw - self.out;
        self.out += if d.norm() > step_max { d * (step_max / d.norm()) } else { d };
        self.out
    }
}

/// Least-squares fit of p(t) = p0 + v0 t + a t^2 / 2 per axis; returns a (mm/s^2).
pub fn fit_acceleration(samples: &[(f64, Vector2<f64>)]) -> Option<Vector2<f64>> {
    if samples.len() < 5 {
        return None;
    }
    let mut ata = Matrix3::zeros();
    let mut atx = Vector3::zeros();
    let mut aty = Vector3::zeros();
    for &(t, p) in samples {
        let row = Vector3::new(1.0, t, 0.5 * t * t);
        ata += row * row.transpose();
        atx += row * p.x;
        aty += row * p.y;
    }
    let inv = ata.try_inverse()?;
    Some(Vector2::new((inv * atx)[2], (inv * aty)[2]))
}

/// Per-joint slop (backlash / dead zone) compensation. Tracks the direction each joint last moved in and offsets
/// its goal by half the slop in that direction, so a reversal immediately crosses the dead zone.
#[derive(Debug, Clone, Default)]
pub struct Backlash {
    last: Option<f64>,
    dir: f64,
}

impl Backlash {
    /// `target` = uncompensated goal (ticks); returns the goal to send.
    pub fn apply(&mut self, target: f64, slop: f64) -> f64 {
        if let Some(last) = self.last {
            let d = target - last;
            if d.abs() >= 1.0 {
                self.dir = d.signum();
            }
        }
        self.last = Some(target);
        target + self.dir * slop / 2.0
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Minimum-jerk interpolation factor for tau in [0, 1].
pub fn min_jerk(tau: f64) -> f64 {
    let t = tau.clamp(0.0, 1.0);
    t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> ControlConfig {
        ControlConfig { max_tilt_rate_deg_s: 1e6, ..Default::default() }
    }

    #[test]
    fn pid_pushes_ball_back_to_centre() {
        let mut p = BallPid::new(cfg());
        let out = p.step(Vector2::new(30.0, -10.0), Vector2::zeros(), 0.033);
        assert!(out.x < 0.0 && out.y > 0.0);
    }

    #[test]
    fn deadband_holds_previous_output() {
        let mut p = BallPid::new(cfg());
        let a = p.step(Vector2::new(30.0, 0.0), Vector2::zeros(), 0.033);
        let b = p.step(Vector2::new(1.0, 0.0), Vector2::new(2.0, 0.0), 0.033);
        assert_eq!(a, b);
    }

    #[test]
    fn output_is_clamped_and_slew_limited() {
        let mut c = cfg();
        c.max_tilt_deg = 5.0;
        let mut p = BallPid::new(c.clone());
        let out = p.step(Vector2::new(500.0, 0.0), Vector2::zeros(), 0.033);
        assert!((out.norm() - 5f64.to_radians()).abs() < 1e-9);

        c.max_tilt_rate_deg_s = 30.0;
        let mut p = BallPid::new(c);
        let out = p.step(Vector2::new(500.0, 0.0), Vector2::zeros(), 0.1);
        assert!((out.norm() - 3f64.to_radians()).abs() < 1e-9);
    }

    #[test]
    fn integral_does_not_wind_up_when_saturated() {
        let mut c = cfg();
        c.ki = 0.001;
        c.max_tilt_deg = 1.0;
        let mut p = BallPid::new(c);
        for _ in 0..100 {
            p.step(Vector2::new(200.0, 0.0), Vector2::zeros(), 0.033);
        }
        assert!(p.integral.norm() < 1e-9);
    }

    #[test]
    fn acceleration_fit() {
        let s: Vec<_> = (0..12)
            .map(|i| {
                let t = i as f64 / 30.0;
                (t, Vector2::new(3.0 + 10.0 * t + 0.5 * 300.0 * t * t, -0.5 * 120.0 * t * t))
            })
            .collect();
        let a = fit_acceleration(&s).unwrap();
        assert!((a - Vector2::new(300.0, -120.0)).norm() < 1e-6);
    }

    #[test]
    fn axes_map_inverts() {
        let m = AxesMap(Matrix2::new(0.0, -1.0, 1.0, 0.0));
        let uv = m.blade_to_uv(Vector2::new(0.02, 0.01));
        assert!((m.0 * uv - Vector2::new(0.02, 0.01)).norm() < 1e-12);
        assert!((m.axes_angle_deg() - 90.0).abs() < 1e-9);
        let n = AxesMap(Matrix2::new(-0.44, 0.03, 0.19, 1.0)).normalized();
        assert!((n.0.column(0).norm() - 1.0).abs() < 1e-12 && (n.0.column(1).norm() - 1.0).abs() < 1e-12);
    }

    /// Ball on a tilting plane at 30 Hz with 4 frames (133 ms) of loop delay plus a first-order servo lag:
    /// the default gains must bring the ball back from 40 mm and keep it on the blade.
    #[test]
    fn default_gains_balance_a_delayed_ball() {
        use crate::estimator::AlphaBeta;
        let c = ControlConfig::default();
        let mut pid = BallPid::new(c.clone());
        let mut filt = AlphaBeta::new(c.alpha, c.beta);
        let dt = 1.0 / 30.0;
        let (mut p, mut v) = (Vector2::new(40.0, -25.0), Vector2::zeros());
        let mut tilt = Vector2::zeros();
        let mut queue = std::collections::VecDeque::from(vec![Vector2::zeros(); 4]);
        let mut worst: f64 = 0.0;
        for i in 0..300 {
            // Physics substeps.
            for _ in 0..10 {
                let a = tilt * BALL_ACCEL_PER_RAD;
                v += a * (dt / 10.0);
                p += v * (dt / 10.0);
            }
            worst = worst.max(p.norm());
            let cmd_tilt = *queue.back().unwrap();
            let accel = cmd_tilt * BALL_ACCEL_PER_RAD;
            filt.update(p, dt, accel);
            let ahead = c.predict_ms / 1e3;
            let out = pid.step(filt.predict(ahead, accel), filt.vel + accel * ahead, dt);
            queue.push_back(out);
            // Servo: first-order lag (~60 ms) on the delayed command.
            let delayed = queue.pop_front().unwrap();
            tilt += (delayed - tilt) * (dt / 0.06).min(1.0);
            if i == 299 {
                assert!(p.norm() < 10.0, "ball still at {:.1} mm", p.norm());
            }
        }
        assert!(worst < 65.0, "ball reached {worst:.1} mm");
    }

    #[test]
    fn backlash_offsets_in_direction_of_motion() {
        let mut b = Backlash::default();
        assert_eq!(b.apply(100.0, 20.0), 100.0); // no direction yet
        assert_eq!(b.apply(110.0, 20.0), 120.0); // moving up: +10
        assert_eq!(b.apply(110.5, 20.0), 120.5); // tiny change keeps direction
        assert_eq!(b.apply(105.0, 20.0), 95.0); // reversal: -10
        assert_eq!(b.apply(105.0, 0.0), 105.0); // off
    }

    #[test]
    fn min_jerk_endpoints() {
        assert_eq!(min_jerk(0.0), 0.0);
        assert_eq!(min_jerk(1.0), 1.0);
        assert!((min_jerk(0.5) - 0.5).abs() < 1e-12);
    }
}
