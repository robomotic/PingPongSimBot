//! Orientation-only forward kinematics of the ElRobot follower, and redundancy resolution of a 2-DoF blade tilt over
//! the 3 wrist joints (M5 roll, M6 pitch, M7 roll).
//!
//! In `hardware/elrobot/simulation/elrobot_follower.urdf` every joint has rpy="0 0 0", so each joint axis is
//! expressed in the base frame at the zero pose and the gripper orientation is just R1(q1) R2(q2) ... R7(q7).
//! Translations do not matter for the tilt.
//!
//! The blade frame is defined at the taught level pose: g0 = gravity seen from the gripper there (the blade normal),
//! and u, v are any orthonormal pair perpendicular to g0. The "tilt" of a pose is the in-plane part of gravity,
//! (g.u, g.v), which is what makes the ball accelerate (a = 0.6 g * tilt for a hollow ball). How u, v line up with
//! the camera's blade axes is measured once by `calib-axes`, so the paddle's mounting angle need not be known.

use nalgebra::{Matrix2, Matrix2x3, Matrix3, Matrix3x2, Rotation3, Unit, Vector2, Vector3};

/// Joint axes rev_motor_01..07 from the URDF.
const AXES: [[f64; 3]; 7] = [
    [0.0, -0.008727, 0.999962],
    [-0.999962, -7.6e-05, 0.008726],
    [-0.999848, -0.000152, 0.017452],
    [-0.99981, 0.008574, 0.017527],
    [-0.008573, -0.999963, 0.000151],
    [-0.999697, 0.0173, 0.017525],
    [-0.017144, -0.999812, 0.009029],
];

/// Gripper orientation in the base frame for joint angles q1..q7 (rad).
pub fn gripper_rotation(q: &[f64; 7]) -> Rotation3<f64> {
    let mut r = Rotation3::identity();
    for (axis, &angle) in AXES.iter().zip(q) {
        let a = Unit::new_normalize(Vector3::new(axis[0], axis[1], axis[2]));
        r *= Rotation3::from_axis_angle(&a, angle);
    }
    r
}

/// Unit gravity seen from the gripper frame.
pub fn gravity_in_gripper(q: &[f64; 7]) -> Vector3<f64> {
    gripper_rotation(q).inverse() * Vector3::new(0.0, 0.0, -1.0)
}

#[derive(Debug, Clone)]
pub struct WristModel {
    /// Arm joints M1..M4, held fixed at the level pose.
    arm: [f64; 4],
    u: Vector3<f64>,
    v: Vector3<f64>,
    /// Effort weights for M5, M6, M7 (higher = used less).
    w_inv: Matrix3<f64>,
    damping: f64,
}

impl WristModel {
    /// `level` = joint angles (rad) of M1..M7 at the taught level pose.
    pub fn new(level: &[f64; 7], weights: [f64; 3], damping: f64) -> Self {
        let g0 = gravity_in_gripper(level);
        // Any vector not parallel to g0 seeds the in-plane basis.
        let seed = if g0.x.abs() < 0.9 { Vector3::x() } else { Vector3::y() };
        let u = (seed - g0 * g0.dot(&seed)).normalize();
        let v = g0.cross(&u);
        Self {
            arm: [level[0], level[1], level[2], level[3]],
            u,
            v,
            w_inv: Matrix3::from_diagonal(&Vector3::new(1.0 / weights[0], 1.0 / weights[1], 1.0 / weights[2])),
            damping,
        }
    }

    fn full(&self, w: &[f64; 3]) -> [f64; 7] {
        [self.arm[0], self.arm[1], self.arm[2], self.arm[3], w[0], w[1], w[2]]
    }

    /// In-plane gravity (sin of tilt) along u and v for wrist angles M5..M7.
    pub fn tilt(&self, wrist: &[f64; 3]) -> Vector2<f64> {
        let g = gravity_in_gripper(&self.full(wrist));
        Vector2::new(g.dot(&self.u), g.dot(&self.v))
    }

    pub fn jacobian(&self, wrist: &[f64; 3]) -> Matrix2x3<f64> {
        const H: f64 = 1e-5;
        let mut j = Matrix2x3::zeros();
        for k in 0..3 {
            let (mut a, mut b) = (*wrist, *wrist);
            a[k] += H;
            b[k] -= H;
            j.set_column(k, &((self.tilt(&a) - self.tilt(&b)) / (2.0 * H)));
        }
        j
    }

    /// Weighted, damped pseudo-inverse J# = W^-1 J^T (J W^-1 J^T + l^2 I)^-1.
    fn pinv(&self, j: &Matrix2x3<f64>) -> Matrix3x2<f64> {
        let jt = self.w_inv * j.transpose();
        let m = j * jt + Matrix2::identity() * self.damping * self.damping;
        jt * m.try_inverse().unwrap_or_else(Matrix2::zeros)
    }

    /// Wrist angles that reach `target` tilt, starting from `current`, preferring low-weight joints and pulling the
    /// redundant direction back toward `rest` (the level pose) by `null_gain`.
    pub fn solve(&self, current: &[f64; 3], target: Vector2<f64>, rest: &[f64; 3], null_gain: f64) -> [f64; 3] {
        let mut q = Vector3::from_column_slice(current);
        let rest = Vector3::from_column_slice(rest);
        for _ in 0..3 {
            let qa = [q[0], q[1], q[2]];
            let j = self.jacobian(&qa);
            let jp = self.pinv(&j);
            let err = target - self.tilt(&qa);
            let null = (Matrix3::identity() - jp * j) * (rest - q) * null_gain;
            q += jp * err + null;
            if err.norm() < 1e-5 {
                break;
            }
        }
        [q[0], q[1], q[2]]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEVEL: [f64; 7] = [0.0, 0.3, -0.6, 0.4, 0.2, 0.5, -0.3];

    #[test]
    fn zero_pose_is_identity() {
        let r = gripper_rotation(&[0.0; 7]);
        assert!((r.matrix() - Matrix3::identity()).norm() < 1e-12);
    }

    #[test]
    fn level_pose_has_zero_tilt() {
        let m = WristModel::new(&LEVEL, [3.0, 1.5, 1.0], 0.0);
        assert!(m.tilt(&[LEVEL[4], LEVEL[5], LEVEL[6]]).norm() < 1e-12);
    }

    #[test]
    fn solver_reaches_target_tilt() {
        let m = WristModel::new(&LEVEL, [3.0, 1.5, 1.0], 0.001);
        let w0 = [LEVEL[4], LEVEL[5], LEVEL[6]];
        let target = Vector2::new(0.05, -0.08);
        let q = m.solve(&w0, target, &w0, 0.0);
        assert!((m.tilt(&q) - target).norm() < 1e-4, "{:?}", m.tilt(&q));
    }

    #[test]
    fn null_space_pull_keeps_tilt() {
        let m = WristModel::new(&LEVEL, [3.0, 1.5, 1.0], 0.001);
        let w0 = [LEVEL[4], LEVEL[5], LEVEL[6]];
        let target = Vector2::new(0.03, 0.02);
        let q = m.solve(&w0, target, &w0, 0.0);
        // Push the redundant direction away and let the null-space term pull it back.
        let off = [q[0] + 0.1, q[1], q[2] - 0.1];
        let q2 = m.solve(&off, target, &w0, 0.5);
        assert!((m.tilt(&q2) - target).norm() < 1e-3);
    }

    #[test]
    fn heavy_joint_moves_least() {
        let m = WristModel::new(&LEVEL, [10.0, 1.0, 1.0], 0.001);
        let w0 = [LEVEL[4], LEVEL[5], LEVEL[6]];
        let q = m.solve(&w0, Vector2::new(0.04, 0.04), &w0, 0.0);
        let d: Vec<f64> = (0..3).map(|k| (q[k] - w0[k]).abs()).collect();
        assert!(d[0] < d[2], "{d:?}");
    }
}
