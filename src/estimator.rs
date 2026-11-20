//! Image -> blade coordinates (homography) and ball position/velocity filtering.

use anyhow::{bail, Result};
use nalgebra::{DMatrix, Matrix3, Vector2, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct Homography(pub Matrix3<f64>);

impl Homography {
    pub fn from_row_major(h: &[f64; 9]) -> Self {
        Self(Matrix3::from_row_slice(h))
    }

    pub fn to_row_major(&self) -> [f64; 9] {
        let m = self.0;
        [m[(0, 0)], m[(0, 1)], m[(0, 2)], m[(1, 0)], m[(1, 1)], m[(1, 2)], m[(2, 0)], m[(2, 1)], m[(2, 2)]]
    }

    pub fn apply(&self, p: Vector2<f64>) -> Vector2<f64> {
        let r = self.0 * Vector3::new(p.x, p.y, 1.0);
        Vector2::new(r.x / r.z, r.y / r.z)
    }

    /// Direct linear transform from >= 4 correspondences (pixel -> mm), solved by SVD.
    pub fn fit(src: &[Vector2<f64>], dst: &[Vector2<f64>]) -> Result<Self> {
        if src.len() < 4 || src.len() != dst.len() {
            bail!("need at least 4 point pairs");
        }
        // Normalise pixel coordinates for conditioning.
        let n = src.len() as f64;
        let c = src.iter().fold(Vector2::zeros(), |a, p| a + p) / n;
        let s = (2.0f64).sqrt() / (src.iter().map(|p| (p - c).norm()).sum::<f64>() / n).max(1e-9);
        let t = Matrix3::new(s, 0.0, -s * c.x, 0.0, s, -s * c.y, 0.0, 0.0, 1.0);

        // Pad to at least 9 rows: nalgebra's thin SVD of an 8x9 matrix would drop the null vector.
        let rows = (2 * src.len()).max(9);
        let mut a = DMatrix::<f64>::zeros(rows, 9);
        for (i, (p, q)) in src.iter().zip(dst).enumerate() {
            let x = s * (p.x - c.x);
            let y = s * (p.y - c.y);
            let (u, v) = (q.x, q.y);
            let r0 = [-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u];
            let r1 = [0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v];
            for k in 0..9 {
                a[(2 * i, k)] = r0[k];
                a[(2 * i + 1, k)] = r1[k];
            }
        }
        let svd = a.svd(false, true);
        let vt = svd.v_t.ok_or_else(|| anyhow::anyhow!("SVD failed"))?;
        let (imin, _) = svd
            .singular_values
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        let h = vt.row(imin);
        let hn = Matrix3::new(h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);
        let full = hn * t;
        Ok(Self(full / full[(2, 2)]))
    }
}

/// Alpha-beta filter with variable time step.
#[derive(Debug, Clone)]
pub struct AlphaBeta {
    alpha: f64,
    beta: f64,
    pub pos: Vector2<f64>,
    pub vel: Vector2<f64>,
    initialised: bool,
}

impl AlphaBeta {
    pub fn new(alpha: f64, beta: f64) -> Self {
        Self { alpha, beta, pos: Vector2::zeros(), vel: Vector2::zeros(), initialised: false }
    }

    pub fn reset(&mut self) {
        self.initialised = false;
        self.vel = Vector2::zeros();
    }

    /// `accel` is the model acceleration (from the commanded tilt), used in the prediction step.
    pub fn update(&mut self, z: Vector2<f64>, dt: f64, accel: Vector2<f64>) {
        if !self.initialised {
            self.pos = z;
            self.vel = Vector2::zeros();
            self.initialised = true;
            return;
        }
        let dt = dt.clamp(1e-3, 0.2);
        let pred = self.pos + self.vel * dt + accel * (0.5 * dt * dt);
        let pred_v = self.vel + accel * dt;
        let r = z - pred;
        self.pos = pred + r * self.alpha;
        self.vel = pred_v + r * (self.beta / dt);
    }

    /// Position `ahead` seconds in the future.
    pub fn predict(&self, ahead: f64, accel: Vector2<f64>) -> Vector2<f64> {
        self.pos + self.vel * ahead + accel * (0.5 * ahead * ahead)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn homography_round_trips_calibration_points() {
        // A synthetic perspective map.
        let truth = Homography(Matrix3::new(0.4, 0.05, -130.0, -0.02, 0.9, -150.0, 0.0001, 0.002, 1.0));
        let px: Vec<Vector2<f64>> = [(320.0, 240.0), (420.0, 245.0), (220.0, 238.0), (318.0, 150.0), (322.0, 330.0)]
            .iter()
            .map(|&(x, y)| Vector2::new(x, y))
            .collect();
        let mm: Vec<Vector2<f64>> = px.iter().map(|p| truth.apply(*p)).collect();
        let h = Homography::fit(&px, &mm).unwrap();
        for (p, m) in px.iter().zip(&mm) {
            assert!((h.apply(*p) - m).norm() < 1e-6);
        }
        let probe = Vector2::new(300.0, 200.0);
        assert!((h.apply(probe) - truth.apply(probe)).norm() < 1e-3);
    }

    #[test]
    fn alpha_beta_tracks_a_ramp() {
        let mut f = AlphaBeta::new(0.6, 0.2);
        for i in 0..60 {
            let t = i as f64 / 30.0;
            f.update(Vector2::new(50.0 * t, -20.0 * t), 1.0 / 30.0, Vector2::zeros());
        }
        assert!((f.vel - Vector2::new(50.0, -20.0)).norm() < 1.0, "{:?}", f.vel);
    }
}
