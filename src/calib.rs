//! Axis calibration on any [`Plant`]: tilt the blade 3° along each solver axis (u, v) and measure which way, and how
//! fast, the ball rolls. Same procedure as the `calib-axes` command, but event-driven for the full-screen app.

use crate::balance::{Plant, WristCtl};
use crate::config::Config;
use crate::control::{fit_acceleration, AxesMap, BALL_ACCEL_PER_RAD};
use crate::estimator::Homography;
use anyhow::{bail, Result};
use nalgebra::{Matrix2, Vector2};
use std::time::{Duration, Instant};
use tokio::sync::mpsc::UnboundedReceiver;

#[derive(Debug, Clone)]
pub enum AxesCmd {
    /// The ball is on the blade centre: measure the next axis.
    Go,
    Stop,
}

#[derive(Debug, Clone)]
pub enum AxesEvent {
    Status(String),
    /// Waiting for the ball at the centre before measuring axis k (0 = u, 1 = v).
    WaitBall(usize),
    Axis { k: usize, accel: Vector2<f64>, ratio: f64, dir_deg: f64 },
    Retry(usize, String),
    Done { map: AxesMap },
    Failed(String),
}

async fn record<P: Plant>(plant: &mut P, h: &Homography, secs: f64) -> Result<Vec<(f64, Vector2<f64>)>> {
    let t0 = Instant::now();
    let mut out = Vec::new();
    while t0.elapsed().as_secs_f64() < secs {
        if let Some(d) = plant.next_frame(Duration::from_millis(200)).await? {
            if let Some(b) = d.blob {
                out.push((d.t.saturating_duration_since(t0).as_secs_f64(), h.apply(b.px)));
            }
        }
    }
    Ok(out)
}

/// Moves to level, then for each axis waits for [`AxesCmd::Go`], tilts by `tilt_deg`, measures, returns to level.
/// The plant ends at level.
pub async fn run<P: Plant>(
    plant: &mut P,
    cfg: &Config,
    tilt_deg: f64,
    cmds: &mut UnboundedReceiver<AxesCmd>,
    emit: &mut impl FnMut(AxesEvent),
) -> Result<AxesMap> {
    let level = cfg.level()?;
    let h = cfg.camera.homography.map(|h| Homography::from_row_major(&h)).ok_or_else(|| anyhow::anyhow!("no camera map: run the camera-map calibration first"))?;
    let mut wrist = WristCtl::new(cfg, &level);
    emit(AxesEvent::Status("moving to level...".into()));
    plant.move_to(&level, cfg.poses.move_time_s).await?;
    let tilt = tilt_deg.to_radians();
    let mut cols = [Vector2::zeros(); 2];
    let mut k = 0;
    while k < 2 {
        emit(AxesEvent::WaitBall(k));
        // Wait for Go, keep the camera drained meanwhile.
        loop {
            tokio::select! {
                c = cmds.recv() => match c {
                    Some(AxesCmd::Go) => break,
                    None | Some(AxesCmd::Stop) => bail!("cancelled"),
                },
                f = plant.next_frame(Duration::from_millis(200)) => { f?; }
            }
        }
        plant.place_ball(); // simulation only
        emit(AxesEvent::Status(format!("axis {}: measuring...", ["u", "v"][k])));
        let base = record(plant, &h, 0.3).await?;
        let a0 = fit_acceleration(&base).unwrap_or_else(Vector2::zeros);
        let mut uv = Vector2::zeros();
        uv[k] = tilt;
        let goal = wrist.ticks_for(cfg, uv);
        plant.send_wrist(goal).await?;
        let samples = record(plant, &h, 0.45).await?;
        plant.move_to(&level, 0.5).await?;
        wrist.reset();
        let late: Vec<_> = samples.into_iter().filter(|(t, _)| *t >= 0.15).collect();
        match fit_acceleration(&late) {
            Some(a) if late.len() >= 6 => {
                let a = a - a0;
                cols[k] = a / (BALL_ACCEL_PER_RAD * tilt);
                emit(AxesEvent::Axis { k, accel: a, ratio: cols[k].norm(), dir_deg: a.y.atan2(a.x).to_degrees() });
                k += 1;
            }
            _ => emit(AxesEvent::Retry(k, format!("ball tracked in only {} frames; put it back on the centre", late.len()))),
        }
    }
    let map = AxesMap(Matrix2::from_columns(&cols));
    emit(AxesEvent::Done { map });
    Ok(map)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{SimParams, SimPlant};

    /// On the simulator, the measured axes point where the simulator's true axes point.
    #[test]
    fn axes_calibration_recovers_the_simulated_axes() {
        let cfg = Config::example();
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        let map = rt.block_on(async {
            let mut sim = SimPlant::new(&cfg, SimParams { noise_px: 0.1, ..Default::default() }, 3).unwrap();
            let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
            tx.send(AxesCmd::Go).unwrap();
            tx.send(AxesCmd::Go).unwrap();
            run(&mut sim, &cfg, 3.0, &mut rx, &mut |_| {}).await.unwrap()
        });
        let truth = AxesMap::from_row_major(&cfg.camera.axes.unwrap()).normalized();
        for k in 0..2 {
            let (a, b) = (map.0.column(k).normalize(), truth.0.column(k).into_owned());
            let ang = a.dot(&b).clamp(-1.0, 1.0).acos().to_degrees();
            assert!(ang < 15.0, "axis {k}: {ang:.1} deg off ({a:?} vs {b:?})");
        }
    }
}
