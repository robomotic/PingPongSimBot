//! The follower arm: the bus plus the last commanded goal of M1..M7.

use crate::bus::Bus;
use crate::config::{Config, N_MOTORS, WRIST};
use crate::control::min_jerk;
use anyhow::Result;
use std::time::{Duration, Instant};

pub const ARM_IDS: [u8; 7] = [1, 2, 3, 4, 5, 6, 7];

pub struct Arm {
    pub bus: Bus,
    /// Last goal sent to M1..M7.
    pub cmd: [u16; 7],
}

impl Arm {
    /// Opens the bus and makes every joint hold where it is (torque on, no jump; saved grip re-applied).
    /// Returns the arm and the present pose of M1..M8.
    pub async fn open(cfg: &Config) -> Result<(Self, [u16; N_MOTORS])> {
        let mut bus = Bus::open(&cfg.bus.port, cfg.bus.timeout_ms)?;
        let present = bus.hold_present(cfg.poses.grip).await?;
        let mut cmd = [0u16; 7];
        cmd.copy_from_slice(&present[..7]);
        let mut all = [0u16; N_MOTORS];
        all.copy_from_slice(&present);
        Ok((Self { bus, cmd }, all))
    }

    pub async fn send(&mut self, goals: [u16; 7]) -> Result<()> {
        let g: Vec<(u8, u16)> = ARM_IDS.iter().zip(goals).map(|(&id, t)| (id, t)).collect();
        self.bus.sync_write_goals(&g).await?;
        self.cmd = goals;
        Ok(())
    }

    pub async fn send_wrist(&mut self, wrist: [u16; 3]) -> Result<()> {
        let g: Vec<(u8, u16)> = WRIST.iter().zip(wrist).map(|(&j, t)| (j as u8 + 1, t)).collect();
        self.bus.sync_write_goals(&g).await?;
        for (k, &j) in WRIST.iter().enumerate() {
            self.cmd[j] = wrist[k];
        }
        Ok(())
    }

    /// Minimum-jerk move of M1..M7 from the last command to `target`, at 50 Hz.
    pub async fn move_to(&mut self, target: &[u16], secs: f64) -> Result<()> {
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

pub fn wrist_of(p: &[u16]) -> [u16; 3] {
    [p[WRIST[0]], p[WRIST[1]], p[WRIST[2]]]
}

pub fn joint_rad(cfg: &Config, ticks: &[u16]) -> [f64; 7] {
    let mut q = [0.0; 7];
    for j in 0..7 {
        q[j] = cfg.tick_to_rad(j, ticks[j] as f64);
    }
    q
}
