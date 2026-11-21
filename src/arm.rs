//! The follower arm: the bus plus the last commanded goal of M1..M7.

use crate::bus::Bus;
use crate::config::{Config, N_MOTORS, WRIST};
use crate::control::min_jerk;
use crate::throw::{ServoSample, ThrowPlan, ThrowReport};
use anyhow::Result;
use st3215::protocol::RamRegister;
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

impl Arm {
    /// Run a throw: set each moving joint's Acc and Goal Speed, send all goals at once, and watch position and load
    /// until the joints arrive. A joint that stays above `abort_load` for 3 reads, or a move that takes far longer
    /// than planned, stops the arm where it is. The servos' previous Acc and Goal Speed are always restored.
    pub async fn throw_move(&mut self, plan: &ThrowPlan, abort_load: i16, timeout_factor: f64, record_after_s: f64) -> Result<ThrowReport> {
        let ids: Vec<u8> = (0..7).filter(|&j| plan.moving[j]).map(|j| j as u8 + 1).collect();
        let mut saved = Vec::with_capacity(ids.len());
        for &id in &ids {
            saved.push((id, self.bus.read_profile(id).await?));
        }
        let res = self.throw_inner(plan, &ids, abort_load, timeout_factor, record_after_s).await;
        // Restore the profiles whatever happened (the arm holds its last goal meanwhile).
        let acc: Vec<(u8, u8)> = saved.iter().map(|&(id, (a, _))| (id, a)).collect();
        let sp: Vec<(u8, u16)> = saved.iter().map(|&(id, (_, s))| (id, s)).collect();
        let r1 = self.bus.sync_write_u8(RamRegister::Acc, &acc).await;
        let r2 = self.bus.sync_write_u16(RamRegister::GoalSpeed, &sp).await;
        let report = res?;
        r1?;
        r2?;
        Ok(report)
    }

    async fn throw_inner(
        &mut self,
        plan: &ThrowPlan,
        ids: &[u8],
        abort_load: i16,
        timeout_factor: f64,
        record_after_s: f64,
    ) -> Result<ThrowReport> {
        let j_of = |id: u8| id as usize - 1;
        let acc: Vec<(u8, u8)> = ids.iter().map(|&id| (id, plan.acc[j_of(id)])).collect();
        let sp: Vec<(u8, u16)> = ids.iter().map(|&id| (id, plan.speed[j_of(id)])).collect();
        self.bus.sync_write_u8(RamRegister::Acc, &acc).await?;
        self.bus.sync_write_u16(RamRegister::GoalSpeed, &sp).await?;
        let goals: Vec<(u8, u16)> = ids.iter().map(|&id| (id, plan.goals[j_of(id)])).collect();
        let t0 = Instant::now();
        self.bus.sync_write_goals(&goals).await?;
        for &id in ids {
            self.cmd[j_of(id)] = plan.goals[j_of(id)];
        }
        let mut rep = ThrowReport::default();
        let mut high = [0u8; 7];
        let limit = plan.duration_s * timeout_factor + 0.5;
        loop {
            let mut done = true;
            let mut sample = ServoSample { t: t0.elapsed().as_secs_f64(), ..Default::default() };
            for &id in ids {
                let j = j_of(id);
                let s = match self.bus.read_state(id).await {
                    Ok(s) => s,
                    Err(_) => {
                        done = false;
                        continue;
                    }
                };
                sample.pos[j] = s.position;
                sample.load[j] = s.load;
                let err = plan.goals[j] as i32 - s.position as i32;
                rep.final_error[j] = err;
                rep.peak_load[j] = rep.peak_load[j].max(s.load.abs());
                high[j] = if s.load.abs() >= abort_load { high[j] + 1 } else { 0 };
                if err.abs() > 20 {
                    done = false;
                    if high[j] >= 3 {
                        rep.aborted = Some(format!("M{id} load {:.0} % while {err} ticks short", s.load.abs() as f64 / 10.0));
                    }
                }
            }
            rep.servo.push(sample);
            rep.move_s = t0.elapsed().as_secs_f64();
            if rep.aborted.is_none() && rep.move_s > limit && !done {
                rep.aborted = Some(format!("not there after {:.1} s (planned {:.2} s)", rep.move_s, plan.duration_s));
            }
            if rep.aborted.is_some() {
                // Stop where it is.
                let present = self.bus.read_positions(ids).await.unwrap_or_default();
                if present.len() == ids.len() {
                    let hold: Vec<(u8, u16)> = ids.iter().zip(&present).map(|(&id, &p)| (id, p)).collect();
                    self.bus.sync_write_goals(&hold).await?;
                    for (&id, &p) in ids.iter().zip(&present) {
                        self.cmd[j_of(id)] = p;
                    }
                }
                return Ok(rep);
            }
            if done {
                break;
            }
        }
        // Keep reading a little longer: the ball leaves while the arm settles.
        let until = t0.elapsed().as_secs_f64() + record_after_s;
        while t0.elapsed().as_secs_f64() < until {
            let mut sample = ServoSample { t: t0.elapsed().as_secs_f64(), ..Default::default() };
            for &id in ids {
                if let Ok(s) = self.bus.read_state(id).await {
                    sample.pos[j_of(id)] = s.position;
                    sample.load[j_of(id)] = s.load;
                }
            }
            rep.servo.push(sample);
        }
        Ok(rep)
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
