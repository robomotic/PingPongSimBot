//! Background task that owns the arm (real bus or a simulated arm) for the Pose screen: it streams the state of all
//! 8 servos at ~10 Hz and executes moves. The UI never waits on bus I/O.

use crate::arm::Arm;
use crate::bus::{MotorState, ServoGains};
use crate::config::{Config, ThrowConfig};
use crate::throw::ThrowPlan;
use crate::control::min_jerk;
use std::time::{Duration, Instant};
use tokio::sync::mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender};

#[derive(Debug, Clone)]
pub enum DevCmd {
    /// Smooth move of M1..M7 to these goals.
    MoveArm { goals: [u16; 7], secs: f64 },
    /// Set the gripper (M8) goal.
    Grip(u16),
    /// Read P/D/I and torque limits of all 8 servos.
    ReadGains,
    /// Write P gains (EEPROM, persistent) to these servos: (id, P).
    WriteP(Vec<(u8, u8)>),
    /// Throw move, hold, then a smooth return to `back`.
    Throw { plan: ThrowPlan, cfg: ThrowConfig, back: [u16; 7], return_s: f64 },
    Close,
}

#[derive(Debug, Clone)]
pub enum DevMsg {
    Opened { goals: [u16; 8] },
    State { states: [MotorState; 8], torque8: bool },
    Moving(bool),
    /// Result line of a throw move.
    Throw(String),
    /// Gains of M1..M8 (None = that servo didn't answer).
    Gains([Option<ServoGains>; 8]),
    /// Result line of a gain write.
    Info(String),
    Error(String),
    Closed,
}

pub struct Device {
    pub tx: UnboundedSender<DevCmd>,
    pub rx: UnboundedReceiver<DevMsg>,
}

impl Drop for Device {
    fn drop(&mut self) {
        let _ = self.tx.send(DevCmd::Close);
    }
}

/// Simulated arm: positions follow goals with a first-order lag; the gripper stalls on the paddle handle.
struct SimArm {
    goal: [f64; 8],
    pos: [f64; 8],
    /// M8 position where the jaws touch the handle (closing = decreasing ticks, like the real arm).
    handle_at: f64,
    gains: [ServoGains; 8],
}

impl SimArm {
    fn step(&mut self, dt: f64) -> [MotorState; 8] {
        let mut out = [MotorState::default(); 8];
        for j in 0..8 {
            let target = if j == 7 { self.goal[7].max(self.handle_at) } else { self.goal[j] };
            self.pos[j] += (target - self.pos[j]) * (dt / 0.12).min(1.0);
            let squeeze = if j == 7 { (self.handle_at - self.goal[7]).max(0.0) } else { 0.0 };
            out[j] = MotorState {
                position: self.pos[j].round() as u16,
                load: (squeeze * 4.0 + (self.goal[j] - self.pos[j]).abs() * 2.0).min(1000.0) as i16,
                current_ma: 3.0 + squeeze * 3.0,
                voltage_dv: 121,
                temperature_c: 29,
                ..Default::default()
            };
        }
        out
    }
}

pub fn open(cfg: &Config, sim: bool) -> Device {
    let (tx, mut crx) = unbounded_channel::<DevCmd>();
    let (mtx, rx) = unbounded_channel::<DevMsg>();
    let cfg = cfg.clone();
    tokio::spawn(async move {
        let send = |m: DevMsg| {
            let _ = mtx.send(m);
        };
        if sim {
            let start = cfg.poses.rest.or(cfg.poses.level).unwrap_or([2048; 8]);
            let mut arm = SimArm {
                goal: start.map(|t| t as f64),
                pos: start.map(|t| t as f64),
                handle_at: cfg.poses.grip.map_or(start[7] as f64 - 40.0, |g| g as f64 + 40.0),
                gains: [ServoGains { p: 16, d: 0, i: 0, max_torque: 500, torque_limit: 500 }; 8],
            };
            send(DevMsg::Opened { goals: start });
            let mut tick = tokio::time::interval(Duration::from_millis(100));
            let mut moving: Option<([f64; 7], [f64; 7], Instant, f64)> = None;
            loop {
                tokio::select! {
                    c = crx.recv() => match c {
                        None | Some(DevCmd::Close) => break,
                        Some(DevCmd::MoveArm { goals, secs }) => {
                            let mut from = [0.0; 7];
                            from.copy_from_slice(&arm.goal[..7]);
                            moving = Some((from, goals.map(|t| t as f64), Instant::now(), secs));
                            send(DevMsg::Moving(true));
                        }
                        Some(DevCmd::Grip(g)) => arm.goal[7] = g as f64,
                        Some(DevCmd::ReadGains) => send(DevMsg::Gains(arm.gains.map(Some))),
                        Some(DevCmd::WriteP(v)) => {
                            for &(id, p) in &v {
                                arm.gains[id as usize - 1].p = p;
                            }
                            send(DevMsg::Info(format!("wrote P to {} servo(s) (simulated)", v.len())));
                            send(DevMsg::Gains(arm.gains.map(Some)));
                        }
                        Some(DevCmd::Throw { plan, cfg, back, return_s }) => {
                            send(DevMsg::Moving(true));
                            let mut from = [0.0; 7];
                            from.copy_from_slice(&arm.goal[..7]);
                            let to = plan.goals.map(|t| t as f64);
                            // Fast flip (linear in the planned time), hold, then the smooth return below.
                            let t0 = Instant::now();
                            let mut tick = tokio::time::interval(Duration::from_millis(20));
                            loop {
                                tick.tick().await;
                                let s = (t0.elapsed().as_secs_f64() / plan.duration_s.max(0.05)).min(1.0);
                                for j in 0..7 {
                                    arm.goal[j] = from[j] + (to[j] - from[j]) * s;
                                }
                                let states = arm.step(0.02);
                                send(DevMsg::State { states, torque8: true });
                                if s >= 1.0 {
                                    break;
                                }
                            }
                            send(DevMsg::Throw(format!("throw done in {:.2} s (simulated)", plan.duration_s)));
                            tokio::time::sleep(Duration::from_secs_f64(cfg.hold_s)).await;
                            let mut f2 = [0.0; 7];
                            f2.copy_from_slice(&arm.goal[..7]);
                            moving = Some((f2, back.map(|t| t as f64), Instant::now(), return_s));
                        }
                    },
                    _ = tick.tick() => {
                        if let Some((from, to, t0, secs)) = moving {
                            let s = min_jerk(t0.elapsed().as_secs_f64() / secs.max(0.05));
                            for j in 0..7 {
                                arm.goal[j] = from[j] + (to[j] - from[j]) * s;
                            }
                            if s >= 1.0 {
                                moving = None;
                                send(DevMsg::Moving(false));
                            }
                        }
                        let states = arm.step(0.1);
                        send(DevMsg::State { states, torque8: true });
                    }
                }
            }
            send(DevMsg::Closed);
            return;
        }

        let (mut arm, present) = match Arm::open(&cfg).await {
            Ok(a) => a,
            Err(e) => {
                send(DevMsg::Error(format!("{e:#}")));
                send(DevMsg::Closed);
                return;
            }
        };
        let mut grip_goal = cfg.poses.grip.filter(|g| (*g as i32 - present[7] as i32).abs() <= crate::bus::GRIP_SNAP_TICKS).unwrap_or(present[7]);
        let mut goals = [0u16; 8];
        goals[..7].copy_from_slice(&arm.cmd);
        goals[7] = grip_goal;
        send(DevMsg::Opened { goals });
        let mut tick = tokio::time::interval(Duration::from_millis(100));
        let mut errors = 0;
        loop {
            tokio::select! {
                c = crx.recv() => match c {
                    None | Some(DevCmd::Close) => break,
                    Some(DevCmd::MoveArm { goals, secs }) => {
                        send(DevMsg::Moving(true));
                        if let Err(e) = arm.move_to(&goals, secs).await {
                            send(DevMsg::Error(format!("move failed: {e:#}")));
                        }
                        send(DevMsg::Moving(false));
                    }
                    Some(DevCmd::ReadGains) => {
                        let mut g = [None; 8];
                        for id in 1..=8u8 {
                            g[id as usize - 1] = arm.bus.read_gains(id).await.ok();
                        }
                        send(DevMsg::Gains(g));
                    }
                    Some(DevCmd::WriteP(v)) => {
                        let mut ok = 0;
                        let mut bad = Vec::new();
                        for &(id, p) in &v {
                            match arm.bus.write_eeprom_u8(id, crate::bus::EEPROM_P, p).await {
                                Ok(back) if back == p => ok += 1,
                                Ok(back) => bad.push(format!("M{id} reads back {back}")),
                                Err(e) => bad.push(format!("M{id}: {e:#}")),
                            }
                        }
                        if bad.is_empty() {
                            send(DevMsg::Info(format!("wrote P to {ok} servo(s), verified")));
                        } else {
                            send(DevMsg::Error(format!("P write: {ok} ok; {}", bad.join("; "))));
                        }
                        let mut g = [None; 8];
                        for id in 1..=8u8 {
                            g[id as usize - 1] = arm.bus.read_gains(id).await.ok();
                        }
                        send(DevMsg::Gains(g));
                    }
                    Some(DevCmd::Throw { plan, cfg, back, return_s }) => {
                        send(DevMsg::Moving(true));
                        match arm.throw_move(&plan, cfg.abort_load, cfg.timeout_factor, 0.0, cfg.torque_limit).await {
                            Ok(rep) => {
                                send(DevMsg::Throw(rep.summary()));
                                tokio::time::sleep(Duration::from_secs_f64(cfg.hold_s)).await;
                                if rep.aborted.is_none() {
                                    if let Err(e) = arm.move_to(&back, return_s).await {
                                        send(DevMsg::Error(format!("return to level failed: {e:#}")));
                                    }
                                } else {
                                    let mut g = [0u16; 8];
                                    g[..7].copy_from_slice(&arm.cmd);
                                    g[7] = grip_goal;
                                    send(DevMsg::Opened { goals: g });
                                    send(DevMsg::Error("throw aborted: the arm holds where it is; check it, then go to level (l)".into()));
                                }
                            }
                            Err(e) => {
                                let mut g = [0u16; 8];
                                g[..7].copy_from_slice(&arm.cmd);
                                g[7] = grip_goal;
                                send(DevMsg::Opened { goals: g });
                                send(DevMsg::Error(format!("throw failed: {e:#}")));
                            }
                        }
                        send(DevMsg::Moving(false));
                    }
                    Some(DevCmd::Grip(g)) => {
                        grip_goal = g;
                        if let Err(e) = arm.bus.sync_write_goals(&[(8, g)]).await {
                            send(DevMsg::Error(format!("gripper: {e:#}")));
                        }
                    }
                },
                _ = tick.tick() => {
                    let mut states = [MotorState::default(); 8];
                    let mut ok = true;
                    for id in 1..=8u8 {
                        match arm.bus.read_state(id).await {
                            Ok(s) => states[id as usize - 1] = s,
                            Err(_) => ok = false,
                        }
                    }
                    errors = if ok { 0 } else { errors + 1 };
                    if errors == 5 {
                        send(DevMsg::Error("servo reads keep failing".into()));
                    }
                    let torque8 = arm.bus.read_torque_enabled(8).await.unwrap_or(false);
                    send(DevMsg::State { states, torque8 });
                }
            }
        }
        let _ = grip_goal;
        send(DevMsg::Closed);
    });
    Device { tx, rx }
}
