//! Pose screen: every joint live, move one at a time with the arrows, go to / save level and rest, set the grip.
//! Replaces `jog` and `grip`.

use super::device::{self, DevCmd, DevMsg, Device};
use super::Ctx;
use crate::bus::MotorState;
use crate::config::Config;
use crossterm::event::{KeyCode, KeyEvent};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Cell, Paragraph, Row, Table};
use ratatui::Frame;

pub const HELP: [&str; 19] = [
    "↑ ↓        choose a joint (M1 base … M7 wrist roll, M8 gripper)",
    "← →        move it by one step (real arm: needs ARMED)",
    "+ / -      bigger / smaller step: 1 · 5 · 20 ticks (gripper: 1 · 5 · 20 · 50)",
    "l / r      go to the level / rest pose (smooth 3 s move, asks first)",
    "L / R      save the current pose as level / rest",
    "G          save the gripper goal as the grip (re-applied by every run)",
    "t          THROW PREVIEW without the ball, from level: 1 = 10 % · 2 = 25 % · 3 = 50 % · 4 = 100 % speed;",
    "           the arm flips to the throw end pose, holds, and returns to level (asks first)",
    "T          save the current pose as the throw end pose (blade upright, face toward the camera side)",
    "Esc        back to Home (releases the bus)",
    "",
    "4096 ticks = one turn (1 tick ≈ 0.09°). Joints stay within ±limit of the level pose",
    "(safety.pose_limit_ticks in paddle.toml) so manual moves stay above the table.",
    "Gripper: ← closes, → opens (+/- steps 1 · 5 · 20 · 50; open far enough and the paddle drops)",
    "Once the jaws touch the handle, extra steps show up as",
    "goal − present and load, not motion. Keep load under ~300; above 600 it backs off by itself.",
    "",
    "Watch the arm while moving it. Ctrl-C stops everything.",
    "",
];

const NAMES: [&str; 8] = ["base", "shoulder", "elbow", "forearm", "wrist roll", "wrist pitch", "wrist roll", "gripper"];
const STEPS: [u16; 3] = [1, 5, 20];
const GRIP_STEPS: [u16; 4] = [1, 5, 20, 50];

pub struct PoseScreen {
    dev: Option<Device>,
    goals: Option<[u16; 8]>,
    states: Option<[MotorState; 8]>,
    torque8: bool,
    moving: bool,
    sel: usize,
    step_i: usize,
    grip_step_i: usize,
    pending: Option<char>,
    msg: Option<(bool, String)>,
}

impl Default for PoseScreen {
    fn default() -> Self {
        Self { dev: None, goals: None, states: None, torque8: false, moving: false, sel: 5, step_i: 1, grip_step_i: 1, pending: None, msg: None }
    }
}

impl PoseScreen {
    pub fn enter(&mut self, ctx: &Ctx) {
        if self.dev.is_none() {
            self.goals = None;
            self.states = None;
            self.msg = Some((true, if ctx.sim { "simulated arm".into() } else { "opening the bus: every joint holds where it is".into() }));
            self.dev = Some(device::open(&ctx.cfg, ctx.sim));
        }
    }

    pub fn leave(&mut self) {
        self.dev = None;
    }

    pub fn pump(&mut self, ctx: &Ctx) {
        let Some(dev) = &mut self.dev else { return };
        let mut closed = false;
        while let Ok(m) = dev.rx.try_recv() {
            match m {
                DevMsg::Opened { goals } => {
                    self.goals = Some(goals);
                    self.msg = Some((true, "ready".into()));
                }
                DevMsg::State { states, torque8 } => {
                    self.states = Some(states);
                    self.torque8 = torque8;
                }
                DevMsg::Moving(m) => {
                    if self.moving && !m && self.msg.as_ref().is_some_and(|(_, t)| t.starts_with("moving to")) {
                        self.msg = Some((true, "done".into()));
                    }
                    self.moving = m;
                }
                DevMsg::Throw(m) => self.msg = Some((!m.contains("ABORTED"), m)),
                DevMsg::Error(e) => self.msg = Some((false, e)),
                DevMsg::Closed => closed = true,
            }
        }
        if closed {
            self.dev = None;
            return;
        }
        // Gripper self-protection.
        if let (Some(st), Some(g)) = (self.states, self.goals.as_mut()) {
            let load = (st[7].load as i32).abs();
            if load >= ctx.cfg.safety.grip_backoff_load as i32 {
                let back: i32 = if (g[7] as i32) < st[7].position as i32 { 10 } else { -10 };
                g[7] = (g[7] as i32 + back).clamp(0, 4095) as u16;
                let _ = self.dev.as_ref().map(|d| d.tx.send(DevCmd::Grip(g[7])));
                self.msg = Some((false, format!("gripper load {load}: backed off 10 ticks to {}", g[7])));
            }
        }
    }

    fn reference(&self, ctx: &Ctx) -> [u16; 8] {
        ctx.cfg.poses.level.or(self.goals).unwrap_or([2048; 8])
    }

    fn save_pose(&mut self, ctx: &mut Ctx, which: char) {
        let (Some(g), Some(st)) = (self.goals, self.states) else { return };
        let mut pose = g;
        if which != 'G' {
            pose[7] = st[7].position;
        }
        let res = Config::load(&ctx.cfg_path).and_then(|mut c| {
            match which {
                'L' => c.poses.level = Some(pose),
                'R' => c.poses.rest = Some(pose),
                _ => c.poses.grip = Some(g[7]),
            }
            std::fs::write(&ctx.cfg_path, toml::to_string_pretty(&c)?)?;
            Ok(c)
        });
        match res {
            Ok(c) => {
                ctx.cfg.poses = c.poses;
                self.msg = Some((true, match which {
                    'L' => format!("saved level {:?}", pose),
                    'R' => format!("saved rest {:?}", pose),
                    _ => format!("saved grip {}", g[7]),
                }));
            }
            Err(e) => self.msg = Some((false, format!("save failed: {e:#}"))),
        }
    }

    /// The preview starts from level: every joint within 30 ticks of it.
    fn at_level(&self, ctx: &Ctx) -> Result<(), String> {
        let (Some(level), Some(g)) = (ctx.cfg.poses.level, self.goals) else { return Err("no level pose saved".into()) };
        match (0..7).find(|&j| (g[j] as i32 - level[j] as i32).abs() > 30) {
            Some(j) => Err(format!("go to level first (l): M{} is {} ticks away", j + 1, g[j] as i32 - level[j] as i32)),
            None => Ok(()),
        }
    }

    fn throw_preview(&mut self, ctx: &Ctx, speed: f64) {
        let (Some(g), Some(dev)) = (self.goals.as_mut(), &self.dev) else { return };
        let Some(level) = ctx.cfg.poses.level else { return };
        let plan = match crate::throw::plan(&ctx.cfg, &g[..7], speed) {
            Ok(p) => p,
            Err(e) => {
                self.msg = Some((false, format!("throw refused: {e:#}")));
                return;
            }
        };
        let mut back = [0u16; 7];
        back.copy_from_slice(&level[..7]);
        g[..7].copy_from_slice(&back);
        let t = &ctx.cfg.throw;
        self.msg = Some((true, format!("throw preview at {:.0} % ({:.2} s move)...", speed * 100.0, plan.duration_s)));
        let _ = dev.tx.send(DevCmd::Throw { plan, cfg: t.clone(), back, return_s: t.return_s });
    }

    fn save_throw_end(&mut self, ctx: &mut Ctx) {
        let Some(g) = self.goals else { return };
        let Some(level) = ctx.cfg.poses.level else {
            self.msg = Some((false, "save the level pose first".into()));
            return;
        };
        let lim = ctx.cfg.throw.limit_ticks;
        if let Some(j) = (0..7).find(|&j| (g[j] as i32 - level[j] as i32).abs() > lim[j] as i32) {
            self.msg = Some((false, format!(
                "M{} is {} ticks from level; the throw allows ±{} (throw.limit_ticks)", j + 1, g[j] as i32 - level[j] as i32, lim[j]
            )));
            return;
        }
        let mut end = [0u16; 7];
        end.copy_from_slice(&g[..7]);
        let res = Config::load(&ctx.cfg_path).and_then(|mut c| {
            c.throw.end = Some(end);
            std::fs::write(&ctx.cfg_path, toml::to_string_pretty(&c)?)?;
            Ok(c)
        });
        match res {
            Ok(c) => {
                ctx.cfg.throw = c.throw;
                self.msg = Some((true, format!("saved throw end {:?}", end)));
            }
            Err(e) => self.msg = Some((false, format!("save failed: {e:#}"))),
        }
    }

    fn go(&mut self, ctx: &Ctx, which: char) {
        let pose = if which == 'l' { ctx.cfg.poses.level } else { ctx.cfg.poses.rest };
        let (Some(pose), Some(g), Some(dev)) = (pose, self.goals.as_mut(), &self.dev) else {
            self.msg = Some((false, format!("no {} pose saved", if which == 'l' { "level" } else { "rest" })));
            return;
        };
        let mut arm = [0u16; 7];
        arm.copy_from_slice(&pose[..7]);
        g[..7].copy_from_slice(&arm);
        let _ = dev.tx.send(DevCmd::MoveArm { goals: arm, secs: ctx.cfg.poses.move_time_s });
        self.msg = Some((true, format!("moving to {}", if which == 'l' { "level" } else { "rest" })));
    }

    /// Returns true to go back Home.
    pub fn key(&mut self, k: KeyEvent, ctx: &mut Ctx) -> bool {
        if let Some(p) = self.pending.take() {
            if p == 't' {
                let speed = match k.code {
                    KeyCode::Char('1') => Some(0.10),
                    KeyCode::Char('2') => Some(0.25),
                    KeyCode::Char('3') => Some(0.50),
                    KeyCode::Char('4') => Some(1.0),
                    _ => None,
                };
                match speed {
                    Some(sp) if ctx.armed() => self.throw_preview(ctx, sp),
                    Some(_) => self.msg = Some((false, "press a to ARM first".into())),
                    None => self.msg = Some((true, "throw preview cancelled".into())),
                }
                return false;
            }
            if k.code == KeyCode::Char('y') {
                if ctx.armed() {
                    self.go(ctx, p);
                } else {
                    self.msg = Some((false, "press a to ARM first".into()));
                }
            }
            return false;
        }
        match k.code {
            KeyCode::Up => self.sel = self.sel.saturating_sub(1),
            KeyCode::Down => self.sel = (self.sel + 1).min(7),
            KeyCode::Char('+') | KeyCode::Char('=') => {
                if self.sel == 7 {
                    self.grip_step_i = (self.grip_step_i + 1).min(GRIP_STEPS.len() - 1)
                } else {
                    self.step_i = (self.step_i + 1).min(STEPS.len() - 1)
                }
            }
            KeyCode::Char('-') | KeyCode::Char('_') => {
                if self.sel == 7 {
                    self.grip_step_i = self.grip_step_i.saturating_sub(1)
                } else {
                    self.step_i = self.step_i.saturating_sub(1)
                }
            }
            KeyCode::Left | KeyCode::Right => self.nudge(ctx, if k.code == KeyCode::Right { 1 } else { -1 }),
            KeyCode::Char(c @ ('l' | 'r')) => {
                if !ctx.armed() {
                    self.msg = Some((false, "press a to ARM first".into()));
                } else {
                    self.pending = Some(c);
                }
            }
            KeyCode::Char(c @ ('L' | 'R' | 'G')) => self.save_pose(ctx, c),
            KeyCode::Char('T') => self.save_throw_end(ctx),

            KeyCode::Char('t') => {
                if !ctx.armed() {
                    self.msg = Some((false, "press a to ARM first".into()));
                } else if self.moving {
                    self.msg = Some((false, "wait for the move to finish".into()));
                } else if let Err(e) = self.at_level(ctx) {
                    self.msg = Some((false, e));
                } else {
                    self.pending = Some('t');
                }
            }
            KeyCode::Esc | KeyCode::Char('q') => {
                self.leave();
                return true;
            }
            _ => {}
        }
        false
    }

    fn nudge(&mut self, ctx: &Ctx, dir: i32) {
        if !ctx.armed() {
            self.msg = Some((false, "press a to ARM first: this moves the arm".into()));
            return;
        }
        if self.moving {
            self.msg = Some((false, "wait for the move to finish".into()));
            return;
        }
        let reference = self.reference(ctx);
        let (Some(g), Some(dev)) = (self.goals.as_mut(), &self.dev) else { return };
        let j = self.sel;
        if j == 7 {
            // Opening a squeezed grip: the goal is past where the jaws stopped on the handle, so step from where the
            // jaws are, or the first presses would only release the squeeze and nothing would visibly move.
            let present = self.states.map_or(g[7], |s| s[7].position);
            let base = if dir > 0 { g[7].max(present) } else { g[7] };
            let next = (base as i32 + dir * GRIP_STEPS[self.grip_step_i] as i32).clamp(0, 4095) as u16;
            g[7] = next;
            let _ = dev.tx.send(DevCmd::Grip(next));
            return;
        }
        let step = STEPS[self.step_i] as i32;
        let next = g[j] as i32 + dir * step;
        let limit = ctx.cfg.safety.pose_limit_ticks[j] as i32;
        if (next - reference[j] as i32).abs() > limit {
            self.msg = Some((false, format!("M{} limit: ±{limit} ticks from level (safety.pose_limit_ticks)", j + 1)));
            return;
        }
        g[j] = next.clamp(0, 4095) as u16;
        let mut arm = [0u16; 7];
        arm.copy_from_slice(&g[..7]);
        let _ = dev.tx.send(DevCmd::MoveArm { goals: arm, secs: 0.15 + step as f64 / 200.0 });
    }

    pub fn keys(&self, ctx: &Ctx) -> Vec<(&'static str, &'static str)> {
        if self.pending == Some('t') {
            return vec![("1", "10 %"), ("2", "25 %"), ("3", "50 %"), ("4", "100 %"), ("any key", "cancel")];
        }
        if self.pending.is_some() {
            return vec![("y", "go"), ("any key", "cancel")];
        }
        let mut v = vec![("↑↓", "joint"), ("←→", "move"), ("+/-", "step"), ("l/r", "go level/rest"), ("L/R", "save level/rest"), ("G", "save grip"), ("t", "throw preview"), ("T", "save throw end")];
        if !ctx.sim {
            v.insert(0, ("a", "arm"));
        }
        v.extend([("Esc", "home"), ("?", "help")]);
        v
    }

    pub fn draw(&self, f: &mut Frame, area: Rect, ctx: &Ctx) {
        let [tbl, info] = Layout::vertical([Constraint::Length(13), Constraint::Min(4)]).areas(area);
        let reference = self.reference(ctx);
        let rows: Vec<Row> = (0..8)
            .map(|j| {
                let sel = j == self.sel;
                let st = self.states.map(|s| s[j]);
                let goal = self.goals.map(|g| g[j]);
                let pos = st.map(|s| s.position);
                let num = |v: Option<u16>| v.map_or("–".to_string(), |x| x.to_string());
                let diff = match (goal, pos) {
                    (Some(g), Some(p)) => format!("{:+}", g as i32 - p as i32),
                    _ => "–".into(),
                };
                let from_level = pos.map_or("–".to_string(), |p| format!("{:+}", p as i32 - reference[j] as i32));
                let range = if j < 7 {
                    let lim = ctx.cfg.safety.pose_limit_ticks[j] as f64;
                    let off = pos.map_or(0.0, |p| p as f64 - reference[j] as f64);
                    let w = 17usize;
                    let i = (((off / lim).clamp(-1.0, 1.0) + 1.0) / 2.0 * (w - 1) as f64).round() as usize;
                    let mut bar: Vec<char> = "─".repeat(w).chars().collect();
                    bar[w / 2] = '┼';
                    bar[i] = '●';
                    format!("├{}┤ ±{}", bar.into_iter().collect::<String>(), lim as u16)
                } else {
                    format!("torque {}", if self.torque8 { "on" } else { "OFF" })
                };
                let load = st.map_or(0, |s| (s.load as i32).abs());
                let load_color = if load >= 300 { Color::Red } else if load >= 150 { Color::Yellow } else { Color::Green };
                let bar_n = (load as usize * 12 / 1000).min(12);
                let load_txt = format!("{}{} {:>4}", "█".repeat(bar_n), "·".repeat(12 - bar_n), load);
                let style = if sel { Style::new().fg(Color::Black).bg(Color::Cyan).bold() } else { Style::new() };
                Row::new(vec![
                    Cell::from(format!("{}M{}", if sel { "▸ " } else { "  " }, j + 1)).style(style),
                    Cell::from(NAMES[j]),
                    Cell::from(num(goal)),
                    Cell::from(num(pos)),
                    Cell::from(diff),
                    Cell::from(from_level),
                    Cell::from(range),
                    Cell::from(Span::styled(load_txt, Style::new().fg(load_color))),
                    Cell::from(st.map_or("–".into(), |s| format!("{}°C", s.temperature_c))),
                ])
            })
            .collect();
        let step = if self.sel == 7 { GRIP_STEPS[self.grip_step_i] } else { STEPS[self.step_i] };
        let title = format!(
            " joints · step {} tick{} ({:.1}°){} ",
            step,
            if step == 1 { "" } else { "s" },
            step as f64 * 360.0 / 4096.0,
            if self.moving { " · moving…" } else { "" }
        );
        f.render_widget(
            Table::new(
                rows,
                [
                    Constraint::Length(6),
                    Constraint::Length(12),
                    Constraint::Length(6),
                    Constraint::Length(8),
                    Constraint::Length(6),
                    Constraint::Length(7),
                    Constraint::Length(25),
                    Constraint::Length(18),
                    Constraint::Length(6),
                ],
            )
            .header(Row::new(vec!["", "joint", "goal", "present", "miss", "Δlevel", "range around level", "load", "temp"]).style(Style::new().fg(Color::DarkGray)))
            .block(Block::bordered().title(title)),
            tbl,
        );
        let dim = Style::new().fg(Color::DarkGray);
        let pose = |p: Option<[u16; 8]>| p.map_or("not saved".to_string(), |p| format!("{:?}", p));
        let mut lines = vec![
            Line::from(vec![Span::styled(" level ", dim), Span::raw(pose(ctx.cfg.poses.level))]),
            Line::from(vec![Span::styled(" rest  ", dim), Span::raw(pose(ctx.cfg.poses.rest))]),
            Line::from(vec![Span::styled(" grip  ", dim), Span::raw(ctx.cfg.poses.grip.map_or("not saved".into(), |g| g.to_string()))]),
            Line::from(vec![Span::styled(" throw ", dim), Span::raw(ctx.cfg.throw.end.map_or("not saved".into(), |p| format!("{:?}", p)))]),
        ];
        if self.pending == Some('t') {
            lines.push(Line::from(Span::styled(
                " THROW PREVIEW, NO BALL: flip to the throw end pose and back to level. Speed? 1 = 10 % · 2 = 25 % · 3 = 50 % · 4 = 100 %, any other key = cancel",
                Style::new().fg(Color::Yellow).bold(),
            )));
        } else if let Some(p) = self.pending {
            lines.push(Line::from(Span::styled(
                format!(" Move every joint to the {} pose (smooth {:.0} s)? y = go, any other key = cancel", if p == 'l' { "LEVEL" } else { "REST" }, ctx.cfg.poses.move_time_s),
                Style::new().fg(Color::Yellow).bold(),
            )));
        } else if let Some((good, m)) = &self.msg {
            lines.push(Line::from(Span::styled(format!(" {m}"), Style::new().fg(if *good { Color::Green } else { Color::Red }))));
        }
        if self.sel == 7 {
            lines.push(Line::from(Span::styled(
                " Gripper: ← closes, → opens (+ for bigger steps). Tighten until the paddle no longer wiggles; keep load under ~300.",
                dim,
            )));
        }
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(" poses ")), info);
    }
}
