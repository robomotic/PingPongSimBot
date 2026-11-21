//! Servos screen: the position-loop P gain of every joint, read from the servos and written back to their EEPROM.
//! The follower ships with P 16 (the station writes 16 when it freezes a calibration; Feetech's default is 32).
//! A higher P makes a joint stiffer: smaller tracking error and lag while balancing, but more prone to buzzing.

use super::device::{self, DevCmd, DevMsg, Device};
use super::Ctx;
use crate::bus::ServoGains;
use crossterm::event::{KeyCode, KeyEvent};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Cell, Paragraph, Row, Table};
use ratatui::Frame;

pub const HELP: [&str; 12] = [
    "↑ ↓        choose a joint",
    "← →        change its P gain by 1 (not written yet: shown in yellow)",
    "d          set the selected joint to the Feetech default, 32",
    "*          copy the selected joint's new P to M1-M7 (the gripper keeps its own)",
    "w          WRITE the changed P gains to the servos (needs ARMED, asks first, reads back to verify)",
    "r          read the servos again (discards unwritten changes)",
    "Esc        back to Home (releases the bus)",
    "",
    "P lives in each servo's EEPROM: a write is permanent, also for the station and teleop, until changed again.",
    "The station writes P 16 again when it freezes a calibration. Higher P = stiffer joint, less lag and tracking",
    "error; too high and the joint buzzes or oscillates. Change one step at a time and watch the arm.",
    "Max torque (EEPROM) and torque limit (RAM) are shown for reference; the throw raises the RAM limit by itself.",
];

const NAMES: [&str; 8] = ["base", "shoulder", "elbow", "forearm", "wrist roll", "wrist pitch", "wrist roll", "gripper"];
const P_MIN: u8 = 1;
const P_MAX: u8 = 100;

#[derive(Default)]
pub struct ServosScreen {
    dev: Option<Device>,
    gains: [Option<ServoGains>; 8],
    /// P to write, per joint (equal to the servo's value when unchanged).
    edit: [u8; 8],
    sel: usize,
    confirm: bool,
    busy: bool,
    msg: Option<(bool, String)>,
}

impl ServosScreen {
    pub fn enter(&mut self, ctx: &Ctx) {
        if self.dev.is_none() {
            self.gains = [None; 8];
            self.confirm = false;
            self.busy = true;
            self.msg = Some((true, if ctx.sim { "simulated arm".into() } else { "opening the bus: every joint holds where it is".into() }));
            self.dev = Some(device::open(&ctx.cfg, ctx.sim));
        }
    }

    pub fn leave(&mut self) {
        self.dev = None;
    }

    fn changes(&self) -> Vec<(u8, u8)> {
        (0..8)
            .filter_map(|j| self.gains[j].filter(|g| g.p != self.edit[j]).map(|_| (j as u8 + 1, self.edit[j])))
            .collect()
    }

    pub fn pump(&mut self) {
        let Some(dev) = &mut self.dev else { return };
        let mut closed = false;
        while let Ok(m) = dev.rx.try_recv() {
            match m {
                DevMsg::Opened { .. } => {
                    let _ = dev.tx.send(DevCmd::ReadGains);
                }
                DevMsg::Gains(g) => {
                    self.gains = g;
                    for j in 0..8 {
                        if let Some(x) = g[j] {
                            self.edit[j] = x.p;
                        }
                    }
                    self.busy = false;
                    if self.msg.as_ref().is_some_and(|(_, t)| t.starts_with("opening") || t.starts_with("simulated") || t.starts_with("reading")) {
                        self.msg = Some((true, "read from the servos".into()));
                    }
                }
                DevMsg::Info(m) => self.msg = Some((true, m)),
                DevMsg::Error(e) => {
                    self.busy = false;
                    self.msg = Some((false, e));
                }
                DevMsg::Closed => closed = true,
                _ => {}
            }
        }
        if closed {
            self.dev = None;
        }
    }

    /// Returns true to go back Home.
    pub fn key(&mut self, k: KeyEvent, ctx: &mut Ctx) -> bool {
        if self.confirm {
            self.confirm = false;
            if k.code == KeyCode::Char('y') {
                if !ctx.armed() {
                    self.msg = Some((false, "press a to ARM first".into()));
                } else if let Some(dev) = &self.dev {
                    let ch = self.changes();
                    self.busy = true;
                    self.msg = Some((true, format!("writing P to {} servo(s)...", ch.len())));
                    let _ = dev.tx.send(DevCmd::WriteP(ch));
                    ctx.disarm();
                }
            } else {
                self.msg = Some((true, "not written".into()));
            }
            return false;
        }
        let loaded = self.gains[self.sel].is_some();
        match k.code {
            KeyCode::Up => self.sel = self.sel.saturating_sub(1),
            KeyCode::Down => self.sel = (self.sel + 1).min(7),
            KeyCode::Left | KeyCode::Right if loaded => {
                let e = &mut self.edit[self.sel];
                *e = if k.code == KeyCode::Right { e.saturating_add(1).min(P_MAX) } else { e.saturating_sub(1).max(P_MIN) };
            }
            KeyCode::Char('d') if loaded => self.edit[self.sel] = 32,
            KeyCode::Char('*') if loaded => {
                let v = self.edit[self.sel];
                for j in 0..7 {
                    if self.gains[j].is_some() {
                        self.edit[j] = v;
                    }
                }
                self.msg = Some((true, format!("P {v} for M1-M7 (not written yet)")));
            }
            KeyCode::Char('w') => {
                if self.changes().is_empty() {
                    self.msg = Some((true, "nothing changed".into()));
                } else if !ctx.armed() {
                    self.msg = Some((false, "press a to ARM first: writing gains changes how the joints hold".into()));
                } else {
                    self.confirm = true;
                }
            }
            KeyCode::Char('r') => {
                if let Some(dev) = &self.dev {
                    self.busy = true;
                    self.msg = Some((true, "reading...".into()));
                    let _ = dev.tx.send(DevCmd::ReadGains);
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

    pub fn keys(&self, ctx: &Ctx) -> Vec<(&'static str, &'static str)> {
        if self.confirm {
            return vec![("y", "write"), ("any key", "cancel")];
        }
        let mut v = vec![("↑↓", "joint"), ("←→", "P"), ("d", "default 32"), ("*", "copy to M1-M7"), ("w", "write"), ("r", "re-read")];
        if !ctx.sim {
            v.insert(0, ("a", "arm"));
        }
        v.extend([("Esc", "home"), ("?", "help")]);
        v
    }

    pub fn draw(&self, f: &mut Frame, area: Rect, _ctx: &Ctx) {
        let [tbl, info] = Layout::vertical([Constraint::Length(12), Constraint::Min(4)]).areas(area);
        let dim = Style::new().fg(Color::DarkGray);
        let rows: Vec<Row> = (0..8)
            .map(|j| {
                let sel = j == self.sel;
                let g = self.gains[j];
                let num = |v: Option<String>| v.unwrap_or_else(|| "–".into());
                let changed = g.is_some_and(|g| g.p != self.edit[j]);
                let new = if g.is_some() { format!("{}", self.edit[j]) } else { "–".into() };
                let pct = |v: u16| format!("{:.0} %", v as f64 / 10.0);
                Row::new(vec![
                    Cell::from(if sel { "▸" } else { " " }),
                    Cell::from(format!("M{}", j + 1)),
                    Cell::from(NAMES[j]),
                    Cell::from(num(g.map(|g| g.p.to_string()))),
                    Cell::from(Span::styled(
                        if sel && g.is_some() { format!("◂ {new} ▸") } else { new },
                        if changed { Style::new().fg(Color::Yellow).bold() } else { Style::new() },
                    )),
                    Cell::from(num(g.map(|g| g.d.to_string()))),
                    Cell::from(num(g.map(|g| g.i.to_string()))),
                    Cell::from(num(g.map(|g| pct(g.max_torque)))),
                    Cell::from(num(g.map(|g| pct(g.torque_limit)))),
                ])
                .style(if sel { Style::new().bg(Color::Indexed(236)) } else { Style::new() })
            })
            .collect();
        let widths = [
            Constraint::Length(2),
            Constraint::Length(4),
            Constraint::Length(12),
            Constraint::Length(8),
            Constraint::Length(10),
            Constraint::Length(4),
            Constraint::Length(4),
            Constraint::Length(12),
            Constraint::Length(13),
        ];
        let title = if self.busy { " servo gains · reading/writing… " } else { " servo gains (position loop) " };
        f.render_widget(
            Table::new(rows, widths)
                .header(Row::new(vec!["", "", "joint", "P now", "P new", "D", "I", "max torque", "torque limit"]).style(dim))
                .block(Block::bordered().title(title)),
            tbl,
        );
        let mut lines = Vec::new();
        let ch = self.changes();
        if self.confirm {
            let list: Vec<String> = ch.iter().map(|(id, p)| format!("M{id} → {p}")).collect();
            lines.push(Line::from(Span::styled(
                format!(" WRITE P to the servos' EEPROM ({})? Permanent, also for the station and teleop. y = write, any other key = cancel", list.join(", ")),
                Style::new().fg(Color::Yellow).bold(),
            )));
        } else if let Some((good, m)) = &self.msg {
            lines.push(Line::from(Span::styled(format!(" {m}"), Style::new().fg(if *good { Color::Green } else { Color::Red }))));
        }
        if !ch.is_empty() && !self.confirm {
            lines.push(Line::from(Span::styled(format!(" {} change(s) not written yet: w to write, r to discard", ch.len()), Style::new().fg(Color::Yellow))));
        }
        lines.push(Line::from(""));
        lines.push(Line::from(Span::styled(
            " P = how hard a joint pushes back per tick of error. The follower has 16; Feetech's default is 32. Higher: stiffer, less lag",
            dim,
        )));
        lines.push(Line::from(Span::styled(
            " and tracking error while balancing. Too high: buzzing or oscillation. Change one step at a time and watch the arm.",
            dim,
        )));
        lines.push(Line::from(Span::styled(
            " Stored in each servo (EEPROM): stays after power-off; the station resets it to 16 when it freezes a calibration.",
            dim,
        )));
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(" about ")), info);
    }
}
