//! Camera screen: live preview, the detector's view (mask), live colour/exposure tuning, detection statistics.

use super::camview::{self, SimBall};
use super::Ctx;
use crate::config::Config;
use crate::estimator::Homography;
use crate::vision::{Camera, Detection, Tuning};
use crossterm::event::{KeyCode, KeyEvent};
use nalgebra::Vector2;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Paragraph};
use ratatui::Frame;
use std::collections::VecDeque;
use std::time::Instant;

pub const HELP: [&str; 12] = [
    "↑ ↓        choose a setting (hue, saturation, brightness limits, min blob size, exposure)",
    "← →        change it (applied to the live camera at once)   + / -  coarse / fine",
    "m          show the detector's view: pixels inside the colour range turn magenta",
    "s          save the settings to paddle.toml ([camera])",
    "b          simulation: put / remove the ball    i j k l  simulation: move the ball",
    "Esc        back to Home (releases the camera)",
    "",
    "Hue is OpenCV-style 0–179 (orange ≈ 5–25). The ball should be one solid magenta blob with m on,",
    "and nothing else should light up. 'blobs' counts separate patches: more than 1 means false detections.",
    "Exposure is in 100 µs units (40 = 4 ms). Shorter = darker, less motion blur.",
    "The whole frame is searched here; balancing only searches the blade (needs the camera map).",
    "",
];

const PARAMS: [(&str, u32, u32); 7] = [
    ("hue min", 1, 5),
    ("hue max", 1, 5),
    ("saturation min", 1, 10),
    ("brightness min", 1, 10),
    ("brightness max", 1, 10),
    ("min blob area", 5, 50),
    ("exposure", 5, 20),
];

pub struct CameraScreen {
    cam: Option<Camera>,
    ball: Option<SimBall>,
    tuning: Option<Tuning>,
    last: Option<Detection>,
    hist: VecDeque<(Instant, bool, u32)>,
    sel: usize,
    coarse: bool,
    mask: bool,
    msg: Option<(bool, String)>,
}

impl Default for CameraScreen {
    fn default() -> Self {
        Self { cam: None, ball: None, tuning: None, last: None, hist: VecDeque::new(), sel: 0, coarse: false, mask: false, msg: None }
    }
}

fn get(t: &Tuning, i: usize) -> i64 {
    match i {
        0 => t.hsv_low[0] as i64,
        1 => t.hsv_high[0] as i64,
        2 => t.hsv_low[1] as i64,
        3 => t.hsv_low[2] as i64,
        4 => t.hsv_high[2] as i64,
        5 => t.min_area as i64,
        _ => t.exposure.unwrap_or(-1),
    }
}

fn set(t: &mut Tuning, i: usize, v: i64) {
    let u8c = |v: i64, max: i64| v.clamp(0, max) as u8;
    match i {
        0 => t.hsv_low[0] = u8c(v, 179),
        1 => t.hsv_high[0] = u8c(v, 179),
        2 => t.hsv_low[1] = u8c(v, 255),
        3 => t.hsv_low[2] = u8c(v, 255),
        4 => t.hsv_high[2] = u8c(v, 255),
        5 => t.min_area = v.clamp(1, 20000) as u32,
        _ => t.exposure = Some(v.clamp(1, 5000)),
    }
}

impl CameraScreen {
    pub fn enter(&mut self, ctx: &Ctx) {
        if self.cam.is_some() {
            return;
        }
        match camview::open(ctx, false) {
            Ok((cam, ball)) => {
                self.tuning = cam.tuning.lock().ok().map(|g| g.1.clone());
                self.cam = Some(cam);
                self.ball = ball;
                self.hist.clear();
                self.msg = Some((true, if ctx.sim { "simulated camera".into() } else { "camera open".into() }));
            }
            Err(e) => self.msg = Some((false, format!("{e:#}"))),
        }
    }

    pub fn leave(&mut self) {
        self.cam = None;
        self.ball = None;
    }

    pub fn pump(&mut self) {
        let Some(cam) = &mut self.cam else { return };
        if cam.rx.has_changed().unwrap_or(false) {
            let d = *cam.rx.borrow_and_update();
            self.hist.push_back((d.t, d.blob.is_some(), d.n_blobs));
            while self.hist.front().is_some_and(|(t, _, _)| t.elapsed().as_secs_f64() > 3.0) {
                self.hist.pop_front();
            }
            self.last = Some(d);
        }
    }

    fn move_ball(&self, dx: f64, dy: f64) {
        if let Some(b) = &self.ball {
            if let Ok(mut g) = b.lock() {
                if let Some(p) = g.as_mut() {
                    *p += Vector2::new(dx, dy);
                }
            }
        }
    }

    /// Returns true to go back Home.
    pub fn key(&mut self, k: KeyEvent, ctx: &mut Ctx) -> bool {
        match k.code {
            KeyCode::Up => self.sel = self.sel.saturating_sub(1),
            KeyCode::Down => self.sel = (self.sel + 1).min(PARAMS.len() - 1),
            KeyCode::Char('+') | KeyCode::Char('=') => self.coarse = true,
            KeyCode::Char('-') | KeyCode::Char('_') => self.coarse = false,
            KeyCode::Left | KeyCode::Right => {
                if let (Some(t), Some(cam)) = (self.tuning.as_mut(), &self.cam) {
                    let (_, fine, coarse) = PARAMS[self.sel];
                    let step = if self.coarse { coarse } else { fine } as i64;
                    let cur = get(t, self.sel);
                    let base = if cur < 0 { 40 } else { cur };
                    set(t, self.sel, base + if k.code == KeyCode::Right { step } else { -step });
                    let new = t.clone();
                    cam.set_tuning(|x| {
                        x.hsv_low = new.hsv_low;
                        x.hsv_high = new.hsv_high;
                        x.min_area = new.min_area;
                        x.exposure = new.exposure;
                    });
                }
            }
            KeyCode::Char('m') => self.mask = !self.mask,
            KeyCode::Char('s') => {
                if let Some(t) = &self.tuning {
                    let res = Config::load(&ctx.cfg_path).and_then(|mut c| {
                        c.camera.hsv_low = t.hsv_low;
                        c.camera.hsv_high = t.hsv_high;
                        c.camera.min_area = t.min_area;
                        c.camera.exposure = t.exposure;
                        std::fs::write(&ctx.cfg_path, toml::to_string_pretty(&c)?)?;
                        Ok(c)
                    });
                    match res {
                        Ok(c) => {
                            ctx.cfg.camera = c.camera;
                            self.msg = Some((true, format!("saved camera settings to {}", ctx.cfg_path.display())));
                        }
                        Err(e) => self.msg = Some((false, format!("save failed: {e:#}"))),
                    }
                }
            }
            KeyCode::Char('b') => {
                if let Some(b) = &self.ball {
                    if let Ok(mut g) = b.lock() {
                        *g = if g.is_some() { None } else { Some(Vector2::new(0.0, 0.0)) };
                    }
                }
            }
            KeyCode::Char('i') => self.move_ball(0.0, 5.0),
            KeyCode::Char('k') => self.move_ball(0.0, -5.0),
            KeyCode::Char('j') => self.move_ball(-5.0, 0.0),
            KeyCode::Char('l') => self.move_ball(5.0, 0.0),
            KeyCode::Esc | KeyCode::Char('q') => {
                self.leave();
                return true;
            }
            _ => {}
        }
        false
    }

    pub fn keys(&self, ctx: &Ctx) -> Vec<(&'static str, &'static str)> {
        let mut v = vec![("↑↓", "setting"), ("←→", "change"), ("+/-", "step"), ("m", if self.mask { "camera view" } else { "detector view" }), ("s", "save")];
        if ctx.sim {
            v.push(("b ijkl", "sim ball"));
        }
        v.extend([("Esc", "home"), ("?", "help")]);
        v
    }

    pub fn draw(&self, f: &mut Frame, area: Rect, ctx: &Ctx) {
        let [img, side] = Layout::horizontal([Constraint::Min(40), Constraint::Length(46)]).areas(area);
        let title = if self.mask { " camera · detector view (magenta = in colour range) " } else { " camera " };
        let block = Block::bordered().title(title);
        let inner = block.inner(img);
        f.render_widget(block, img);
        if let Some(cam) = &self.cam {
            if let Some(p) = cam.preview.borrow().clone() {
                let marks: Vec<(f64, f64, Color)> = self.last.and_then(|d| d.blob).map(|b| vec![(b.px.x, b.px.y, Color::LightGreen)]).unwrap_or_default();
                camview::draw_preview(f.buffer_mut(), inner, &p, self.mask, &marks);
            }
        }
        let [tune, stats] = Layout::vertical([Constraint::Length(PARAMS.len() as u16 + 3), Constraint::Min(0)]).areas(side);
        let mut lines = Vec::new();
        if let Some(t) = &self.tuning {
            for (i, (name, _, _)) in PARAMS.iter().enumerate() {
                let sel = i == self.sel;
                let v = get(t, i);
                let val = if v < 0 { "auto".to_string() } else { v.to_string() };
                lines.push(Line::from(vec![
                    Span::raw(if sel { " ▸ " } else { "   " }),
                    Span::styled(format!("{name:<16}"), if sel { Style::new().fg(Color::Black).bg(Color::Cyan).bold() } else { Style::new() }),
                    Span::styled(format!("{val:>6}"), if sel { Style::new().bold() } else { Style::new() }),
                    Span::raw(if sel { "  ◂ ▸" } else { "" }),
                ]));
            }
            lines.push(Line::from(Span::styled(format!("   step {}", if self.coarse { "coarse (+)" } else { "fine (-)" }), Style::new().fg(Color::DarkGray))));
        }
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(" colour & exposure ")), tune);

        let dim = Style::new().fg(Color::DarkGray);
        let mut s = Vec::new();
        let n = self.hist.len();
        if n >= 2 {
            let span = self.hist.back().unwrap().0.saturating_duration_since(self.hist.front().unwrap().0).as_secs_f64();
            let found = self.hist.iter().filter(|h| h.1).count();
            let extra = self.hist.iter().filter(|h| h.2 > 1).count();
            s.push(Line::from(format!(" {:.1} fps   found {:.0}%   >1 blob {:.0}%", (n - 1) as f64 / span.max(1e-3), 100.0 * found as f64 / n as f64, 100.0 * extra as f64 / n as f64)));
        }
        if let Some(d) = &self.last {
            match d.blob {
                Some(b) => {
                    s.push(Line::from(format!(" ball   px ({:.0}, {:.0})   area {}", b.px.x, b.px.y, b.area)));
                    s.push(Line::from(format!(" colour H {:.0}  S {:.0}  V {:.0}", b.hsv[0], b.hsv[1], b.hsv[2])));
                    if let Some(h) = ctx.cfg.camera.homography.map(|h| Homography::from_row_major(&h)) {
                        let mm = h.apply(b.px);
                        s.push(Line::from(format!(" blade  ({:+.1}, {:+.1}) mm", mm.x, mm.y)));
                    }
                }
                None => s.push(Line::from(Span::styled(" no ball found", Style::new().fg(Color::Yellow)))),
            }
            s.push(Line::from(Span::styled(format!(" blobs {}   detect {:.1} ms", d.n_blobs, d.detect_ms), dim)));
        }
        if let Some((good, m)) = &self.msg {
            s.push(Line::from(""));
            s.push(Line::from(Span::styled(format!(" {m}"), Style::new().fg(if *good { Color::Green } else { Color::Red }))));
        }
        f.render_widget(Paragraph::new(s).wrap(ratatui::widgets::Wrap { trim: false }).block(Block::bordered().title(" detection ")), stats);
    }
}
