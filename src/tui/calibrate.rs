//! Calibrate screen: camera map (pixel → blade mm) wizard. The ball is placed on 5 marks; each is captured
//! automatically once the ball has been still for ~1 s. No arm motion. (Axis calibration and the joint probe still use
//! the `calib-axes` / `probe-joints` commands.)

use super::camview::{self, SimBall};
use super::Ctx;
use crate::analysis::median;
use crate::config::Config;
use crate::estimator::Homography;
use crate::vision::Camera;
use crossterm::event::{KeyCode, KeyEvent};
use nalgebra::Vector2;
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style, Stylize};
use ratatui::symbols::Marker;
use ratatui::text::{Line, Span};
use ratatui::widgets::canvas::{Canvas, Circle, Line as CLine};
use ratatui::widgets::{Block, Gauge, Paragraph};
use ratatui::Frame;
use std::collections::VecDeque;
use std::time::Instant;

pub const HELP: [&str; 13] = [
    "← →        before starting: distance of the outer marks from the centre (40 or 50 mm)",
    "Space      start · during capture: capture now, without waiting for stillness",
    "r          redo the current mark      Backspace  go back one mark",
    "Enter / s  after all 5: save the map to paddle.toml (the axes calibration is cleared: redo it)",
    "Esc        cancel, back to Home",
    "",
    "Mark the blade: the centre, and the chosen distance from it in 4 directions at right angles:",
    "+x right, −x left (as seen from the gripper), +y away from the gripper, −y towards the handle.",
    "Put the ball on the highlighted mark and take your hand out of view: it is captured when it",
    "has been still for ~1 s (the bar fills). The paddle must be level (Pose → l).",
    "A worst error under ~3 mm is good. In simulation the ball places itself.",
    "",
    "",
];

const MARKS: [(&str, f64, f64); 5] = [("centre", 0.0, 0.0), ("+x", 1.0, 0.0), ("−x", -1.0, 0.0), ("+y", 0.0, 1.0), ("−y", 0.0, -1.0)];

enum Stage {
    Setup,
    Capture { i: usize, since: Instant },
    Done { h: Homography, errors: Vec<f64> },
}

pub struct CalibrateScreen {
    cam: Option<Camera>,
    ball: Option<SimBall>,
    stage: Stage,
    offset: f64,
    captured: Vec<Option<Vector2<f64>>>,
    window: VecDeque<(Instant, Option<Vector2<f64>>)>,
    still: f64,
    msg: Option<(bool, String)>,
}

impl Default for CalibrateScreen {
    fn default() -> Self {
        Self { cam: None, ball: None, stage: Stage::Setup, offset: 40.0, captured: vec![None; 5], window: VecDeque::new(), still: 0.0, msg: None }
    }
}

impl CalibrateScreen {
    pub fn enter(&mut self, ctx: &Ctx) {
        if self.cam.is_some() {
            return;
        }
        self.stage = Stage::Setup;
        self.captured = vec![None; 5];
        match camview::open(ctx, false) {
            Ok((cam, ball)) => {
                self.cam = Some(cam);
                self.ball = ball;
                self.msg = None;
            }
            Err(e) => self.msg = Some((false, format!("{e:#}"))),
        }
    }

    pub fn leave(&mut self) {
        self.cam = None;
        self.ball = None;
    }

    fn mark_mm(&self, i: usize) -> Vector2<f64> {
        Vector2::new(MARKS[i].1 * self.offset, MARKS[i].2 * self.offset)
    }

    fn goto(&mut self, i: usize) {
        self.window.clear();
        self.still = 0.0;
        if i >= 5 {
            self.finish();
        } else {
            self.stage = Stage::Capture { i, since: Instant::now() };
            if let Some(b) = &self.ball {
                if let Ok(mut g) = b.lock() {
                    *g = None; // "hand moves the ball"
                }
            }
        }
    }

    fn finish(&mut self) {
        let px: Vec<Vector2<f64>> = self.captured.iter().flatten().copied().collect();
        let mm: Vec<Vector2<f64>> = (0..5).map(|i| self.mark_mm(i)).collect();
        match Homography::fit(&px, &mm) {
            Ok(h) => {
                let errors = px.iter().zip(&mm).map(|(p, m)| (h.apply(*p) - m).norm()).collect();
                self.stage = Stage::Done { h, errors };
            }
            Err(e) => {
                self.msg = Some((false, format!("fit failed: {e:#}")));
                self.stage = Stage::Setup;
            }
        }
    }

    pub fn pump(&mut self) {
        let Stage::Capture { i, since } = self.stage else {
            if let Some(cam) = &mut self.cam {
                let _ = cam.rx.borrow_and_update();
            }
            return;
        };
        let target = self.mark_mm(i);
        let Some(cam) = &mut self.cam else { return };
        // Simulation: the "hand" puts the ball on the mark after 1.2 s, with a little placement error.
        if let Some(b) = &self.ball {
            if since.elapsed().as_secs_f64() > 1.2 {
                if let Ok(mut g) = b.lock() {
                    if g.is_none() {
                        let j = since.elapsed().as_nanos() as f64;
                        *g = Some(target + Vector2::new((j % 7.0 - 3.0) * 0.1, (j % 5.0 - 2.0) * 0.1));
                    }
                }
            }
        }
        if cam.rx.has_changed().unwrap_or(false) {
            let d = *cam.rx.borrow_and_update();
            self.window.push_back((d.t, d.blob.map(|b| b.px)));
        }
        while self.window.front().is_some_and(|(t, _)| t.elapsed().as_secs_f64() > 1.0) {
            self.window.pop_front();
        }
        // Stillness: share of the last second with the ball seen within 1.5 px of the window median.
        let pts: Vec<Vector2<f64>> = self.window.iter().filter_map(|w| w.1).collect();
        if pts.len() >= 5 && pts.len() + 1 >= self.window.len() {
            let m = Vector2::new(median(&pts.iter().map(|p| p.x).collect::<Vec<_>>()), median(&pts.iter().map(|p| p.y).collect::<Vec<_>>()));
            let near = pts.iter().filter(|p| (*p - m).norm() < 1.5).count();
            let span = self.window.back().unwrap().0.saturating_duration_since(self.window.front().unwrap().0).as_secs_f64();
            self.still = (near as f64 / pts.len() as f64) * (span / 0.9).min(1.0);
            if self.still >= 0.99 {
                self.captured[i] = Some(m);
                self.goto(i + 1);
            }
        } else {
            self.still = 0.0;
        }
    }

    fn capture_now(&mut self) {
        if let Stage::Capture { i, .. } = self.stage {
            let pts: Vec<Vector2<f64>> = self.window.iter().filter_map(|w| w.1).collect();
            if pts.is_empty() {
                self.msg = Some((false, "no ball seen".into()));
                return;
            }
            self.captured[i] = Some(Vector2::new(median(&pts.iter().map(|p| p.x).collect::<Vec<_>>()), median(&pts.iter().map(|p| p.y).collect::<Vec<_>>())));
            self.goto(i + 1);
        }
    }

    /// Returns true to go back Home.
    pub fn key(&mut self, k: KeyEvent, ctx: &mut Ctx) -> bool {
        match (&self.stage, k.code) {
            (_, KeyCode::Esc) | (_, KeyCode::Char('q')) => {
                self.leave();
                return true;
            }
            (Stage::Setup, KeyCode::Left) => self.offset = 40.0,
            (Stage::Setup, KeyCode::Right) => self.offset = 50.0,
            (Stage::Setup, KeyCode::Char(' ')) => {
                if self.cam.is_none() {
                    self.msg = Some((false, "no camera".into()));
                } else {
                    self.captured = vec![None; 5];
                    self.msg = None;
                    self.goto(0);
                }
            }
            (Stage::Capture { .. }, KeyCode::Char(' ')) => self.capture_now(),
            (Stage::Capture { i, .. }, KeyCode::Char('r')) => {
                let i = *i;
                self.captured[i] = None;
                self.goto(i);
            }
            (Stage::Capture { i, .. }, KeyCode::Backspace) => {
                let i = i.saturating_sub(1);
                self.captured[i] = None;
                self.goto(i);
            }
            (Stage::Done { h, .. }, KeyCode::Enter | KeyCode::Char('s')) => {
                let h = *h;
                let res = Config::load(&ctx.cfg_path).and_then(|mut c| {
                    c.camera.homography = Some(h.to_row_major());
                    c.camera.axes = None;
                    std::fs::write(&ctx.cfg_path, toml::to_string_pretty(&c)?)?;
                    Ok(c)
                });
                match res {
                    Ok(c) => {
                        ctx.cfg.camera = c.camera;
                        self.msg = Some((true, "saved the camera map; the axes calibration was cleared (run calib-axes next)".into()));
                        self.stage = Stage::Setup;
                    }
                    Err(e) => self.msg = Some((false, format!("save failed: {e:#}"))),
                }
            }
            (Stage::Done { .. }, KeyCode::Char('r')) => self.goto(0),
            _ => {}
        }
        false
    }

    pub fn keys(&self) -> Vec<(&'static str, &'static str)> {
        let mut v = match self.stage {
            Stage::Setup => vec![("←→", "40/50 mm"), ("Space", "start")],
            Stage::Capture { .. } => vec![("Space", "capture now"), ("r", "redo"), ("⌫", "back")],
            Stage::Done { .. } => vec![("Enter", "save"), ("r", "start over")],
        };
        v.extend([("Esc", "cancel"), ("?", "help")]);
        v
    }

    pub fn draw(&self, f: &mut Frame, area: Rect) {
        let [img, side] = Layout::horizontal([Constraint::Min(40), Constraint::Length(50)]).areas(area);
        let block = Block::bordered().title(" camera ");
        let inner = block.inner(img);
        f.render_widget(block, img);
        if let Some(cam) = &self.cam {
            if let Some(p) = cam.preview.borrow().clone() {
                let mut marks: Vec<(f64, f64, Color)> = self.captured.iter().flatten().map(|p| (p.x, p.y, Color::LightGreen)).collect();
                if let Some((_, Some(p))) = self.window.back() {
                    marks.push((p.x, p.y, Color::Yellow));
                }
                camview::draw_preview(f.buffer_mut(), inner, &p, false, &marks);
            }
        }
        let [diagram, steps, bar, info] =
            Layout::vertical([Constraint::Length(17), Constraint::Length(8), Constraint::Length(3), Constraint::Min(3)]).areas(side);
        let current = if let Stage::Capture { i, .. } = self.stage { Some(i) } else { None };
        let offset = self.offset;
        let captured: Vec<bool> = self.captured.iter().map(|c| c.is_some()).collect();
        f.render_widget(
            Canvas::default()
                .block(Block::bordered().title(format!(" marks on the blade (±{offset:.0} mm) ")))
                .marker(Marker::Braille)
                .x_bounds([-85.0, 85.0])
                .y_bounds([-85.0, 85.0])
                .paint(move |c| {
                    c.draw(&Circle { x: 0.0, y: 0.0, radius: 75.0, color: Color::Gray });
                    c.draw(&CLine { x1: -offset, y1: 0.0, x2: offset, y2: 0.0, color: Color::DarkGray });
                    c.draw(&CLine { x1: 0.0, y1: -offset, x2: 0.0, y2: offset, color: Color::DarkGray });
                    for (i, (_, mx, my)) in MARKS.iter().enumerate() {
                        let color = if Some(i) == current { Color::Yellow } else if captured[i] { Color::Green } else { Color::DarkGray };
                        for r in [8.0, 6.0, 4.0] {
                            c.draw(&Circle { x: mx * offset, y: my * offset, radius: r, color });
                        }
                    }
                    c.print(-8.0, 80.0, Line::from("far").fg(Color::DarkGray));
                    c.print(-14.0, -84.0, Line::from("handle").fg(Color::DarkGray));
                }),
            diagram,
        );
        let mut lines = Vec::new();
        for (i, (name, _, _)) in MARKS.iter().enumerate() {
            let (icon, style) = match (Some(i) == current, self.captured[i]) {
                (true, _) => ("▸", Style::new().fg(Color::Yellow).bold()),
                (false, Some(_)) => ("✓", Style::new().fg(Color::Green)),
                _ => (" ", Style::new().fg(Color::DarkGray)),
            };
            let px = self.captured[i].map_or(String::new(), |p| format!("px ({:.1}, {:.1})", p.x, p.y));
            lines.push(Line::from(vec![Span::styled(format!(" {icon} {name:<7}"), style), Span::raw(px)]));
        }
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(" steps ")), steps);
        let (ratio, label) = match &self.stage {
            Stage::Capture { i, .. } => (self.still.clamp(0.0, 1.0), format!("hold still on {}", MARKS[*i].0)),
            Stage::Done { .. } => (1.0, "all captured".into()),
            Stage::Setup => (0.0, "press Space to start".into()),
        };
        f.render_widget(Gauge::default().block(Block::bordered().title(" stillness ")).gauge_style(Style::new().fg(Color::Yellow)).ratio(ratio).label(label), bar);
        let mut info_lines = Vec::new();
        if let Stage::Done { errors, .. } = &self.stage {
            let worst = errors.iter().cloned().fold(0.0, f64::max);
            info_lines.push(Line::from(Span::styled(
                format!(" worst error {worst:.2} mm  {}", if worst < 3.0 { "✓ good" } else { "✗ redo: check the marks" }),
                Style::new().fg(if worst < 3.0 { Color::Green } else { Color::Red }).bold(),
            )));
            info_lines.push(Line::from(format!(" per mark: {}", errors.iter().map(|e| format!("{e:.1}")).collect::<Vec<_>>().join(" · "))));
            info_lines.push(Line::from(Span::styled(" Enter saves (clears the axes calibration)", Style::new().fg(Color::DarkGray))));
        } else if let Stage::Setup = self.stage {
            info_lines.push(Line::from(" Paddle level? Marks drawn? Ball ready? Space."));
        }
        if let Some((good, m)) = &self.msg {
            info_lines.push(Line::from(Span::styled(format!(" {m}"), Style::new().fg(if *good { Color::Green } else { Color::Red }))));
        }
        f.render_widget(Paragraph::new(info_lines).wrap(ratatui::widgets::Wrap { trim: false }).block(Block::bordered().title(" result ")), info);
    }
}
