//! Calibrate screen: camera map (pixel → blade mm) wizard. The ball is placed on 5 marks; each is captured
//! automatically once the ball has been still for ~1 s. No arm motion. (Axis calibration and the joint probe still use
//! the `calib-axes` / `probe-joints` commands.)

use super::camview::{self, SimBall};
use super::Ctx;
use crate::analysis::median;
use crate::calib::{AxesCmd, AxesEvent};
use crate::control::AxesMap;
use tokio::sync::mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender};
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

pub const HELP: [&str; 14] = [
    "← →        before starting: distance of the outer marks from the centre (40 or 50 mm)",
    "Space      start · during capture: capture now, without waiting for stillness",
    "r          redo the current mark      Backspace  go back one mark",
    "Enter / s  after all 5: save the map to paddle.toml (the axes calibration is cleared: redo it)",
    "t          axis calibration (after the map; needs ARMED): tilts the wrist 3° per axis, measures the roll",
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
    /// Axis calibration running in a background task (it owns the arm and the camera).
    Axes { tx: UnboundedSender<AxesCmd>, rx: UnboundedReceiver<AxesEvent>, waiting: Option<usize>, results: Vec<String>, map: Option<AxesMap> },
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
    confirm_axes: bool,
}

impl Default for CalibrateScreen {
    fn default() -> Self {
        Self { cam: None, ball: None, stage: Stage::Setup, offset: 40.0, captured: vec![None; 5], window: VecDeque::new(), still: 0.0, msg: None, confirm_axes: false }
    }
}

impl CalibrateScreen {
    pub fn enter(&mut self, ctx: &Ctx) {
        if self.cam.is_some() || matches!(self.stage, Stage::Axes { .. }) {
            return;
        }
        self.stage = Stage::Setup;
        self.captured = vec![None; 5];
        self.enter_camera(ctx);
    }

    fn enter_camera(&mut self, ctx: &Ctx) {
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

    fn start_axes(&mut self, ctx: &Ctx) {
        let mut cfg = ctx.cfg.clone();
        if ctx.sim {
            let ex = Config::example();
            if cfg.poses.level.is_none() {
                cfg.poses = ex.poses.clone();
            }
            if cfg.camera.homography.is_none() {
                cfg.camera.homography = ex.camera.homography;
            }
            // The simulator needs "true" axes to roll the ball; measuring them is the point of the exercise.
            cfg.camera.axes = cfg.camera.axes.or(ex.camera.axes);
        }
        if cfg.camera.homography.is_none() {
            self.msg = Some((false, "do the camera map first (Space)".into()));
            return;
        }
        self.cam = None; // the calibration task needs the camera
        self.ball = None;
        let (tx, mut crx) = unbounded_channel();
        let (etx, rx) = unbounded_channel();
        let sim = ctx.sim;
        tokio::spawn(async move {
            let etx2 = etx.clone();
            let mut emit = move |e: AxesEvent| {
                let _ = etx2.send(e);
            };
            let res = async {
                if sim {
                    let mut p = crate::sim::SimPlant::new(&cfg, crate::sim::SimParams::default(), 11)?;
                    crate::calib::run(&mut p, &cfg, 3.0, &mut crx, &mut emit).await
                } else {
                    #[cfg(target_os = "linux")]
                    {
                        emit(AxesEvent::Status("opening the bus and the camera...".into()));
                        let mut p = crate::balance::RealPlant::open_with(&cfg, false, true).await?;
                        crate::calib::run(&mut p, &cfg, 3.0, &mut crx, &mut emit).await
                    }
                    #[cfg(not(target_os = "linux"))]
                    {
                        anyhow::bail!("needs Linux")
                    }
                }
            }
            .await;
            if let Err(e) = res {
                let _ = etx.send(AxesEvent::Failed(format!("{e:#}")));
            }
        });
        self.stage = Stage::Axes { tx, rx, waiting: None, results: Vec::new(), map: None };
        self.msg = None;
    }

    fn pump_axes(&mut self) {
        let Stage::Axes { rx, waiting, results, map, .. } = &mut self.stage else { return };
        while let Ok(e) = rx.try_recv() {
            match e {
                AxesEvent::Status(s) => self.msg = Some((true, s)),
                AxesEvent::WaitBall(k) => {
                    *waiting = Some(k);
                    self.msg = Some((true, format!("axis {}: put the ball on the centre mark, let go, then press Space", ["u", "v"][k])));
                }
                AxesEvent::Axis { k, accel, ratio, dir_deg } => {
                    *waiting = None;
                    results.push(format!(
                        "axis {}: rolls towards {:.0}°, {:.0} mm/s², {:.2} of the model",
                        ["u", "v"][k],
                        dir_deg,
                        accel.norm(),
                        ratio
                    ));
                }
                AxesEvent::Retry(k, m) => {
                    *waiting = Some(k);
                    self.msg = Some((false, m));
                }
                AxesEvent::Done { map: m } => {
                    *map = Some(m);
                    let ang = m.axes_angle_deg();
                    self.msg = Some((ang > 60.0 && ang < 120.0, format!("axes {ang:.0}° apart (expect ~90). Enter saves.")));
                }
                AxesEvent::Failed(e) => self.msg = Some((false, e)),
            }
        }
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
        if matches!(self.stage, Stage::Axes { .. }) {
            self.pump_axes();
            return;
        }
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
        if self.confirm_axes {
            self.confirm_axes = false;
            if k.code == KeyCode::Char('y') {
                if ctx.armed() {
                    self.start_axes(ctx);
                } else {
                    self.msg = Some((false, "press a to ARM first: this tilts the wrist".into()));
                }
            }
            return false;
        }
        if let Stage::Axes { tx, waiting, map, .. } = &mut self.stage {
            match k.code {
                KeyCode::Char(' ') if waiting.is_some() => {
                    let _ = tx.send(AxesCmd::Go);
                    *waiting = None;
                }
                KeyCode::Enter | KeyCode::Char('s') if map.is_some() => {
                    let m = map.unwrap();
                    let res = Config::load(&ctx.cfg_path).and_then(|mut c| {
                        c.camera.axes = Some(m.to_row_major());
                        std::fs::write(&ctx.cfg_path, toml::to_string_pretty(&c)?)?;
                        Ok(c)
                    });
                    match res {
                        Ok(c) => {
                            ctx.cfg.camera = c.camera;
                            self.msg = Some((true, "saved the axes calibration".into()));
                        }
                        Err(e) => self.msg = Some((false, format!("save failed: {e:#}"))),
                    }
                }
                KeyCode::Esc | KeyCode::Char('q') => {
                    let _ = tx.send(AxesCmd::Stop);
                    self.stage = Stage::Setup;
                    self.enter_camera(ctx);
                }
                _ => {}
            }
            return false;
        }
        match (&self.stage, k.code) {
            (_, KeyCode::Esc) | (_, KeyCode::Char('q')) => {
                self.leave();
                return true;
            }
            (Stage::Setup, KeyCode::Char('t')) => {
                if ctx.armed() {
                    self.confirm_axes = true;
                } else {
                    self.msg = Some((false, "press a to ARM first: axis calibration tilts the wrist".into()));
                }
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
        if self.confirm_axes {
            return vec![("y", "start axis calibration"), ("any key", "cancel")];
        }
        let mut v = match &self.stage {
            Stage::Axes { waiting, map, .. } => {
                let mut v = Vec::new();
                if waiting.is_some() {
                    v.push(("Space", "ball is on the centre"));
                }
                if map.is_some() {
                    v.push(("Enter", "save axes"));
                }
                v.push(("Esc", "stop"));
                return v;
            }
            Stage::Setup => vec![("←→", "40/50 mm"), ("Space", "camera map"), ("t", "axes (moves wrist)")],
            Stage::Capture { .. } => vec![("Space", "capture now"), ("r", "redo"), ("⌫", "back")],
            Stage::Done { .. } => vec![("Enter", "save"), ("r", "start over")],
        };
        v.extend([("Esc", "cancel"), ("?", "help")]);
        v
    }

    pub fn draw(&self, f: &mut Frame, area: Rect) {
        if let Stage::Axes { waiting, results, map, .. } = &self.stage {
            let mut lines = vec![
                Line::from(Span::styled(" AXIS CALIBRATION: the wrist tilts 3° per axis and the camera measures where the ball rolls", Style::new().bold())),
                Line::from(""),
            ];
            for r in results {
                lines.push(Line::from(format!("   ✓ {r}")));
            }
            if let Some(k) = waiting {
                lines.push(Line::from(Span::styled(
                    format!("   ▸ axis {}: put the ball on the CENTRE mark, let go, press Space. Be ready to catch it.", ["u", "v"][*k]),
                    Style::new().fg(Color::Yellow).bold(),
                )));
            }
            if let Some(m) = map {
                lines.push(Line::from(""));
                lines.push(Line::from(format!("   angle between the axes {:.0}° (expect ~90)", m.axes_angle_deg())));
                lines.push(Line::from(Span::styled("   Enter saves · Esc leaves without saving", Style::new().fg(Color::DarkGray))));
            }
            if let Some((good, m)) = &self.msg {
                lines.push(Line::from(""));
                lines.push(Line::from(Span::styled(format!("   {m}"), Style::new().fg(if *good { Color::Green } else { Color::Red }))));
            }
            f.render_widget(Paragraph::new(lines).wrap(ratatui::widgets::Wrap { trim: false }).block(Block::bordered().title(" calibrate · axes ")), area);
            return;
        }
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
            Stage::Setup | Stage::Axes { .. } => (0.0, "press Space to start".into()),
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
            info_lines.push(Line::from(" Camera map: paddle level, marks drawn, ball ready → Space."));
            info_lines.push(Line::from(Span::styled(" Axes (after the map): a to ARM, then t. The wrist tilts 3°.", Style::new().fg(Color::DarkGray))));
        }
        if let Some((good, m)) = &self.msg {
            info_lines.push(Line::from(Span::styled(format!(" {m}"), Style::new().fg(if *good { Color::Green } else { Color::Red }))));
        }
        f.render_widget(Paragraph::new(info_lines).wrap(ratatui::widgets::Wrap { trim: false }).block(Block::bordered().title(" result ")), info);
    }
}
