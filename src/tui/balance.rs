//! Balance screen: start/stop, live view of the ball on the blade, live tuning, live statistics.

use super::Ctx;
use crate::analysis::{detrend, dominant_freq, lag_samples, mean, median, percentile};
use crate::balance::{run_loop, Balancer, Cmd, Event, Phase, Snapshot, Summary};
use crate::config::{Config, ControlConfig};
use crate::sim::{SimParams, SimPlant};
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style, Stylize};
use ratatui::symbols::Marker;
use ratatui::text::{Line, Span};
use ratatui::widgets::canvas::{Canvas, Circle, Line as CLine, Points};
use ratatui::widgets::{Block, Paragraph, Sparkline};
use ratatui::Frame;
use std::collections::VecDeque;
use std::path::PathBuf;
use std::time::Instant;
use tokio::sync::mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender};

pub const HELP: [&str; 18] = [
    "Space        start (real arm: needs ARMED) · while running: pause / resume (holds level)",
    "Esc          stop: the arm returns to its start pose · when stopped: back to Home",
    "b            simulation: put a ball on the paddle",
    "↑ ↓          choose a parameter in the tuning panel",
    "← →          change it (applied live while running)",
    "+ / -        coarse / fine steps",
    "0 1 2        profiles: 0 first run · 1 test A (kd 0.002) · 2 test B (+ slop compensation)",
    "s            save the tuning to paddle.toml ([control] only)",
    "Enter        after a run: open it in Results",
    "a            ARM / disarm the real arm",
    "t            THROW the ball back (needs ARMED and a balancing ball): 2 s countdown, then a fast flip to the",
    "             taught end pose ([throw] in paddle.toml) and back to level. Any key cancels the countdown.",
    "[ ]          throw aim: where on the blade the ball is thrown from (10 mm steps). Toward the handle (-) throws",
    "             further, toward the far edge (+) a short high lob. Shown as a cyan + on the blade; s saves it.",
    "",
    "Blade view: grey = blade (Ø150), red = edge limit, dim = deadband, orange = ball, yellow = trail, magenta = tilt.",
    "Up = far side of the blade (away from the gripper), down = handle.",
    "Statistics cover the last 10 s of balancing.",
];

pub enum Nav {
    Stay,
    Home,
    Results(Option<PathBuf>),
}

#[derive(Clone, Copy)]
struct Param {
    name: &'static str,
    unit: &'static str,
    fine: f64,
    coarse: f64,
    digits: usize,
}

const PARAMS: [Param; 9] = [
    Param { name: "kp", unit: "rad/mm", fine: 0.0001, coarse: 0.0005, digits: 4 },
    Param { name: "kd", unit: "rad·s/mm", fine: 0.0001, coarse: 0.0005, digits: 4 },
    Param { name: "ki", unit: "rad/mm·s", fine: 0.0001, coarse: 0.0005, digits: 4 },
    Param { name: "slop M5", unit: "ticks", fine: 1.0, coarse: 5.0, digits: 0 },
    Param { name: "slop M6", unit: "ticks", fine: 1.0, coarse: 5.0, digits: 0 },
    Param { name: "slop M7", unit: "ticks", fine: 1.0, coarse: 5.0, digits: 0 },
    Param { name: "deadband", unit: "mm", fine: 0.5, coarse: 2.0, digits: 1 },
    Param { name: "max tilt", unit: "deg", fine: 0.5, coarse: 2.0, digits: 1 },
    Param { name: "look-ahead", unit: "ms", fine: 10.0, coarse: 30.0, digits: 0 },
];

fn get(c: &ControlConfig, i: usize) -> f64 {
    match i {
        0 => c.kp,
        1 => c.kd,
        2 => c.ki,
        3..=5 => c.backlash_ticks[i - 3],
        6 => c.deadband_mm,
        7 => c.max_tilt_deg,
        _ => c.predict_ms,
    }
}

fn set(c: &mut ControlConfig, i: usize, v: f64) {
    let v = v.max(0.0);
    match i {
        0 => c.kp = v,
        1 => c.kd = v,
        2 => c.ki = v,
        3..=5 => c.backlash_ticks[i - 3] = v,
        6 => c.deadband_mm = v,
        7 => c.max_tilt_deg = v.clamp(1.0, 15.0),
        _ => c.predict_ms = v.min(400.0),
    }
}

/// (name, kp, kd, slop). Only these fields are changed by a profile.
const PROFILES: [(&str, f64, f64, [f64; 3]); 3] = [
    ("first run", 0.0017, 0.0008, [0.0, 0.0, 0.0]),
    ("test A", 0.0017, 0.002, [0.0, 0.0, 0.0]),
    ("test B", 0.0017, 0.002, [4.0, 16.0, 6.0]),
];

fn profile_name(c: &ControlConfig) -> Option<&'static str> {
    PROFILES
        .iter()
        .find(|(_, kp, kd, sl)| (c.kp - kp).abs() < 1e-9 && (c.kd - kd).abs() < 1e-9 && c.backlash_ticks == *sl)
        .map(|p| p.0)
}

enum Run {
    Idle,
    Running { tx: UnboundedSender<Cmd>, rx: UnboundedReceiver<Event>, paused: bool, started: Instant },
    Finished(Summary),
}

#[derive(Default, Clone)]
struct Stats {
    median: f64,
    p95: f64,
    osc_x: (f64, f64),
    osc_y: (f64, f64),
    lag_ms: [f64; 3],
    track: [f64; 3],
    current: [f64; 3],
    latency: f64,
    fps: f64,
    deadband_pct: f64,
    n: usize,
}

pub struct BalanceScreen {
    run: Run,
    control: ControlConfig,
    sel: usize,
    coarse: bool,
    dirty: bool,
    hist: VecDeque<Snapshot>,
    trail: VecDeque<(f64, f64)>,
    msgs: VecDeque<(Instant, String)>,
    stats: Stats,
    stats_at: Instant,
    last: Option<Snapshot>,
    /// When `t` was pressed (the loop's countdown shows up in the next frame).
    throw_req: Option<Instant>,
    /// Throw aim along the handle axis (mm).
    aim: f64,
    /// Throw release radius around the aim point (mm), for the drawing.
    release_r: f64,
}

const HIST_FRAMES: usize = 29 * 20;

impl BalanceScreen {
    pub fn new(ctx: &Ctx) -> Self {
        Self {
            run: Run::Idle,
            control: ctx.cfg.control.clone(),
            sel: 1,
            coarse: false,
            dirty: false,
            hist: VecDeque::new(),
            trail: VecDeque::new(),
            msgs: VecDeque::new(),
            stats: Stats::default(),
            stats_at: Instant::now(),
            last: None,
            throw_req: None,
            aim: ctx.cfg.throw.aim_y_mm.clamp(-50.0, 50.0),
            release_r: ctx.cfg.throw.centre_mm,
        }
    }

    pub fn running(&self) -> bool {
        matches!(self.run, Run::Running { .. })
    }

    /// A throw countdown is running (any key cancels it).
    pub fn throw_pending(&self) -> bool {
        self.running()
            && (self.throw_req.is_some_and(|t| t.elapsed().as_millis() < 400)
                || self.last.as_ref().is_some_and(|s| s.tele.phase == Phase::Balance && s.throw_in.is_some()))
    }

    pub fn cancel_throw(&mut self) {
        self.throw_req = None;
        if let Run::Running { tx, .. } = &self.run {
            let _ = tx.send(Cmd::CancelThrow);
        }
    }

    fn request_throw(&mut self, ctx: &mut Ctx) {
        let Run::Running { tx, paused, .. } = &self.run else {
            self.msg("throw: start a run first (Space) and put the ball on the paddle");
            return;
        };
        if *paused {
            self.msg("throw: resume first (Space)");
            return;
        }
        if !ctx.armed() {
            self.msg("press a to ARM first: the throw is a fast, large move");
            return;
        }
        if !self.last.as_ref().is_some_and(|s| s.tele.phase == Phase::Balance) {
            self.msg("throw: wait until the ball is balancing");
            return;
        }
        let _ = tx.send(Cmd::Throw);
        self.throw_req = Some(Instant::now());
        ctx.disarm();
    }

    fn msg(&mut self, m: impl Into<String>) {
        self.msgs.push_back((Instant::now(), m.into()));
        while self.msgs.len() > 6 {
            self.msgs.pop_front();
        }
    }

    pub fn stop(&mut self) {
        if let Run::Running { tx, .. } = &self.run {
            let _ = tx.send(Cmd::Stop);
        }
    }

    /// Config for this run: the user's config with the UI's gains. In simulation, missing poses and calibrations
    /// come from the built-in example (the real arm's values).
    fn run_config(&self, ctx: &Ctx) -> Config {
        let mut cfg = ctx.cfg.clone();
        if ctx.sim {
            let ex = Config::example();
            if cfg.poses.level.is_none() {
                cfg.poses = ex.poses.clone();
            }
            if cfg.camera.homography.is_none() || cfg.camera.axes.is_none() {
                cfg.camera.homography = ex.camera.homography;
                cfg.camera.axes = ex.camera.axes;
            }
        }
        cfg.control = self.control.clone();
        cfg.throw.aim_y_mm = self.aim;
        cfg
    }

    fn start(&mut self, ctx: &mut Ctx) {
        if !ctx.armed() {
            self.msg("press a to ARM first: the arm will move to level");
            return;
        }
        let cfg = self.run_config(ctx);
        let (ctx_tx, mut ctx_rx) = unbounded_channel();
        let (ev_tx, ev_rx) = unbounded_channel();
        let sim = ctx.sim;
        let dir = ctx.log_dir.clone();
        tokio::spawn(async move {
            let etx = ev_tx.clone();
            let emit = move |e: Event| {
                let _ = etx.send(e);
            };
            let res = async {
                let bal = Balancer::new(cfg.clone())?;
                if sim {
                    let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_nanos() as u64;
                    let mut p = SimPlant::new(&cfg, SimParams::default(), seed)?;
                    run_loop(&mut p, bal, &dir, &mut ctx_rx, emit).await
                } else {
                    #[cfg(target_os = "linux")]
                    {
                        let _ = ev_tx.send(Event::Status("opening the bus and the camera...".into()));
                        let mut p = crate::balance::RealPlant::open(&cfg).await?;
                        run_loop(&mut p, bal, &dir, &mut ctx_rx, emit).await
                    }
                    #[cfg(not(target_os = "linux"))]
                    {
                        anyhow::bail!("the real arm needs Linux")
                    }
                }
            }
            .await;
            if let Err(e) = res {
                let _ = ev_tx.send(Event::Finished(Summary {
                    longest_s: 0.0,
                    duration_s: 0.0,
                    mean_current_ma: [0.0; 3],
                    log: PathBuf::new(),
                    error: Some(format!("{e:#}")),
                }));
            }
        });
        self.hist.clear();
        self.trail.clear();
        self.last = None;
        self.stats = Stats::default();
        self.run = Run::Running { tx: ctx_tx, rx: ev_rx, paused: false, started: Instant::now() };
        self.msg(if sim { "simulation started: press b to put a ball on the paddle" } else { "starting..." });
        ctx.disarm();
    }

    /// Drain events from the running loop (called every UI tick).
    pub fn pump(&mut self, _ctx: &mut Ctx) {
        let mut events = Vec::new();
        if let Run::Running { rx, .. } = &mut self.run {
            while let Ok(e) = rx.try_recv() {
                events.push(e);
            }
        }
        for e in events {
            match e {
                Event::Status(s) => self.msg(s),
                Event::Frame(f) => {
                    if f.tele.phase == Phase::Balance {
                        if let Some(b) = f.tele.ball {
                            self.trail.push_back((b.x, b.y));
                            while self.trail.len() > 30 {
                                self.trail.pop_front();
                            }
                        }
                    } else {
                        self.trail.clear();
                    }
                    self.hist.push_back((*f).clone());
                    while self.hist.len() > HIST_FRAMES {
                        self.hist.pop_front();
                    }
                    self.last = Some(*f);
                }
                Event::Finished(s) => {
                    match &s.error {
                        Some(e) => self.msg(format!("stopped: {e}")),
                        None => self.msg(format!("finished: longest balance {:.1} s", s.longest_s)),
                    }
                    self.run = Run::Finished(s);
                }
            }
        }
        if self.stats_at.elapsed().as_millis() > 500 {
            self.stats_at = Instant::now();
            self.stats = self.compute_stats();
        }
    }

    fn compute_stats(&self) -> Stats {
        let w: Vec<&Snapshot> = self.hist.iter().rev().take(29 * 10).filter(|s| s.tele.phase == Phase::Balance).collect();
        let mut st = Stats { n: w.len(), ..Default::default() };
        if w.len() < 30 {
            return st;
        }
        let w: Vec<&Snapshot> = w.into_iter().rev().collect();
        let r: Vec<f64> = w.iter().map(|s| s.tele.pos.norm()).collect();
        st.median = median(&r);
        st.p95 = percentile(&r, 95.0);
        let dt_all: Vec<f64> = w.windows(2).map(|p| p[1].tele.t.saturating_duration_since(p[0].tele.t).as_secs_f64()).collect();
        let dt = median(&dt_all).max(1e-3);
        st.fps = 1.0 / dt;
        let xs: Vec<f64> = w.iter().map(|s| s.tele.pos.x).collect();
        let ys: Vec<f64> = w.iter().map(|s| s.tele.pos.y).collect();
        let sm = ((3.0 / dt) as usize).max(3);
        st.osc_x = dominant_freq(&detrend(&xs, sm), dt, 0.2, 3.0);
        st.osc_y = dominant_freq(&detrend(&ys, sm), dt, 0.2, 3.0);
        for k in 0..3 {
            let g: Vec<f64> = w.iter().map(|s| s.goal[k] as f64).collect();
            let p: Vec<f64> = w.iter().map(|s| s.present[k] as f64).collect();
            st.lag_ms[k] = lag_samples(&g, &p, 12) as f64 * dt * 1e3;
            st.track[k] = mean(&g.iter().zip(&p).map(|(a, b)| (a - b).abs()).collect::<Vec<_>>());
            st.current[k] = mean(&w.iter().map(|s| s.current[k]).collect::<Vec<_>>());
        }
        st.latency = median(&w.iter().map(|s| s.latency_ms).collect::<Vec<_>>());
        let db = self.control.deadband_mm;
        let quiet = w.iter().filter(|s| s.tele.pos.norm() < db && s.tele.vel.norm() < self.control.deadband_speed_mm_s).count();
        st.deadband_pct = 100.0 * quiet as f64 / w.len() as f64;
        st
    }

    fn apply_control(&mut self) {
        self.dirty = true;
        if let Run::Running { tx, .. } = &self.run {
            let _ = tx.send(Cmd::SetControl(self.control.clone()));
        }
    }

    pub fn key(&mut self, k: KeyEvent, ctx: &mut Ctx) -> Nav {
        let step = |p: &Param, coarse: bool| if coarse { p.coarse } else { p.fine };
        match k.code {
            KeyCode::Up => self.sel = self.sel.saturating_sub(1),
            KeyCode::Down => self.sel = (self.sel + 1).min(PARAMS.len() - 1),
            KeyCode::Left | KeyCode::Right => {
                let p = PARAMS[self.sel];
                let d = step(&p, self.coarse || k.modifiers.contains(KeyModifiers::SHIFT));
                let v = get(&self.control, self.sel) + if k.code == KeyCode::Right { d } else { -d };
                let r = 10f64.powi(p.digits as i32 + 1);
                set(&mut self.control, self.sel, (v * r).round() / r);
                self.apply_control();
            }
            KeyCode::Char('+') | KeyCode::Char('=') => self.coarse = true,
            KeyCode::Char('-') | KeyCode::Char('_') => self.coarse = false,
            KeyCode::Char(c @ ('0' | '1' | '2')) => {
                let (name, kp, kd, sl) = PROFILES[c as usize - '0' as usize];
                self.control.kp = kp;
                self.control.kd = kd;
                self.control.backlash_ticks = sl;
                self.apply_control();
                self.msg(format!("profile: {name}"));
            }
            KeyCode::Char('s') => {
                let res = Config::load(&ctx.cfg_path).and_then(|mut c| {
                    c.control = self.control.clone();
                    c.throw.aim_y_mm = self.aim;
                    let text = toml::to_string_pretty(&c)?;
                    std::fs::write(&ctx.cfg_path, text)?;
                    Ok(c)
                });
                match res {
                    Ok(c) => {
                        ctx.cfg.control = c.control;
                        ctx.cfg.throw.aim_y_mm = c.throw.aim_y_mm;
                        self.dirty = false;
                        self.msg(format!("saved tuning to {}", ctx.cfg_path.display()));
                    }
                    Err(e) => self.msg(format!("save failed: {e:#}")),
                }
            }
            KeyCode::Char('t') => self.request_throw(ctx),
            KeyCode::Char(c @ ('[' | ']')) => {
                self.aim = (self.aim + if c == ']' { 10.0 } else { -10.0 }).clamp(-50.0, 50.0);
                self.dirty = true;
                if let Run::Running { tx, .. } = &self.run {
                    let _ = tx.send(Cmd::SetAim(self.aim));
                }
                let what = if self.aim < -5.0 { "toward the handle: longer throw" } else if self.aim > 5.0 { "toward the far edge: shorter, higher lob" } else { "centre" };
                self.msg(format!("throw aim {:+.0} mm ({what})", self.aim));
            }
            KeyCode::Char('b') => {
                if let Run::Running { tx, .. } = &self.run {
                    if ctx.sim {
                        let _ = tx.send(Cmd::PlaceBall);
                    }
                }
            }
            KeyCode::Char(' ') => match &mut self.run {
                Run::Running { tx, paused, .. } => {
                    *paused = !*paused;
                    let _ = tx.send(Cmd::Pause(*paused));
                }
                _ => self.start(ctx),
            },
            KeyCode::Enter => {
                if let Run::Finished(s) = &self.run {
                    let p = s.log.clone();
                    return Nav::Results(if p.as_os_str().is_empty() { None } else { Some(p) });
                }
            }
            KeyCode::Esc | KeyCode::Char('q') => match &self.run {
                Run::Running { tx, .. } => {
                    let _ = tx.send(Cmd::Stop);
                    self.msg("stopping: returning to the start pose");
                }
                _ => return Nav::Home,
            },
            _ => {}
        }
        Nav::Stay
    }

    pub fn keys(&self, ctx: &Ctx) -> Vec<(&'static str, &'static str)> {
        let mut v = match &self.run {
            Run::Idle | Run::Finished(_) => {
                let mut v = vec![("Space", "start")];
                if !ctx.sim {
                    v.insert(0, ("a", "arm"));
                }
                if matches!(self.run, Run::Finished(_)) {
                    v.push(("Enter", "results"));
                }
                v.push(("Esc", "home"));
                v
            }
            Run::Running { paused, .. } => {
                if self.throw_pending() {
                    return vec![("any key", "cancel the throw")];
                }
                let mut v = vec![("Space", if *paused { "resume" } else { "pause" }), ("Esc", "stop"), ("t", "throw"), ("[ ]", "aim")];
                if ctx.sim {
                    v.push(("b", "ball"));
                }
                v
            }
        };
        v.extend([("↑↓", "param"), ("←→", "change"), ("0/1/2", "profile"), ("s", "save"), ("?", "help")]);
        v
    }

    // ---------------------------------------------------------------------------------------------------------

    pub fn draw(&self, f: &mut Frame, area: Rect, ctx: &Ctx) {
        let status_h = if area.height >= 40 { 8 } else { 5 };
        let [top, spark, bottom] = Layout::vertical([Constraint::Min(16), Constraint::Length(4), Constraint::Length(status_h)]).areas(area);
        let blade_w = (top.height.saturating_sub(0) * 2 + 2).min(top.width / 2);
        let [blade, right] = Layout::horizontal([Constraint::Length(blade_w), Constraint::Min(30)]).areas(top);
        let [tune, stats] = Layout::vertical([Constraint::Length(PARAMS.len() as u16 + 3), Constraint::Min(0)]).areas(right);
        self.draw_blade(f, blade);
        self.draw_tuning(f, tune);
        self.draw_stats(f, stats, ctx);
        self.draw_spark(f, spark);
        self.draw_status(f, bottom);
    }

    fn draw_blade(&self, f: &mut Frame, area: Rect) {
        let db = self.control.deadband_mm;
        let ball = self.last.as_ref().and_then(|s| s.tele.ball);
        let tilt = self.last.as_ref().map(|s| s.tele.tilt);
        let trail: Vec<(f64, f64)> = self.trail.iter().copied().collect();
        let phase = self.last.as_ref().map(|s| (s.tele.phase, s.tele.paused));
        let throw_in = self.last.as_ref().and_then(|s| s.throw_in).filter(|_| self.running());
        let aim = self.aim;
        let pending = throw_in.is_some();
        let release_r = self.release_r;
        let title = match (&self.run, phase) {
            (Run::Running { .. }, Some((Phase::Balance, _))) if throw_in.is_some() => {
                let t = throw_in.unwrap_or(0.0);
                if t > 0.0 {
                    let total = 2.0f64.max(t);
                    let n = ((t / total) * 10.0).ceil() as usize;
                    format!(" THROW in {t:.1} s {}{} · any key cancels ", "█".repeat(n), "░".repeat(10 - n.min(10)))
                } else {
                    " THROW: waiting for the ball to settle · any key cancels ".to_string()
                }
            }
            (Run::Running { .. }, Some((_, true))) => " blade · paused ".to_string(),
            (Run::Running { .. }, Some((Phase::Balance, _))) => {
                format!(" blade · balancing {:.1} s ", self.last.as_ref().map_or(0.0, |s| s.tele.balance_s))
            }
            (Run::Running { .. }, _) => " blade · waiting for the ball ".to_string(),
            _ => " blade (top view) ".to_string(),
        };
        let canvas = Canvas::default()
            .block(Block::bordered().title(title))
            .marker(Marker::Braille)
            .x_bounds([-85.0, 85.0])
            .y_bounds([-85.0, 85.0])
            .paint(move |c| {
                c.draw(&Circle { x: 0.0, y: 0.0, radius: 75.0, color: Color::Gray });
                c.draw(&Circle { x: 0.0, y: 0.0, radius: 65.0, color: Color::Red });
                c.draw(&Circle { x: 0.0, y: 0.0, radius: db.max(1.0), color: Color::DarkGray });
                c.draw(&CLine { x1: -6.0, y1: 0.0, x2: 6.0, y2: 0.0, color: Color::DarkGray });
                c.draw(&CLine { x1: 0.0, y1: -6.0, x2: 0.0, y2: 6.0, color: Color::DarkGray });
                if let Some(t) = tilt {
                    // Tilt arrow: where the controller pushes the ball (1 deg = 6 mm).
                    let k = 6.0 * 180.0 / std::f64::consts::PI;
                    c.draw(&CLine { x1: 0.0, y1: 0.0, x2: t.x * k, y2: t.y * k, color: Color::Magenta });
                }
                c.layer();
                c.draw(&Points { coords: &trail, color: Color::Yellow });
                if let Some(b) = ball {
                    // Solid orange disc (Ø40 mm ball): concentric rings fill it in Braille.
                    let orange = Color::Indexed(208);
                    let mut r = 20.0;
                    while r > 0.5 {
                        c.draw(&Circle { x: b.x, y: b.y, radius: r, color: orange });
                        r -= 1.2;
                    }
                }
                // Throw aim point (cyan +), and the release circle while a throw is pending.
                let ay = aim;
                c.draw(&CLine { x1: -5.0, y1: ay, x2: 5.0, y2: ay, color: Color::Cyan });
                c.draw(&CLine { x1: 0.0, y1: ay - 5.0, x2: 0.0, y2: ay + 5.0, color: Color::Cyan });
                if pending {
                    c.draw(&Circle { x: 0.0, y: ay, radius: release_r, color: Color::Cyan });
                }
                c.print(-12.0, 80.0, Line::from("far").fg(Color::DarkGray));
                c.print(-18.0, -84.0, Line::from("handle").fg(Color::DarkGray));
            });
        f.render_widget(canvas, area);
    }

    fn draw_tuning(&self, f: &mut Frame, area: Rect) {
        let mut lines = Vec::new();
        for (i, p) in PARAMS.iter().enumerate() {
            let v = get(&self.control, i);
            let sel = i == self.sel;
            let val = format!("{:>9.*}", p.digits, v);
            let style = if sel { Style::new().fg(Color::Black).bg(Color::Cyan).bold() } else { Style::new() };
            lines.push(Line::from(vec![
                Span::raw(if sel { " ▸ " } else { "   " }),
                Span::styled(format!("{:<11}", p.name), style),
                Span::raw(" "),
                Span::styled(val, if sel { Style::new().bold() } else { Style::new() }),
                Span::styled(format!(" {}", p.unit), Style::new().fg(Color::DarkGray)),
                Span::raw(if sel { "   ◂ ▸" } else { "" }),
            ]));
        }
        lines.push(Line::from(vec![
            Span::raw("   "),
            Span::styled(format!("{:<11}", "throw aim"), Style::new()),
            Span::raw(" "),
            Span::styled(format!("{:>+9.0}", self.aim), Style::new().fg(Color::Cyan)),
            Span::styled(" mm  [ ]", Style::new().fg(Color::DarkGray)),
        ]));
        lines.push(Line::from(Span::styled(
            format!(
                "   step {}   profile {}",
                if self.coarse { "coarse (+)" } else { "fine (-)" },
                profile_name(&self.control).unwrap_or("custom")
            ),
            Style::new().fg(Color::DarkGray),
        )));
        let title = if self.dirty { " live tuning · unsaved (s) " } else { " live tuning " };
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(title)), area);
    }

    fn draw_stats(&self, f: &mut Frame, area: Rect, ctx: &Ctx) {
        let s = &self.stats;
        let dim = Style::new().fg(Color::DarkGray);
        let num = |v: f64, d: usize| if v.is_finite() { format!("{v:.d$}") } else { "–".into() };
        let mut lines = Vec::new();
        if let Some(l) = &self.last {
            let t = &l.tele;
            lines.push(Line::from(format!(
                " ball  {}   tilt x {:+.1}° y {:+.1}°   longest {:.1} s",
                t.ball.map_or("not seen".to_string(), |b| format!("({:+.0}, {:+.0}) mm", b.x, b.y)),
                t.tilt.x.to_degrees(),
                t.tilt.y.to_degrees(),
                t.longest_s.max(t.balance_s)
            )));
        } else {
            lines.push(Line::from(Span::styled(
                if ctx.sim { " press Space to start the simulation" } else { " press a, then Space: the arm goes to level" },
                dim,
            )));
        }
        if s.n >= 30 {
            lines.push(Line::from(format!(" offset   median {} mm   p95 {} mm", num(s.median, 1), num(s.p95, 1))));
            lines.push(Line::from(format!(
                " wobble   x {} Hz ±{} mm   y {} Hz ±{} mm",
                num(s.osc_x.0, 2),
                num(s.osc_x.1, 1),
                num(s.osc_y.0, 2),
                num(s.osc_y.1, 1)
            )));
            lines.push(Line::from(format!(
                " current  M5 {} · M6 {} · M7 {} mA",
                num(s.current[0], 0),
                num(s.current[1], 0),
                num(s.current[2], 0)
            )));
            lines.push(Line::from(format!(
                " servo    lag {}/{}/{} ms   miss {}/{}/{} ticks",
                num(s.lag_ms[0], 0),
                num(s.lag_ms[1], 0),
                num(s.lag_ms[2], 0),
                num(s.track[0], 0),
                num(s.track[1], 0),
                num(s.track[2], 0)
            )));
            lines.push(Line::from(format!(
                " loop     {} fps   latency {} ms   deadband {}%",
                num(s.fps, 1),
                num(s.latency, 1),
                num(s.deadband_pct, 0)
            )));
        } else if self.running() {
            lines.push(Line::from(Span::styled(" statistics appear after a few seconds of balancing", dim)));
        }
        if let Run::Finished(sum) = &self.run {
            lines.push(Line::from(""));
            lines.push(Line::from(vec![
                Span::styled(" last run ", Style::new().bold()),
                Span::raw(format!(
                    "longest {:.1} s · {:.0} s · mean mA {:.0}/{:.0}/{:.0}",
                    sum.longest_s, sum.duration_s, sum.mean_current_ma[0], sum.mean_current_ma[1], sum.mean_current_ma[2]
                )),
            ]));
            if !sum.log.as_os_str().is_empty() {
                lines.push(Line::from(Span::styled(format!(" {}  (Enter: results)", sum.log.display()), dim)));
            }
        }
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(" statistics (last 10 s) ")), area);
    }

    fn draw_spark(&self, f: &mut Frame, area: Rect) {
        let n = area.width.saturating_sub(2) as usize;
        let data: Vec<u64> = self
            .hist
            .iter()
            .rev()
            .take(n)
            .map(|s| if s.tele.phase == Phase::Balance { s.tele.pos.norm().min(80.0).round() as u64 } else { 0 })
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect();
        f.render_widget(
            Sparkline::default()
                .block(Block::bordered().title(" distance from centre (mm, last frames; full height = 40 mm) "))
                .data(&data[..])
                .max(40)
                .style(Style::new().fg(Color::Yellow)),
            area,
        );
    }

    fn draw_status(&self, f: &mut Frame, area: Rect) {
        let started = match &self.run {
            Run::Running { started, .. } => Some(*started),
            _ => None,
        };
        let lines: Vec<Line> = self
            .msgs
            .iter()
            .rev()
            .take(area.height.saturating_sub(2) as usize)
            .rev()
            .map(|(t, m)| {
                let ts = started.map_or(String::new(), |s| format!("{:>6.1}s ", t.saturating_duration_since(s).as_secs_f64()));
                Line::from(vec![Span::styled(ts, Style::new().fg(Color::DarkGray)), Span::raw(m.clone())])
            })
            .collect();
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(" status ")), area);
    }
}
