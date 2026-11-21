//! Full-screen app: `paddle-balance` with no arguments (or `paddle-balance tui [--sim]`).
//!
//! Same keys on every screen: ↑↓ select · ←→ adjust · +/- step size · Space main action · Enter confirm · s save ·
//! ? help · Esc back / q quit · Ctrl-C emergency stop (the arm returns to its start pose).
//! Keys that move the real arm need the ARMED state (`a`), which switches itself off after 30 s without a key.

mod balance;
mod calibrate;
mod camera;
mod camview;
mod device;
mod pose;
mod results;
mod servos;
mod system;

use crate::config::Config;
use anyhow::Result;
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Flex, Layout, Rect};
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Clear, Paragraph, Wrap};
use ratatui::Frame;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

pub const ARM_TIMEOUT: Duration = Duration::from_secs(30);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Screen {
    Home,
    Balance,
    Results,
    Pose,
    Camera,
    Calibrate,
    Servos,
}

const HOME_ITEMS: [(&str, &str, bool); 7] = [
    ("Balance", "run + live tuning", true),
    ("Results", "past runs, compare", true),
    ("Pose", "joints & gripper: move, go to / save level and rest, set the grip", true),
    ("Camera", "live view, detector view, colour & exposure tuning", true),
    ("Calibrate", "camera map wizard (auto-capture); axes: calib-axes for now", true),
    ("Servos", "P gain per joint: read, change, write to the servos", true),
    ("System ID", "servo identification   (ElRobotSysId)", false),
];

pub struct Ctx {
    pub cfg_path: PathBuf,
    pub cfg: Config,
    pub sim: bool,
    pub log_dir: PathBuf,
    armed_until: Option<Instant>,
}

impl Ctx {
    pub fn armed(&self) -> bool {
        self.sim || self.armed_until.is_some_and(|t| Instant::now() < t)
    }
    fn toggle_arm(&mut self) {
        self.armed_until = if self.armed_until.is_some_and(|t| Instant::now() < t) { None } else { Some(Instant::now() + ARM_TIMEOUT) };
    }
    fn touch(&mut self) {
        if let Some(t) = self.armed_until {
            if Instant::now() < t {
                self.armed_until = Some(Instant::now() + ARM_TIMEOUT);
            }
        }
    }
    pub fn disarm(&mut self) {
        self.armed_until = None;
    }
    /// Directories searched for run logs.
    pub fn log_dirs(&self) -> Vec<PathBuf> {
        vec![self.log_dir.clone(), PathBuf::from("data")]
    }
}

/// A y/n question shown on top of everything.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Confirm {
    StopStation,
    StartStation,
    /// Quitting after this app stopped the station: start it again?
    QuitRestart,
}

impl Confirm {
    fn text(self) -> &'static str {
        match self {
            Confirm::StopStation => "Stop the NormaCore station? Servos keep holding their pose; the camera is re-attached and the bus checked. (y/n)",
            Confirm::StartStation => "Start the NormaCore station? It takes the bus and camera back; leader→follower mirroring may move the arm. (y/n)",
            Confirm::QuitRestart => "This app stopped the station. Start it again before quitting? (y = start and quit, n = just quit, Esc = stay)",
        }
    }
}

#[derive(Default)]
struct SysView {
    station: Option<bool>,
    camera_node: bool,
    camera_usb: Option<PathBuf>,
    bus: Option<Result<system::BusInfo, String>>,
    busy: Option<&'static str>,
    msg: Option<(bool, String)>,
    known: bool,
}

struct App {
    ctx: Ctx,
    screen: Screen,
    home_sel: usize,
    help: bool,
    quit: bool,
    balance: balance::BalanceScreen,
    results: results::ResultsScreen,
    pose: pose::PoseScreen,
    camera: camera::CameraScreen,
    calibrate: calibrate::CalibrateScreen,
    servos: servos::ServosScreen,
    sys: SysView,
    sys_tx: tokio::sync::mpsc::UnboundedSender<system::SysMsg>,
    sys_rx: tokio::sync::mpsc::UnboundedReceiver<system::SysMsg>,
    sys_refreshed: Instant,
    confirm: Option<Confirm>,
    we_stopped_station: bool,
}

pub async fn run(cfg_path: &Path, log_dir: &Path, sim: bool) -> Result<()> {
    let cfg = Config::load(cfg_path)?;
    let ctx = Ctx { cfg_path: cfg_path.to_path_buf(), cfg, sim, log_dir: log_dir.to_path_buf(), armed_until: None };
    let (sys_tx, sys_rx) = tokio::sync::mpsc::unbounded_channel();
    let mut app = App {
        sys: SysView::default(),
        sys_tx,
        sys_rx,
        sys_refreshed: Instant::now() - Duration::from_secs(60),
        confirm: None,
        we_stopped_station: false,
        balance: balance::BalanceScreen::new(&ctx),
        results: results::ResultsScreen::default(),
        pose: pose::PoseScreen::default(),
        camera: camera::CameraScreen::default(),
        calibrate: calibrate::CalibrateScreen::default(),
        servos: servos::ServosScreen::default(),
        ctx,
        screen: Screen::Home,
        home_sel: 0,
        help: false,
        quit: false,
    };
    let mut terminal = ratatui::init();
    let res = async {
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        loop {
            tick.tick().await;
            app.balance.pump(&mut app.ctx);
            app.pump_sys();
            app.pose.pump(&app.ctx);
            app.camera.pump();
            app.calibrate.pump();
            app.servos.pump();
            while event::poll(Duration::ZERO)? {
                if let Event::Key(k) = event::read()? {
                    if k.kind == KeyEventKind::Press {
                        app.key(k);
                    }
                }
            }
            terminal.draw(|f| app.draw(f))?;
            if app.quit && !app.balance.running() {
                return Ok::<(), anyhow::Error>(());
            }
        }
    }
    .await;
    ratatui::restore();
    res
}

impl App {
    fn key(&mut self, k: KeyEvent) {
        self.ctx.touch();
        // Emergency stop and quit, from anywhere.
        if k.code == KeyCode::Char('c') && k.modifiers.contains(KeyModifiers::CONTROL) {
            self.balance.stop();
            self.quit = true;
            return;
        }
        if self.help {
            self.help = false;
            return;
        }
        if let Some(c) = self.confirm {
            match (c, k.code) {
                (Confirm::StopStation, KeyCode::Char('y')) => {
                    self.we_stopped_station = true;
                    self.sys_action(system::SysAction::StopStation, "stopping the station...");
                }
                (Confirm::StartStation, KeyCode::Char('y')) => {
                    self.we_stopped_station = false;
                    self.sys_action(system::SysAction::StartStation, "starting the station...");
                }
                (Confirm::QuitRestart, KeyCode::Char('y')) => {
                    let _ = system::spawn(system::SysAction::StartStation, self.ctx.cfg.clone(), self.sys_tx.clone());
                    self.quit = true;
                }
                (Confirm::QuitRestart, KeyCode::Char('n')) => self.quit = true,
                _ => {}
            }
            self.confirm = None;
            return;
        }
        // A pending throw is cancelled by any key, before anything else sees it.
        if self.screen == Screen::Balance && self.balance.throw_pending() {
            self.balance.cancel_throw();
            return;
        }
        match k.code {
            KeyCode::Char('?') => {
                self.help = true;
                return;
            }
            KeyCode::Char('a') if !self.ctx.sim => {
                self.ctx.toggle_arm();
                return;
            }
            _ => {}
        }
        match self.screen {
            Screen::Home => match k.code {
                KeyCode::Up => self.home_sel = self.home_sel.saturating_sub(1),
                KeyCode::Down => self.home_sel = (self.home_sel + 1).min(HOME_ITEMS.len() - 1),
                KeyCode::Enter | KeyCode::Char(' ') => self.open(self.home_sel),
                KeyCode::Char('1') => self.open(0),
                KeyCode::Char('2') => self.open(1),
                KeyCode::Char('3') => self.open(2),
                KeyCode::Char('4') => self.open(3),
                KeyCode::Char('5') => self.open(4),
                KeyCode::Char('6') => self.open(5),
                KeyCode::Char('x') => self.ask_station(),
                KeyCode::Char('c') => self.sys_action(system::SysAction::ReattachCamera, "re-attaching the camera..."),
                KeyCode::Char('r') => self.sys_action(system::SysAction::CheckBus, "checking the bus..."),
                KeyCode::Char('q') | KeyCode::Esc => {
                    if self.we_stopped_station && self.sys.station == Some(false) && !self.ctx.sim {
                        self.confirm = Some(Confirm::QuitRestart);
                    } else {
                        self.quit = true;
                    }
                }
                _ => {}
            },
            Screen::Balance => match self.balance.key(k, &mut self.ctx) {
                balance::Nav::Stay => {}
                balance::Nav::Home => self.screen = Screen::Home,
                balance::Nav::Results(p) => {
                    self.results.load(&self.ctx, p);
                    self.screen = Screen::Results;
                }
            },
            Screen::Results => {
                if self.results.key(k) {
                    self.screen = Screen::Home;
                }
            }
            Screen::Pose => {
                if self.pose.key(k, &mut self.ctx) {
                    self.screen = Screen::Home;
                }
            }
            Screen::Camera => {
                if self.camera.key(k, &mut self.ctx) {
                    self.screen = Screen::Home;
                }
            }
            Screen::Servos => {
                if self.servos.key(k, &mut self.ctx) {
                    self.screen = Screen::Home;
                }
            }
            Screen::Calibrate => {
                if self.calibrate.key(k, &mut self.ctx) {
                    self.screen = Screen::Home;
                }
            }
        }
    }

    fn sys_blocked(&mut self) -> bool {
        let why = if self.ctx.sim {
            Some("simulation: station and camera controls are off")
        } else if self.balance.running() {
            Some("stop the balance run first")
        } else if self.sys.busy.is_some() {
            Some("busy, wait a moment")
        } else {
            None
        };
        if let Some(w) = why {
            self.sys.msg = Some((false, w.into()));
        }
        why.is_some()
    }

    fn sys_action(&mut self, a: system::SysAction, label: &'static str) {
        if self.sys_blocked() {
            return;
        }
        if a == system::SysAction::CheckBus && self.sys.station == Some(true) {
            self.sys.msg = Some((false, "the station holds the bus: stop it first (x)".into()));
            return;
        }
        self.sys.busy = Some(label);
        system::spawn(a, self.ctx.cfg.clone(), self.sys_tx.clone());
    }

    fn ask_station(&mut self) {
        if self.sys_blocked() {
            return;
        }
        self.confirm = match self.sys.station {
            Some(true) => Some(Confirm::StopStation),
            Some(false) => Some(Confirm::StartStation),
            None => {
                self.sys.msg = Some((false, format!("no systemd unit '{}' here", self.ctx.cfg.system.station_service)));
                None
            }
        };
    }

    fn pump_sys(&mut self) {
        if self.sys_refreshed.elapsed() > Duration::from_secs(3) && self.sys.busy.is_none() {
            self.sys_refreshed = Instant::now();
            system::spawn(system::SysAction::Refresh, self.ctx.cfg.clone(), self.sys_tx.clone());
        }
        while let Ok(m) = self.sys_rx.try_recv() {
            match m {
                system::SysMsg::Status { station, camera_node, camera_usb } => {
                    self.sys.station = station;
                    self.sys.camera_node = camera_node;
                    self.sys.camera_usb = camera_usb;
                    self.sys.known = true;
                    self.sys.busy = None;
                }
                system::SysMsg::Bus(b) => self.sys.bus = Some(b),
                system::SysMsg::Done(m) => self.sys.msg = Some((true, m)),
                system::SysMsg::Failed(m) => self.sys.msg = Some((false, m)),
            }
        }
    }

    fn open(&mut self, i: usize) {
        match i {
            0 => self.screen = Screen::Balance,
            1 => {
                self.results.load(&self.ctx, None);
                self.screen = Screen::Results;
            }
            2 => {
                if self.balance.running() {
                    self.sys.msg = Some((false, "stop the balance run first: it owns the bus".into()));
                } else {
                    self.pose.enter(&self.ctx);
                    self.screen = Screen::Pose;
                }
            }
            5 => {
                if self.balance.running() {
                    self.sys.msg = Some((false, "stop the balance run first: it owns the bus".into()));
                } else {
                    self.servos.enter(&self.ctx);
                    self.screen = Screen::Servos;
                }
            }
            3 | 4 => {
                if self.balance.running() && !self.ctx.sim {
                    self.sys.msg = Some((false, "stop the balance run first: it owns the camera".into()));
                } else if i == 3 {
                    self.camera.enter(&self.ctx);
                    self.screen = Screen::Camera;
                } else {
                    self.calibrate.enter(&self.ctx);
                    self.screen = Screen::Calibrate;
                }
            }
            _ => {}
        }
    }

    fn draw(&mut self, f: &mut Frame) {
        let [head, body, foot] = Layout::vertical([Constraint::Length(1), Constraint::Min(0), Constraint::Length(1)]).areas(f.area());
        let (title, keys): (&str, Vec<(&str, &str)>) = match self.screen {
            Screen::Home => (
                "HOME",
                if self.ctx.sim {
                    vec![("↑↓", "select"), ("Enter", "open"), ("?", "help"), ("q", "quit")]
                } else {
                    vec![("↑↓", "select"), ("Enter", "open"), ("x", "station"), ("c", "camera"), ("r", "bus"), ("a", "arm"), ("?", "help"), ("q", "quit")]
                },
            ),
            Screen::Balance => ("BALANCE", self.balance.keys(&self.ctx)),
            Screen::Results => ("RESULTS", self.results.keys()),
            Screen::Pose => ("POSE", self.pose.keys(&self.ctx)),
            Screen::Camera => ("CAMERA", self.camera.keys(&self.ctx)),
            Screen::Calibrate => ("CALIBRATE", self.calibrate.keys()),
            Screen::Servos => ("SERVOS", self.servos.keys(&self.ctx)),
        };
        header(f, head, title, &self.ctx, self.balance.running());
        match self.screen {
            Screen::Home => self.draw_home(f, body),
            Screen::Balance => self.balance.draw(f, body, &self.ctx),
            Screen::Results => self.results.draw(f, body),
            Screen::Pose => self.pose.draw(f, body, &self.ctx),
            Screen::Camera => self.camera.draw(f, body, &self.ctx),
            Screen::Calibrate => self.calibrate.draw(f, body),
            Screen::Servos => self.servos.draw(f, body, &self.ctx),
        }
        footer(f, foot, &keys);
        if let Some(c) = self.confirm {
            confirm_popup(f, c.text());
        }
        if self.help {
            let lines = match self.screen {
                Screen::Home => vec![
                    "↑↓ / Enter   choose a screen (1 Balance, 2 Results, 3 Pose, 4 Camera, 5 Calibrate, 6 Servos)",
                    "x            stop / start the NormaCore station (asks first). Stopping re-attaches the camera and checks the bus",
                    "c            re-attach the camera to the kernel (after the station used it)",
                    "r            check the bus: which servos answer, voltage, temperature, gripper load (read-only, nothing moves)",
                    "a            ARM / disarm: needed before anything moves the real arm (30 s timeout)",
                    "Ctrl-C       emergency stop from any screen: the arm returns to its start pose",
                    "q / Esc      quit",
                ],
                Screen::Balance => balance::HELP.to_vec(),
                Screen::Results => results::HELP.to_vec(),
                Screen::Pose => pose::HELP.to_vec(),
                Screen::Camera => camera::HELP.to_vec(),
                Screen::Calibrate => calibrate::HELP.to_vec(),
                Screen::Servos => servos::HELP.to_vec(),
            };
            help_popup(f, &lines);
        }
    }

    fn draw_home(&self, f: &mut Frame, area: Rect) {
        let c = &self.ctx;
        let ok = |b: bool| if b { Span::styled("✓", Style::new().fg(Color::Green).bold()) } else { Span::styled("✗", Style::new().fg(Color::Red).bold()) };
        let dim = Style::new().fg(Color::DarkGray);
        let sv = &self.sys;
        let mut lines = vec![
            Line::from(""),
            Line::from(vec![
                Span::raw("  Mode      "),
                if c.sim { Span::styled("SIMULATION: nothing moves", Style::new().fg(Color::Cyan).bold()) } else { Span::styled("REAL ARM", Style::new().fg(Color::Yellow).bold()) },
                Span::styled(format!("    config {}", c.cfg_path.display()), dim),
            ]),
        ];
        // Station
        let station = match (sv.known, sv.station) {
            (false, _) => Span::styled("checking...", dim),
            (true, Some(true)) => Span::styled("● running: it owns the bus and the camera", Style::new().fg(Color::Yellow)),
            (true, Some(false)) => Span::styled("○ stopped: this app can use the bus and the camera", Style::new().fg(Color::Green)),
            (true, None) => Span::styled(format!("– no '{}' service on this machine", c.cfg.system.station_service), dim),
        };
        lines.push(Line::from(vec![Span::raw("  Station   "), station]));
        // Camera
        let cam = if !sv.known {
            vec![Span::styled("checking...", dim)]
        } else if sv.camera_node {
            vec![ok(true), Span::raw(format!(" {}", c.cfg.camera.device))]
        } else if sv.camera_usb.is_some() {
            vec![ok(false), Span::raw(" on USB but detached from the kernel (station / libuvc) — press c")]
        } else {
            vec![ok(false), Span::styled(" not found", dim)]
        };
        lines.push(Line::from([vec![Span::raw("  Camera    ")], cam].concat()));
        // Bus
        let bus = match &sv.bus {
            None => vec![Span::styled(format!("{}  (r to check, read-only)", c.cfg.bus.port), dim)],
            Some(Ok(b)) => vec![
                ok(b.answering.len() == 8),
                Span::raw(format!(
                    " {}/8 servos · {:.1} V · max {} °C · gripper {} load {}",
                    b.answering.len(),
                    b.voltage,
                    b.max_temp_c,
                    if b.m8_torque { "on," } else { "OFF," },
                    b.m8_load
                )),
            ],
            Some(Err(e)) => vec![ok(false), Span::raw(format!(" {e}"))],
        };
        lines.push(Line::from([vec![Span::raw("  Bus       ")], bus].concat()));
        lines.push(Line::from(vec![
            Span::raw("  Poses     "),
            Span::raw("level "),
            ok(c.cfg.poses.level.is_some()),
            Span::raw("  rest "),
            ok(c.cfg.poses.rest.is_some()),
            Span::raw(format!("  grip {}", c.cfg.poses.grip.map_or("–".to_string(), |g| g.to_string()))),
            Span::raw("    calibration: camera "),
            ok(c.cfg.camera.homography.is_some()),
            Span::raw("  axes "),
            ok(c.cfg.camera.axes.is_some()),
        ]));
        if let Some(b) = sv.busy {
            lines.push(Line::from(Span::styled(format!("  ⏳ {b}"), Style::new().fg(Color::Cyan))));
        } else if let Some((good, m)) = &sv.msg {
            lines.push(Line::from(Span::styled(format!("  {m}"), Style::new().fg(if *good { Color::Green } else { Color::Red }))));
        } else {
            lines.push(Line::from(""));
        }
        lines.push(Line::from(""));
        for (i, (name, desc, ok)) in HOME_ITEMS.iter().enumerate() {
            let sel = i == self.home_sel;
            let style = match (sel, ok) {
                (true, true) => Style::new().fg(Color::Black).bg(Color::Cyan).bold(),
                (true, false) => Style::new().fg(Color::Black).bg(Color::DarkGray),
                (false, true) => Style::new().bold(),
                (false, false) => Style::new().fg(Color::DarkGray),
            };
            lines.push(Line::from(vec![
                Span::raw(if sel { "  ▸ " } else { "    " }),
                Span::styled(format!(" {name:<10} "), style),
                Span::styled(format!("  {desc}"), if *ok { Style::new() } else { Style::new().fg(Color::DarkGray) }),
            ]));
        }
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(" paddle-balance ")), area);
    }
}

fn confirm_popup(f: &mut Frame, text: &str) {
    let w = (text.chars().count() as u16 + 6).min(f.area().width.saturating_sub(4)).max(30);
    let [area] = Layout::horizontal([Constraint::Length(w)]).flex(Flex::Center).areas(f.area());
    let [area] = Layout::vertical([Constraint::Length(5)]).flex(Flex::Center).areas(area);
    f.render_widget(Clear, area);
    f.render_widget(
        Paragraph::new(vec![Line::from(""), Line::from(format!(" {text}"))])
            .wrap(Wrap { trim: false })
            .block(Block::bordered().title(" confirm ").border_style(Style::new().fg(Color::Yellow))),
        area,
    );
}

fn header(f: &mut Frame, area: Rect, title: &str, c: &Ctx, running: bool) {
    let mut spans = vec![Span::styled(format!(" {title} "), Style::new().fg(Color::Black).bg(Color::Cyan).bold()), Span::raw("  ")];
    if c.sim {
        spans.push(Span::styled(" SIM ", Style::new().fg(Color::White).bg(Color::Blue).bold()));
    } else if c.armed() {
        let left = c.armed_until.map_or(0, |t| t.saturating_duration_since(Instant::now()).as_secs());
        spans.push(Span::styled(format!(" ⚠ ARMED {left:>2}s "), Style::new().fg(Color::White).bg(Color::Red).bold()));
    } else {
        spans.push(Span::styled(" disarmed (a to arm) ", Style::new().fg(Color::DarkGray)));
    }
    if running {
        spans.push(Span::raw("  "));
        spans.push(Span::styled(" ● RUNNING ", Style::new().fg(Color::Black).bg(Color::Green).bold()));
    }
    f.render_widget(Paragraph::new(Line::from(spans)), area);
}

fn footer(f: &mut Frame, area: Rect, keys: &[(&str, &str)]) {
    let mut spans = Vec::new();
    for (k, d) in keys {
        spans.push(Span::styled(format!(" {k} "), Style::new().fg(Color::Black).bg(Color::Gray)));
        spans.push(Span::raw(format!(" {d}  ")));
    }
    f.render_widget(Paragraph::new(Line::from(spans)), area);
}

fn help_popup(f: &mut Frame, lines: &[&str]) {
    let w = lines.iter().map(|l| l.chars().count()).max().unwrap_or(20) as u16 + 6;
    let h = lines.len() as u16 + 4;
    let [area] = Layout::horizontal([Constraint::Length(w)]).flex(Flex::Center).areas(f.area());
    let [area] = Layout::vertical([Constraint::Length(h)]).flex(Flex::Center).areas(area);
    f.render_widget(Clear, area);
    let text: Vec<Line> = lines.iter().map(|l| Line::from(format!(" {l}"))).collect();
    f.render_widget(
        Paragraph::new(text)
            .wrap(Wrap { trim: false })
            .block(Block::bordered().title(" keys (any key closes) ").border_style(Style::new().fg(Color::Cyan)).add_modifier(Modifier::BOLD)),
        area,
    );
}
