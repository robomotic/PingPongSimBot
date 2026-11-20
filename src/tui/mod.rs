//! Full-screen app: `paddle-balance` with no arguments (or `paddle-balance tui [--sim]`).
//!
//! Same keys on every screen: ↑↓ select · ←→ adjust · +/- step size · Space main action · Enter confirm · s save ·
//! ? help · Esc back / q quit · Ctrl-C emergency stop (the arm returns to its start pose).
//! Keys that move the real arm need the ARMED state (`a`), which switches itself off after 30 s without a key.

mod balance;
mod results;

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
}

const HOME_ITEMS: [(&str, &str, bool); 7] = [
    ("Balance", "run + live tuning", true),
    ("Results", "past runs, compare", true),
    ("Pose", "level / rest / jog joints   (next)", false),
    ("Gripper", "tighten / loosen   (next; for now: paddle-balance grip)", false),
    ("Camera", "live view, colour thresholds   (next)", false),
    ("Calibrate", "camera map → axes → joint check   (next)", false),
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

struct App {
    ctx: Ctx,
    screen: Screen,
    home_sel: usize,
    help: bool,
    quit: bool,
    balance: balance::BalanceScreen,
    results: results::ResultsScreen,
}

pub async fn run(cfg_path: &Path, log_dir: &Path, sim: bool) -> Result<()> {
    let cfg = Config::load(cfg_path)?;
    let ctx = Ctx { cfg_path: cfg_path.to_path_buf(), cfg, sim, log_dir: log_dir.to_path_buf(), armed_until: None };
    let mut app = App {
        balance: balance::BalanceScreen::new(&ctx),
        results: results::ResultsScreen::default(),
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
                KeyCode::Char('q') | KeyCode::Esc => self.quit = true,
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
        }
    }

    fn open(&mut self, i: usize) {
        match i {
            0 => self.screen = Screen::Balance,
            1 => {
                self.results.load(&self.ctx, None);
                self.screen = Screen::Results;
            }
            _ => {}
        }
    }

    fn draw(&mut self, f: &mut Frame) {
        let [head, body, foot] = Layout::vertical([Constraint::Length(1), Constraint::Min(0), Constraint::Length(1)]).areas(f.area());
        let (title, keys): (&str, Vec<(&str, &str)>) = match self.screen {
            Screen::Home => ("HOME", vec![("↑↓", "select"), ("Enter", "open"), ("?", "help"), ("q", "quit")]),
            Screen::Balance => ("BALANCE", self.balance.keys(&self.ctx)),
            Screen::Results => ("RESULTS", self.results.keys()),
        };
        header(f, head, title, &self.ctx, self.balance.running());
        match self.screen {
            Screen::Home => self.draw_home(f, body),
            Screen::Balance => self.balance.draw(f, body, &self.ctx),
            Screen::Results => self.results.draw(f, body),
        }
        footer(f, foot, &keys);
        if self.help {
            let lines = match self.screen {
                Screen::Home => vec![
                    "↑↓ / Enter   choose a screen (1 Balance, 2 Results)",
                    "a            ARM / disarm: needed before anything moves the real arm (30 s timeout)",
                    "Ctrl-C       emergency stop from any screen: the arm returns to its start pose",
                    "q / Esc      quit",
                ],
                Screen::Balance => balance::HELP.to_vec(),
                Screen::Results => results::HELP.to_vec(),
            };
            help_popup(f, &lines);
        }
    }

    fn draw_home(&self, f: &mut Frame, area: Rect) {
        let c = &self.ctx;
        let mut lines = vec![
            Line::from(""),
            Line::from(vec![
                Span::raw("  Mode      "),
                if c.sim { Span::styled("SIMULATION: nothing moves", Style::new().fg(Color::Cyan).bold()) } else { Span::styled("REAL ARM", Style::new().fg(Color::Yellow).bold()) },
            ]),
            Line::from(format!("  Config    {}", c.cfg_path.display())),
            Line::from(format!("  Bus       {}", c.cfg.bus.port)),
            Line::from(format!(
                "  Poses     level {}   rest {}   grip {}",
                if c.cfg.poses.level.is_some() { "✓" } else { "✗" },
                if c.cfg.poses.rest.is_some() { "✓" } else { "✗" },
                c.cfg.poses.grip.map_or("✗".to_string(), |g| g.to_string())
            )),
            Line::from(format!(
                "  Calib     camera {}   axes {}",
                if c.cfg.camera.homography.is_some() { "✓" } else { "✗" },
                if c.cfg.camera.axes.is_some() { "✓" } else { "✗" }
            )),
        ];
        if !c.sim {
            lines.push(Line::from(Span::styled(
                "  Before a real run: sudo systemctl stop normacore-station, and re-attach the camera (see README)",
                Style::new().fg(Color::DarkGray),
            )));
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

fn header(f: &mut Frame, area: Rect, title: &str, c: &Ctx, running: bool) {
    let mut spans = vec![Span::styled(format!(" {title} "), Style::new().fg(Color::Black).bg(Color::Cyan).bold()), Span::raw("  ")];
    if c.sim {
        spans.push(Span::styled(" SIM ", Style::new().fg(Color::Black).bg(Color::Blue).bold()));
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
