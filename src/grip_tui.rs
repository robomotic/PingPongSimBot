//! Full-screen gripper (M8) adjuster: arrow keys move the jaws, live load bar, save with 's'.

use crate::bus::Bus;
use crate::config::Config;
use anyhow::Result;
use crossterm::{
    cursor,
    event::{self, Event, KeyCode, KeyEventKind, KeyModifiers},
    execute, queue,
    style::{Color, Print, ResetColor, SetForegroundColor, Stylize},
    terminal::{self, ClearType},
};
use st3215::protocol::RamRegister;
use std::io::{stdout, Write};
use std::path::Path;
use std::time::{Duration, Instant};

const STEPS: [i32; 5] = [1, 2, 5, 10, 20];
/// Max distance from where M8 was when the tool started.
const MAX_TRAVEL: i32 = 300;
/// |load| (0..1000 = 0..100% torque) at which the tool backs off by itself.
const LOAD_AUTO_BACKOFF: i32 = 600;
const LOAD_WARN: i32 = 300;

/// Restores the terminal even on error or panic-unwind.
struct RawGuard;
impl RawGuard {
    fn new() -> Result<Self> {
        terminal::enable_raw_mode()?;
        execute!(stdout(), terminal::EnterAlternateScreen, cursor::Hide)?;
        Ok(Self)
    }
}
impl Drop for RawGuard {
    fn drop(&mut self) {
        let _ = execute!(stdout(), cursor::Show, terminal::LeaveAlternateScreen);
        let _ = terminal::disable_raw_mode();
    }
}

struct View {
    start: u16,
    goal: u16,
    step_i: usize,
    saved: Option<u16>,
    position: u16,
    load: i16,
    current_ma: f64,
    temp: u8,
    torque: bool,
    peak_load: i32,
    msg: String,
    msg_color: Color,
}

fn bar(value: i32, max: i32, width: usize) -> String {
    let filled = ((value.clamp(0, max) as f64 / max as f64) * width as f64).round() as usize;
    format!("{}{}", "█".repeat(filled), "·".repeat(width - filled))
}

fn draw(v: &View) -> Result<()> {
    let mut out = stdout();
    let load = (v.load as i32).abs();
    let load_color = if load >= LOAD_WARN {
        Color::Red
    } else if load >= LOAD_WARN / 2 {
        Color::Yellow
    } else {
        Color::Green
    };
    let squeeze = v.goal as i32 - v.position as i32;
    queue!(out, cursor::MoveTo(0, 0), terminal::Clear(ClearType::All))?;
    let lines: Vec<(String, Option<Color>)> = vec![
        ("  GRIPPER M8 · paddle grip".bold().to_string(), None),
        (String::new(), None),
        (format!("  ← / →   move jaws by {} tick(s)        ↑ / ↓   step size", STEPS[v.step_i]), None),
        ("  s       save grip                     q / Esc  quit (torque stays on)".to_string(), None),
        (String::new(), None),
        (
            format!(
                "  step      {}",
                STEPS.iter().enumerate().map(|(i, s)| if i == v.step_i { format!("[{s}]") } else { format!(" {s} ") }).collect::<String>()
            ),
            None,
        ),
        (format!("  goal      {:>4}     start {}   moved {:+}", v.goal, v.start, v.goal as i32 - v.start as i32), None),
        (format!("  present   {:>4}     goal - present = {:+}  (stays non-zero once the jaws press on the handle)", v.position, squeeze), None),
        (String::new(), None),
        (format!("  load      {} {:>4} / 1000   peak {}", bar(load, 1000, 40), load, v.peak_load), Some(load_color)),
        (format!("  current   {:>4.0} mA      temp {} °C", v.current_ma, v.temp), None),
        (
            format!("  torque    {}", if v.torque { "ON" } else { "OFF  ← overload protection? press ← or → to re-enable" }),
            Some(if v.torque { Color::Green } else { Color::Red }),
        ),
        (format!("  saved     {}", v.saved.map_or("—".to_string(), |g| g.to_string())), None),
        (String::new(), None),
        ("  Press → a few times and watch the jaws. If they open, use ← to close.".to_string(), Some(Color::DarkGrey)),
        ("  Tighten until the blade no longer wiggles in the jaws; keep load under ~300 (yellow, not red).".to_string(), Some(Color::DarkGrey)),
        (String::new(), None),
        (format!("  {}", v.msg), Some(v.msg_color)),
    ];
    for (i, (text, color)) in lines.into_iter().enumerate() {
        queue!(out, cursor::MoveTo(0, i as u16))?;
        if let Some(c) = color {
            queue!(out, SetForegroundColor(c), Print(text), ResetColor)?;
        } else {
            queue!(out, Print(text))?;
        }
    }
    out.flush()?;
    Ok(())
}

pub async fn run(cfg_path: &Path) -> Result<()> {
    let mut cfg = Config::load(cfg_path)?;
    let mut bus = Bus::open(&cfg.bus.port, cfg.bus.timeout_ms)?;
    // Torque on where M8 is now: stiffens the jaws without moving them.
    let start = bus.read_state(8).await?.position;
    bus.sync_write_goals(&[(8, start)]).await?;
    bus.sync_write_u8(RamRegister::TorqueEnable, &[(8, 1)]).await?;

    let mut v = View {
        start,
        goal: start,
        step_i: 2,
        saved: cfg.poses.grip,
        position: start,
        load: 0,
        current_ma: 0.0,
        temp: 0,
        torque: true,
        peak_load: 0,
        msg: "M8 torque ON at its present position. Nothing moved.".into(),
        msg_color: Color::Cyan,
    };
    let _guard = RawGuard::new()?;
    let mut last_read = Instant::now() - Duration::from_secs(1);

    loop {
        // Keys (non-blocking, 50 ms).
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(k) = event::read()? {
                if k.kind != KeyEventKind::Press {
                    continue;
                }
                let mut delta = 0;
                match k.code {
                    KeyCode::Char('q') | KeyCode::Esc => break,
                    KeyCode::Char('c') if k.modifiers.contains(KeyModifiers::CONTROL) => break,
                    KeyCode::Right => delta = STEPS[v.step_i],
                    KeyCode::Left => delta = -STEPS[v.step_i],
                    KeyCode::Up => v.step_i = (v.step_i + 1).min(STEPS.len() - 1),
                    KeyCode::Down => v.step_i = v.step_i.saturating_sub(1),
                    KeyCode::Char('s') => {
                        cfg.poses.grip = Some(v.goal);
                        // Saving writes the file; keep the log line out of the TUI.
                        let text = toml::to_string_pretty(&cfg)?;
                        std::fs::write(cfg_path, text)?;
                        v.saved = Some(v.goal);
                        v.msg = format!("saved grip = {} to {}", v.goal, cfg_path.display());
                        v.msg_color = Color::Green;
                    }
                    _ => {}
                }
                if delta != 0 {
                    let next = v.goal as i32 + delta;
                    if (next - start as i32).abs() > MAX_TRAVEL {
                        v.msg = format!("limit: at most {MAX_TRAVEL} ticks from the start position ({start})");
                        v.msg_color = Color::Red;
                    } else {
                        v.goal = next.clamp(0, 4095) as u16;
                        bus.sync_write_goals(&[(8, v.goal)]).await?;
                        if !v.torque {
                            bus.sync_write_u8(RamRegister::TorqueEnable, &[(8, 1)]).await?;
                        }
                        v.msg = format!("goal {}", v.goal);
                        v.msg_color = Color::Cyan;
                    }
                }
            }
        }
        // Live state at ~10 Hz.
        if last_read.elapsed() >= Duration::from_millis(100) {
            last_read = Instant::now();
            if let Ok(s) = bus.read_state(8).await {
                v.position = s.position;
                v.load = s.load;
                v.current_ma = s.current_ma;
                v.temp = s.temperature_c;
                v.peak_load = v.peak_load.max((s.load as i32).abs());
            }
            if let Ok(t) = bus.read_torque_enabled(8).await {
                v.torque = t;
            }
            // Self-protection: squeezing too hard -> back off towards the start.
            if (v.load as i32).abs() >= LOAD_AUTO_BACKOFF && v.goal != start {
                let back = if v.goal > start { -10 } else { 10 };
                v.goal = (v.goal as i32 + back).clamp(0, 4095) as u16;
                bus.sync_write_goals(&[(8, v.goal)]).await?;
                v.msg = format!("load {} too high: backed off 10 ticks to {}", v.load, v.goal);
                v.msg_color = Color::Red;
            }
            draw(&v)?;
        }
    }
    drop(_guard);
    println!(
        "M8 goal {} (start {start}), torque {}. Saved grip: {}",
        v.goal,
        if v.torque { "on" } else { "OFF" },
        v.saved.map_or("none".into(), |g| g.to_string())
    );
    Ok(())
}
