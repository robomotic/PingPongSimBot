//! Results screen: past runs (paddle_logs/ and data/), their statistics, and a side-by-side comparison.

use super::Ctx;
use crate::analysis::{analyze, list_runs, RunStats};
use crossterm::event::{KeyCode, KeyEvent};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Cell, Paragraph, Row, Table};
use ratatui::Frame;
use std::path::PathBuf;

pub const HELP: [&str; 8] = [
    "↑ ↓        choose a run (newest first; paddle_logs/ and data/)",
    "c          make the selected run the reference (compare against it); again to clear",
    "r          reload the list",
    "Esc / q    back to Home",
    "",
    "Statistics are over each run's longest balance segment (same as tools/analyze_run.py).",
    "Δ is selected minus reference; green = better (less offset, wobble, lag, current), red = worse.",
    "Runs logged before 2026-11-20 have no gain columns, so kp/kd/slop show –.",
];

#[derive(Default)]
pub struct ResultsScreen {
    runs: Vec<(PathBuf, Result<RunStats, String>)>,
    sel: usize,
    reference: Option<usize>,
}

impl ResultsScreen {
    /// Rescan the log folders; select `focus` if given.
    pub fn load(&mut self, ctx: &Ctx, focus: Option<PathBuf>) {
        let keep_ref = self.reference.and_then(|i| self.runs.get(i)).map(|r| r.0.clone());
        self.runs = list_runs(&ctx.log_dirs()).into_iter().map(|p| (p.clone(), analyze(&p).map_err(|e| format!("{e:#}")))).collect();
        self.sel = focus.and_then(|f| self.runs.iter().position(|r| r.0 == f)).unwrap_or(0);
        self.reference = keep_ref.and_then(|p| self.runs.iter().position(|r| r.0 == p));
    }

    /// Returns true to go back Home.
    pub fn key(&mut self, k: KeyEvent) -> bool {
        match k.code {
            KeyCode::Up => self.sel = self.sel.saturating_sub(1),
            KeyCode::Down => self.sel = (self.sel + 1).min(self.runs.len().saturating_sub(1)),
            KeyCode::Char('c') => self.reference = if self.reference == Some(self.sel) { None } else { Some(self.sel) },
            KeyCode::Char('r') => {
                let p = self.runs.get(self.sel).map(|r| r.0.clone());
                let dirs: Vec<PathBuf> = self.runs.iter().filter_map(|r| r.0.parent().map(|d| d.to_path_buf())).collect();
                let keep_ref = self.reference.and_then(|i| self.runs.get(i)).map(|r| r.0.clone());
                let mut d = dirs;
                d.sort();
                d.dedup();
                self.runs = list_runs(&d).into_iter().map(|p| (p.clone(), analyze(&p).map_err(|e| format!("{e:#}")))).collect();
                self.sel = p.and_then(|p| self.runs.iter().position(|r| r.0 == p)).unwrap_or(0);
                self.reference = keep_ref.and_then(|p| self.runs.iter().position(|r| r.0 == p));
            }
            KeyCode::Esc | KeyCode::Char('q') => return true,
            _ => {}
        }
        false
    }

    pub fn keys(&self) -> Vec<(&'static str, &'static str)> {
        vec![("↑↓", "run"), ("c", "compare"), ("r", "reload"), ("Esc", "home"), ("?", "help")]
    }

    pub fn draw(&self, f: &mut Frame, area: Rect) {
        let [list, detail] = Layout::horizontal([Constraint::Length(46), Constraint::Min(40)]).areas(area);
        self.draw_list(f, list);
        self.draw_detail(f, detail);
    }

    fn draw_list(&self, f: &mut Frame, area: Rect) {
        let rows: Vec<Row> = self
            .runs
            .iter()
            .enumerate()
            .map(|(i, (p, r))| {
                let name = p.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
                let mark = if Some(i) == self.reference { "◆" } else { " " };
                let (a, b) = match r {
                    Ok(s) => (format!("{:>6.1}", s.longest_s), if s.median_mm.is_finite() { format!("{:>5.1}", s.median_mm) } else { "    –".into() }),
                    Err(_) => ("   err".into(), "".into()),
                };
                let style = if i == self.sel { Style::new().fg(Color::Black).bg(Color::Cyan).bold() } else { Style::new() };
                Row::new(vec![Cell::from(mark), Cell::from(name), Cell::from(a), Cell::from(b)]).style(style)
            })
            .collect();
        let empty = self.runs.is_empty();
        let table = Table::new(rows, [Constraint::Length(1), Constraint::Min(18), Constraint::Length(7), Constraint::Length(6)])
            .header(Row::new(vec!["", "run", "best s", "med mm"]).style(Style::new().fg(Color::DarkGray)))
            .block(Block::bordered().title(if empty { " runs: none yet " } else { " runs (◆ = reference) " }));
        f.render_widget(table, area);
    }

    fn draw_detail(&self, f: &mut Frame, area: Rect) {
        let Some((path, r)) = self.runs.get(self.sel) else {
            f.render_widget(
                Paragraph::new(" No run logs found in paddle_logs/ or data/. Run the Balance screen first.")
                    .block(Block::bordered().title(" details ")),
                area,
            );
            return;
        };
        let s = match r {
            Ok(s) => s,
            Err(e) => {
                f.render_widget(Paragraph::new(format!(" {}\n {e}", path.display())).block(Block::bordered().title(" details ")), area);
                return;
            }
        };
        let rf = self.reference.filter(|&i| i != self.sel).and_then(|i| self.runs.get(i)).and_then(|r| r.1.as_ref().ok());

        // (label, value getter, lower is better?, digits)
        type Get = fn(&RunStats) -> Option<f64>;
        let metrics: Vec<(&str, Get, Option<bool>, usize)> = vec![
            ("longest balance (s)", |s| Some(s.longest_s), Some(false), 1),
            ("balance segments", |s| Some(s.segments_s.len() as f64), None, 0),
            ("run length (s)", |s| Some(s.duration_s), None, 0),
            ("offset median (mm)", |s| Some(s.median_mm), Some(true), 1),
            ("offset p95 (mm)", |s| Some(s.p95_mm), Some(true), 1),
            ("time < 10 mm (%)", |s| Some(s.under10_pct), Some(false), 0),
            ("wobble x (Hz)", |s| Some(s.osc_x.0), None, 2),
            ("wobble x (± mm)", |s| Some(s.osc_x.1), Some(true), 1),
            ("wobble y (Hz)", |s| Some(s.osc_y.0), None, 2),
            ("wobble y (± mm)", |s| Some(s.osc_y.1), Some(true), 1),
            ("tilt std x (deg)", |s| Some(s.tilt_std_deg.0), None, 2),
            ("tilt std y (deg)", |s| Some(s.tilt_std_deg.1), None, 2),
            ("servo lag M5 (ms)", |s| Some(s.lag_ms[0]), Some(true), 0),
            ("servo lag M6 (ms)", |s| Some(s.lag_ms[1]), Some(true), 0),
            ("servo lag M7 (ms)", |s| Some(s.lag_ms[2]), Some(true), 0),
            ("M6 goal miss (ticks)", |s| Some(s.track_err[1]), Some(true), 1),
            ("current M5 (mA)", |s| Some(s.current_ma[0]), Some(true), 0),
            ("current M6 (mA)", |s| Some(s.current_ma[1]), Some(true), 0),
            ("current M7 (mA)", |s| Some(s.current_ma[2]), Some(true), 0),
            ("deadband active (%)", |s| Some(s.deadband_pct), Some(false), 0),
            ("loop latency (ms)", |s| Some(s.latency_ms), Some(true), 1),
            ("ball found (%)", |s| Some(s.found_pct), Some(false), 1),
            ("kp", |s| s.kp, None, 4),
            ("kd", |s| s.kd, None, 4),
            ("slop M6 (ticks)", |s| s.slop.map(|v| v[1]), None, 0),
        ];
        let fmt = |v: Option<f64>, d: usize| v.filter(|x| x.is_finite()).map_or("–".to_string(), |x| format!("{x:.d$}"));
        let rows: Vec<Row> = metrics
            .iter()
            .map(|(label, g, lower_better, d)| {
                let a = g(s);
                let mut cells = vec![Cell::from(*label), Cell::from(fmt(a, *d))];
                if let Some(r) = rf {
                    let b = g(r);
                    cells.push(Cell::from(fmt(b, *d)));
                    let delta = match (a, b) {
                        (Some(x), Some(y)) if x.is_finite() && y.is_finite() => Some(x - y),
                        _ => None,
                    };
                    let color = match (delta, lower_better) {
                        (Some(dv), Some(lb)) if dv.abs() > 1e-9 => {
                            if (dv < 0.0) == *lb {
                                Color::Green
                            } else {
                                Color::Red
                            }
                        }
                        _ => Color::Reset,
                    };
                    cells.push(Cell::from(Span::styled(delta.map_or("–".into(), |x| format!("{x:+.d$}")), Style::new().fg(color))));
                }
                Row::new(cells)
            })
            .collect();
        let widths = [Constraint::Length(22), Constraint::Length(16), Constraint::Length(16), Constraint::Length(10)];
        let mut header = vec![Cell::from("metric"), Cell::from(Span::styled(s.name.clone(), Style::new().bold()))];
        if let Some(r) = rf {
            header.push(Cell::from(r.name.clone()));
            header.push(Cell::from("Δ"));
        }
        let [tbl, info] = Layout::vertical([Constraint::Min(10), Constraint::Length(3)]).areas(area);
        f.render_widget(
            Table::new(rows, widths)
                .header(Row::new(header).style(Style::new().fg(Color::Cyan)))
                .block(Block::bordered().title(" details (longest balance segment) ")),
            tbl,
        );
        let segs: Vec<String> = s.segments_s.iter().map(|x| format!("{x:.1}")).collect();
        f.render_widget(
            Paragraph::new(vec![Line::from(vec![
                Span::styled(" segments (s): ", Style::new().fg(Color::DarkGray)),
                Span::raw(segs.join(", ")),
            ])])
            .block(Block::bordered().title(format!(" {} ", path.display()))),
            info,
        );
    }
}
