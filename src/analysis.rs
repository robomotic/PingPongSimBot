//! Run-log analysis (Rust port of tools/analyze_run.py) plus the building blocks for live statistics.

use anyhow::{bail, Context, Result};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

pub fn median(v: &[f64]) -> f64 {
    percentile(v, 50.0)
}

pub fn percentile(v: &[f64], p: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let i = ((p / 100.0) * (s.len() - 1) as f64).round() as usize;
    s[i.min(s.len() - 1)]
}

pub fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        f64::NAN
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

/// Remove slow drift: subtract a centred moving average over `window` samples.
pub fn detrend(v: &[f64], window: usize) -> Vec<f64> {
    let n = v.len();
    let w = window.max(1);
    let mut prefix = vec![0.0; n + 1];
    for i in 0..n {
        prefix[i + 1] = prefix[i] + v[i];
    }
    (0..n)
        .map(|i| {
            let a = i.saturating_sub(w / 2);
            let b = (i + w / 2 + 1).min(n);
            v[i] - (prefix[b] - prefix[a]) / (b - a) as f64
        })
        .collect()
}

/// Strongest frequency between `fmin` and `fmax` Hz (direct DFT on a 0.01 Hz grid) and its amplitude.
pub fn dominant_freq(v: &[f64], dt: f64, fmin: f64, fmax: f64) -> (f64, f64) {
    let n = v.len();
    if n < 8 || dt <= 0.0 {
        return (f64::NAN, f64::NAN);
    }
    let m = mean(v);
    let mut best = (f64::NAN, 0.0);
    let mut f = fmin;
    while f <= fmax {
        let w = 2.0 * std::f64::consts::PI * f * dt;
        let (mut re, mut im) = (0.0, 0.0);
        for (i, x) in v.iter().enumerate() {
            let a = w * i as f64;
            re += (x - m) * a.cos();
            im -= (x - m) * a.sin();
        }
        let amp = 2.0 * (re * re + im * im).sqrt() / n as f64;
        if amp > best.1 {
            best = (f, amp);
        }
        f += 0.01;
    }
    best
}

/// Delay (samples) that best aligns `present` with `goal`: cross-correlation (0..max_lag) of their frame-to-frame
/// changes, so slow drift doesn't dominate. On the first real run this gives the same 5/5/4 frames as the raw signals.
pub fn lag_samples(goal: &[f64], present: &[f64], max_lag: usize) -> usize {
    let n = goal.len().min(present.len());
    if n < max_lag + 3 {
        return 0;
    }
    let dg: Vec<f64> = goal[..n].windows(2).map(|w| w[1] - w[0]).collect();
    let dp: Vec<f64> = present[..n].windows(2).map(|w| w[1] - w[0]).collect();
    let m = dg.len();
    (0..=max_lag)
        .map(|k| (k, (0..m - k).map(|i| dg[i] * dp[i + k]).sum::<f64>()))
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map_or(0, |(k, _)| k)
}

// ---------------------------------------------------------------------------------------------------------------

/// Everything the Results screen shows for one run (over its longest balance segment).
#[derive(Debug, Clone, Default)]
pub struct RunStats {
    pub path: PathBuf,
    pub name: String,
    pub frames: usize,
    pub duration_s: f64,
    pub fps: f64,
    pub segments_s: Vec<f64>,
    pub longest_s: f64,
    pub window_s: f64,
    pub median_mm: f64,
    pub p95_mm: f64,
    pub under10_pct: f64,
    pub osc_x: (f64, f64),
    pub osc_y: (f64, f64),
    pub tilt_std_deg: (f64, f64),
    pub lag_ms: [f64; 3],
    pub track_err: [f64; 3],
    pub current_ma: [f64; 3],
    pub latency_ms: f64,
    pub deadband_pct: f64,
    pub found_pct: f64,
    /// Gains active during the window (None for logs written before they were logged).
    pub kp: Option<f64>,
    pub kd: Option<f64>,
    pub slop: Option<[f64; 3]>,
}

struct Table {
    cols: HashMap<String, usize>,
    rows: Vec<Vec<String>>,
}

impl Table {
    fn col(&self, name: &str) -> Option<Vec<f64>> {
        let i = *self.cols.get(name)?;
        Some(self.rows.iter().map(|r| r.get(i).and_then(|v| v.parse().ok()).unwrap_or(f64::NAN)).collect())
    }
    fn text(&self, name: &str) -> Option<Vec<String>> {
        let i = *self.cols.get(name)?;
        Some(self.rows.iter().map(|r| r.get(i).cloned().unwrap_or_default()).collect())
    }
}

fn read_table(path: &Path) -> Result<Table> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let mut lines = text.lines();
    let header = lines.next().context("empty log")?;
    let cols = header.split(',').enumerate().map(|(i, n)| (n.trim().to_string(), i)).collect();
    let rows = lines.filter(|l| !l.is_empty()).map(|l| l.split(',').map(str::to_string).collect()).collect();
    Ok(Table { cols, rows })
}

pub fn analyze(path: &Path) -> Result<RunStats> {
    let tb = read_table(path)?;
    let t = tb.col("t").context("no t column")?;
    let state = tb.text("state").context("no state column")?;
    if t.len() < 10 {
        bail!("only {} rows", t.len());
    }
    let mut st = RunStats {
        path: path.to_path_buf(),
        name: path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default(),
        frames: t.len(),
        duration_s: t[t.len() - 1] - t[0],
        ..Default::default()
    };
    st.fps = st.frames as f64 / st.duration_s.max(1e-9);

    // Balance segments.
    let mut segs: Vec<(usize, usize)> = Vec::new();
    let mut start = None;
    for (i, s) in state.iter().enumerate() {
        let b = s == "Balance";
        if b && start.is_none() {
            start = Some(i);
        }
        if !b {
            if let Some(a) = start.take() {
                segs.push((a, i));
            }
        }
    }
    if let Some(a) = start {
        segs.push((a, state.len()));
    }
    st.segments_s = segs.iter().map(|&(a, b)| t[b - 1] - t[a]).collect();
    let Some(&(i0, i1)) = segs.iter().max_by(|a, b| (t[a.1 - 1] - t[a.0]).partial_cmp(&(t[b.1 - 1] - t[b.0])).unwrap()) else {
        return Ok(st);
    };
    st.longest_s = t[i1 - 1] - t[i0];
    st.window_s = st.longest_s;
    let w = |name: &str| tb.col(name).map(|v| v[i0..i1].to_vec());

    let (bx, by) = (w("bx").unwrap_or_default(), w("by").unwrap_or_default());
    let r: Vec<f64> = bx.iter().zip(&by).map(|(x, y)| x.hypot(*y)).collect();
    st.median_mm = median(&r);
    st.p95_mm = percentile(&r, 95.0);
    st.under10_pct = 100.0 * r.iter().filter(|&&x| x < 10.0).count() as f64 / r.len().max(1) as f64;

    let tw = &t[i0..i1];
    let dt = median(&tw.windows(2).map(|p| p[1] - p[0]).collect::<Vec<_>>());
    let smooth = ((5.0 / dt.max(1e-3)) as usize).max(3);
    st.osc_x = dominant_freq(&detrend(&bx, smooth), dt, 0.1, 3.0);
    st.osc_y = dominant_freq(&detrend(&by, smooth), dt, 0.1, 3.0);
    let sd = |v: &[f64]| {
        let m = mean(v);
        (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len().max(1) as f64).sqrt().to_degrees()
    };
    st.tilt_std_deg = (sd(&w("tilt_x").unwrap_or_default()), sd(&w("tilt_y").unwrap_or_default()));
    for (k, j) in ["5", "6", "7"].iter().enumerate() {
        if let (Some(g), Some(p)) = (w(&format!("g{j}")), w(&format!("p{j}"))) {
            st.lag_ms[k] = lag_samples(&g, &p, 14) as f64 * dt * 1e3;
            st.track_err[k] = mean(&g.iter().zip(&p).map(|(a, b)| (a - b).abs()).collect::<Vec<_>>());
        }
        st.current_ma[k] = mean(&w(&format!("i{j}")).unwrap_or_default());
    }
    st.latency_ms = median(&w("latency_ms").unwrap_or_default());
    let (vx, vy) = (w("vx").unwrap_or_default(), w("vy").unwrap_or_default());
    let db = w("deadband_mm").map_or(4.0, |v| median(&v));
    let quiet = (0..r.len()).filter(|&i| r[i] < db && vx.get(i).unwrap_or(&0.0).hypot(*vy.get(i).unwrap_or(&0.0)) < 10.0).count();
    st.deadband_pct = 100.0 * quiet as f64 / r.len().max(1) as f64;
    st.found_pct = 100.0 * mean(&w("found").unwrap_or_default());
    st.kp = w("kp").map(|v| median(&v));
    st.kd = w("kd").map(|v| median(&v));
    st.slop = match (w("bl5"), w("bl6"), w("bl7")) {
        (Some(a), Some(b), Some(c)) => Some([median(&a), median(&b), median(&c)]),
        _ => None,
    };
    Ok(st)
}

/// Run logs in `dirs`, newest first (by modification time).
pub fn list_runs(dirs: &[PathBuf]) -> Vec<PathBuf> {
    let mut v: Vec<(std::time::SystemTime, PathBuf)> = dirs
        .iter()
        .filter_map(|d| std::fs::read_dir(d).ok())
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "csv"))
        // throw_*.csv are servo/camera logs of a throw, not balance runs.
        .filter(|p| !p.file_name().and_then(|n| n.to_str()).is_some_and(|n| n.starts_with("throw_")))
        .filter_map(|p| Some((std::fs::metadata(&p).ok()?.modified().ok()?, p)))
        .collect();
    v.sort_by(|a, b| b.0.cmp(&a.0));
    v.into_iter().map(|(_, p)| p).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dominant_frequency_of_a_sine() {
        let dt = 1.0 / 29.0;
        let v: Vec<f64> = (0..600).map(|i| 7.0 * (2.0 * std::f64::consts::PI * 0.95 * i as f64 * dt).sin()).collect();
        let (f, a) = dominant_freq(&v, dt, 0.1, 3.0);
        assert!((f - 0.95).abs() < 0.03, "{f}");
        assert!((a - 7.0).abs() < 1.0, "{a}");
    }

    #[test]
    fn lag_of_a_delayed_copy() {
        let g: Vec<f64> = (0..300).map(|i| ((i as f64) * 0.2).sin()).collect();
        let p: Vec<f64> = (0..300).map(|i| if i >= 4 { g[i - 4] } else { 0.0 }).collect();
        assert_eq!(lag_samples(&g, &p, 14), 4);
    }

    #[test]
    fn first_real_run_matches_the_python_analysis() {
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("data/run_1795119100.csv");
        let s = analyze(&p).unwrap();
        assert!((s.longest_s - 125.3).abs() < 0.2, "{}", s.longest_s);
        assert!((s.median_mm - 20.4).abs() < 0.5, "{}", s.median_mm);
        assert!((s.osc_y.0 - 0.95).abs() < 0.05, "{:?}", s.osc_y);
        assert!(s.kp.is_none(), "old log has no gain columns");
        let frames: Vec<f64> = s.lag_ms.iter().map(|ms| (ms / 1e3 * s.fps).round()).collect();
        assert_eq!(frames, vec![5.0, 5.0, 4.0], "{:?}", s.lag_ms);
    }
}
