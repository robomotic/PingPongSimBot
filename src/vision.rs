//! Orange-ball detection on raw YUYV frames from the U20CAM wrist camera, in pure Rust.
//!
//! Every frame is processed at half resolution (one sample per YUYV macropixel, every other row): converted to
//! OpenCV-style HSV, thresholded, and the largest 4-connected blob is taken as the ball. Thresholds and camera
//! settings come from RoboSim #69 (H 5-25, S >= 100, V >= 40; manual exposure 4 ms, auto white balance, 2 buffers).

use crate::config::CameraConfig;
use crate::estimator::Homography;
use nalgebra::Vector2;
use std::time::Instant;

#[derive(Debug, Clone, Copy)]
pub struct Blob {
    /// Centroid in full-resolution pixels.
    pub px: Vector2<f64>,
    /// Area in half-resolution pixels.
    pub area: u32,
    /// Mean HSV of the blob (OpenCV ranges).
    pub hsv: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
pub struct Detection {
    pub seq: u64,
    pub t: Instant,
    pub blob: Option<Blob>,
    /// Number of separate blobs that passed the threshold (false-positive check).
    pub n_blobs: u32,
    pub detect_ms: f64,
}

/// OpenCV `COLOR_RGB2HSV` for 8-bit images: H 0..179, S, V 0..255.
pub fn rgb_to_hsv(r: u8, g: u8, b: u8) -> [u8; 3] {
    let (r, g, b) = (r as i32, g as i32, b as i32);
    let v = r.max(g).max(b);
    let min = r.min(g).min(b);
    let d = v - min;
    let s = if v == 0 { 0 } else { (255 * d + v / 2) / v };
    let (d, v_f) = (d as f64, v);
    let h = if d == 0.0 {
        0.0
    } else if v_f == r {
        60.0 * (g - b) as f64 / d
    } else if v_f == g {
        120.0 + 60.0 * (b - r) as f64 / d
    } else {
        240.0 + 60.0 * (r - g) as f64 / d
    };
    let h = if h < 0.0 { h + 360.0 } else { h };
    [((h / 2.0).round() as i32 % 180) as u8, s as u8, v as u8]
}

/// BT.601 limited-range YUV -> RGB (what UVC webcams send).
pub fn yuv_to_rgb(y: u8, u: u8, v: u8) -> (u8, u8, u8) {
    let c = (y as f32 - 16.0) * 1.164;
    let d = u as f32 - 128.0;
    let e = v as f32 - 128.0;
    let cl = |x: f32| x.round().clamp(0.0, 255.0) as u8;
    (cl(c + 1.596 * e), cl(c - 0.392 * d - 0.813 * e), cl(c + 2.017 * d))
}

pub struct Detector {
    w: usize,
    h: usize,
    low: [u8; 3],
    high: [u8; 3],
    min_area: u32,
    /// Half-resolution ROI; None = whole frame.
    roi: Option<Vec<bool>>,
    mask: Vec<u8>,
    hsv: Vec<[u8; 3]>,
    stack: Vec<usize>,
}

impl Detector {
    pub fn new(cfg: &CameraConfig, homography: Option<Homography>) -> Self {
        let (w, h) = (cfg.width as usize / 2, cfg.height as usize / 2);
        let roi = homography.map(|hm| {
            let mut m = vec![false; w * h];
            for y in 0..h {
                for x in 0..w {
                    let p = hm.apply(Vector2::new(2.0 * x as f64 + 0.5, 2.0 * y as f64 + 0.5));
                    m[y * w + x] = p.norm() <= cfg.roi_radius_mm;
                }
            }
            m
        });
        Self {
            w,
            h,
            low: cfg.hsv_low,
            high: cfg.hsv_high,
            min_area: cfg.min_area,
            roi,
            mask: vec![0; w * h],
            hsv: vec![[0; 3]; w * h],
            stack: Vec::new(),
        }
    }

    pub fn set_thresholds(&mut self, low: [u8; 3], high: [u8; 3], min_area: u32) {
        self.low = low;
        self.high = high;
        self.min_area = min_area;
    }

    /// Hue may wrap (low > high), e.g. 170..10 for red.
    pub fn in_range(&self, hsv: [u8; 3]) -> bool {
        let h_ok = if self.low[0] <= self.high[0] {
            hsv[0] >= self.low[0] && hsv[0] <= self.high[0]
        } else {
            hsv[0] >= self.low[0] || hsv[0] <= self.high[0]
        };
        h_ok && (1..3).all(|k| hsv[k] >= self.low[k] && hsv[k] <= self.high[k])
    }

    /// Returns (largest blob, number of blobs >= min_area / 4).
    pub fn detect_yuyv(&mut self, frame: &[u8]) -> (Option<Blob>, u32) {
        let (w, h) = (self.w, self.h);
        let stride = w * 2 * 2; // full-res width * 2 bytes per pixel
        if frame.len() < stride * h * 2 {
            return (None, 0);
        }
        // Pass 1: threshold. mask: 0 = no, 1 = candidate, 2 = visited.
        let mut hsv_buf = std::mem::take(&mut self.hsv);
        hsv_buf.resize(w * h, [0; 3]);
        for y in 0..h {
            let row = &frame[(2 * y) * stride..(2 * y + 1) * stride];
            for x in 0..w {
                let i = y * w + x;
                if let Some(roi) = &self.roi {
                    if !roi[i] {
                        self.mask[i] = 0;
                        continue;
                    }
                }
                let p = &row[x * 4..x * 4 + 4];
                let yy = ((p[0] as u16 + p[2] as u16) / 2) as u8;
                let (r, g, b) = yuv_to_rgb(yy, p[1], p[3]);
                let hsv = rgb_to_hsv(r, g, b);
                hsv_buf[i] = hsv;
                self.mask[i] = self.in_range(hsv) as u8;
            }
        }
        // Pass 2: 4-connected components, keep the largest.
        let mut best: Option<(u32, f64, f64, [f64; 3])> = None;
        let mut n_blobs = 0;
        for start in 0..w * h {
            if self.mask[start] != 1 {
                continue;
            }
            self.mask[start] = 2;
            self.stack.clear();
            self.stack.push(start);
            let (mut area, mut sx, mut sy) = (0u32, 0f64, 0f64);
            let mut shsv = [0f64; 3];
            while let Some(i) = self.stack.pop() {
                let (x, y) = (i % w, i / w);
                area += 1;
                sx += x as f64;
                sy += y as f64;
                for k in 0..3 {
                    shsv[k] += hsv_buf[i][k] as f64;
                }
                let push = |j: usize, m: &mut Vec<u8>, s: &mut Vec<usize>| {
                    if m[j] == 1 {
                        m[j] = 2;
                        s.push(j);
                    }
                };
                if x > 0 { push(i - 1, &mut self.mask, &mut self.stack); }
                if x + 1 < w { push(i + 1, &mut self.mask, &mut self.stack); }
                if y > 0 { push(i - w, &mut self.mask, &mut self.stack); }
                if y + 1 < h { push(i + w, &mut self.mask, &mut self.stack); }
            }
            if area >= (self.min_area / 4).max(1) {
                n_blobs += 1;
            }
            if best.map_or(true, |b| area > b.0) {
                best = Some((area, sx, sy, shsv));
            }
        }
        let blob = best.filter(|b| b.0 >= self.min_area).map(|(area, sx, sy, shsv)| {
            let a = area as f64;
            Blob {
                px: Vector2::new(2.0 * sx / a + 0.5, 2.0 * sy / a + 0.5),
                area,
                hsv: [shsv[0] / a, shsv[1] / a, shsv[2] / a],
            }
        });
        self.hsv = hsv_buf;
        (blob, n_blobs)
    }
}

/// Make a YUYV frame of a flat background with an orange disc, for tests.
#[cfg(test)]
pub fn synthetic_frame(w: usize, h: usize, cx: f64, cy: f64, r: f64) -> Vec<u8> {
    let mut f = vec![0u8; w * h * 2];
    for y in 0..h {
        for x2 in 0..w / 2 {
            let x = 2 * x2;
            let inside = ((x as f64 - cx).powi(2) + (y as f64 - cy).powi(2)).sqrt() <= r;
            // Orange (R 240, G 120, B 20) vs dark grey, in BT.601 limited range.
            let (yy, u, v) = if inside { (140u8, 66u8, 188u8) } else { (40u8, 128u8, 128u8) };
            let o = (y * w + x) * 2;
            f[o..o + 4].copy_from_slice(&[yy, u, yy, v]);
        }
    }
    f
}

/// Detector settings that can change while the camera runs (Camera screen).
#[derive(Debug, Clone, PartialEq)]
pub struct Tuning {
    pub hsv_low: [u8; 3],
    pub hsv_high: [u8; 3],
    pub min_area: u32,
    /// Manual exposure (100 us units); None = leave as is.
    pub exposure: Option<i64>,
    /// Produce low-resolution previews (costs ~1 ms per frame; only while a screen shows them).
    pub preview: bool,
}

impl Tuning {
    pub fn from_cfg(c: &CameraConfig) -> Self {
        Self { hsv_low: c.hsv_low, hsv_high: c.hsv_high, min_area: c.min_area, exposure: c.exposure, preview: false }
    }
}

/// Shared tuning with a version counter, so the capture thread notices changes cheaply.
pub type SharedTuning = std::sync::Arc<std::sync::Mutex<(u64, Tuning)>>;

/// Low-resolution colour frame (every 4th pixel and row) with the threshold mask.
#[derive(Debug, Clone)]
pub struct Preview {
    pub w: usize,
    pub h: usize,
    pub rgb: Vec<[u8; 3]>,
    pub mask: Vec<bool>,
    pub seq: u64,
}

pub fn make_preview(frame: &[u8], w: usize, h: usize, det: &Detector, seq: u64) -> Preview {
    let (pw, ph) = (w / 4, h / 4);
    let mut rgb = Vec::with_capacity(pw * ph);
    let mut mask = Vec::with_capacity(pw * ph);
    for y in 0..ph {
        let row = 4 * y * w * 2;
        for x in 0..pw {
            let o = row + (4 * x) * 2; // macropixel start (x even)
            if o + 3 >= frame.len() {
                rgb.push([0, 0, 0]);
                mask.push(false);
                continue;
            }
            let (r, g, b) = yuv_to_rgb(frame[o], frame[o + 1], frame[o + 3]);
            rgb.push([r, g, b]);
            mask.push(det.in_range(rgb_to_hsv(r, g, b)));
        }
    }
    Preview { w: pw, h: ph, rgb, mask, seq }
}

/// Per-frame work shared by the real and the simulated camera: detect, apply tuning changes, make previews.
pub struct FrameProc {
    pub detector: Detector,
    tuning: SharedTuning,
    version: u64,
    current: Tuning,
    tx: tokio::sync::watch::Sender<Detection>,
    ptx: tokio::sync::watch::Sender<Option<std::sync::Arc<Preview>>>,
    seq: u64,
    w: usize,
    h: usize,
}

impl FrameProc {
    /// Returns the new exposure when the tuning asks for a different one, and false when nobody listens anymore.
    pub fn handle(&mut self, buf: &[u8]) -> (bool, Option<i64>) {
        let t = Instant::now();
        let mut exposure = None;
        if let Ok(g) = self.tuning.lock() {
            if g.0 != self.version {
                self.version = g.0;
                let new = g.1.clone();
                self.detector.set_thresholds(new.hsv_low, new.hsv_high, new.min_area);
                if new.exposure != self.current.exposure {
                    exposure = new.exposure;
                }
                self.current = new;
            }
        }
        let (blob, n_blobs) = self.detector.detect_yuyv(buf);
        self.seq += 1;
        let detect_ms = t.elapsed().as_secs_f64() * 1e3;
        if self.current.preview && self.seq % 2 == 0 {
            let p = make_preview(buf, self.w, self.h, &self.detector, self.seq);
            let _ = self.ptx.send(Some(std::sync::Arc::new(p)));
        }
        let alive = self.tx.send(Detection { seq: self.seq, t, blob, n_blobs, detect_ms }).is_ok();
        (alive, exposure)
    }
}

/// A running camera (real V4L2 or simulated): detections, optional previews, live tuning.
pub struct Camera {
    pub rx: tokio::sync::watch::Receiver<Detection>,
    pub preview: tokio::sync::watch::Receiver<Option<std::sync::Arc<Preview>>>,
    pub tuning: SharedTuning,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Camera {
    /// Change the tuning (bumps the version so the capture thread applies it).
    pub fn set_tuning(&self, f: impl FnOnce(&mut Tuning)) {
        if let Ok(mut g) = self.tuning.lock() {
            f(&mut g.1);
            g.0 += 1;
        }
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// Channels + per-frame processor for a new camera thread.
fn new_pipeline(cfg: &CameraConfig, homography: Option<Homography>) -> (FrameProc, Camera) {
    let (tx, rx) = tokio::sync::watch::channel(Detection { seq: 0, t: Instant::now(), blob: None, n_blobs: 0, detect_ms: 0.0 });
    let (ptx, prx) = tokio::sync::watch::channel(None);
    let tuning = Tuning::from_cfg(cfg);
    let shared: SharedTuning = std::sync::Arc::new(std::sync::Mutex::new((0, tuning.clone())));
    let proc_ = FrameProc {
        detector: Detector::new(cfg, homography),
        tuning: shared.clone(),
        version: 0,
        current: tuning,
        tx,
        ptx,
        seq: 0,
        w: cfg.width as usize,
        h: cfg.height as usize,
    };
    let cam = Camera { rx, preview: prx, tuning: shared, stop: Default::default(), handle: None };
    (proc_, cam)
}

/// Simulated camera: renders the blade and an orange ball into a real-size YUYV frame through the inverse of
/// `world` (pixel -> blade mm on the ball-centre plane) and runs the normal detector on it, at `fps`.
/// `ball` (blade mm) is shared, so a screen or the simulator can move the ball.
pub fn start_sim_camera(
    cfg: &CameraConfig,
    roi: Option<Homography>,
    world: Homography,
    ball: std::sync::Arc<std::sync::Mutex<Option<Vector2<f64>>>>,
    fps: f64,
) -> Camera {
    let (mut proc_, mut cam) = new_pipeline(cfg, roi);
    let stop = cam.stop.clone();
    let (w, h) = (cfg.width as usize, cfg.height as usize);
    cam.handle = Some(std::thread::spawn(move || {
        // Blade mm of every macropixel, computed once.
        let lut: Vec<Vector2<f64>> = (0..h).flat_map(|y| (0..w / 2).map(move |x2| (x2, y))).map(|(x2, y)| world.apply(Vector2::new(2.0 * x2 as f64 + 1.0, y as f64))).collect();
        let mut frame = vec![0u8; w * h * 2];
        let mut rng = 0x9E37_79B9_7F4A_7C15u64;
        let period = std::time::Duration::from_secs_f64(1.0 / fps);
        while !stop.load(std::sync::atomic::Ordering::Relaxed) {
            let t0 = Instant::now();
            let b = ball.lock().ok().and_then(|g| *g);
            for (i, mm) in lut.iter().enumerate() {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                let noise = (rng % 7) as i32 - 3;
                let (yy, u, v) = match b.map(|b| (mm - b).norm()) {
                    Some(d) if d < 20.0 => {
                        // orange, darker towards the rim
                        let shade = (1.0 - 0.35 * (d / 20.0).powi(2)) as f32;
                        ((140.0 * shade) as i32, 66, 188)
                    }
                    _ if mm.norm() < 75.0 => (24, 128, 128),
                    _ => (150 - (mm.norm() / 10.0).min(40.0) as i32, 126, 130),
                };
                let y = (yy + noise).clamp(16, 235) as u8;
                frame[i * 4..i * 4 + 4].copy_from_slice(&[y, u as u8, y, v as u8]);
            }
            let (alive, _) = proc_.handle(&frame);
            if !alive {
                break;
            }
            if let Some(rest) = period.checked_sub(t0.elapsed()) {
                std::thread::sleep(rest);
            }
        }
    }));
    cam
}

#[cfg(target_os = "linux")]
pub mod capture {
    use super::*;
    use anyhow::{Context, Result};
    use std::sync::atomic::Ordering;
    use v4l::buffer::Type;
    use v4l::control::{Control, Value};
    use v4l::io::traits::CaptureStream;
    use v4l::prelude::*;
    use v4l::video::Capture;

    // V4L2 control IDs (linux/v4l2-controls.h).
    const CID_EXPOSURE_AUTO: u32 = 0x009a_0901; // 1 = manual
    const CID_EXPOSURE_ABSOLUTE: u32 = 0x009a_0902; // 100 us units
    const CID_EXPOSURE_AUTO_PRIORITY: u32 = 0x009a_0903; // "exposure_dynamic_framerate"
    const CID_AUTO_WHITE_BALANCE: u32 = 0x0098_090c;

    fn set(dev: &Device, id: u32, v: i64, name: &str) {
        if let Err(e) = dev.set_control(Control { id, value: Value::Integer(v) }) {
            log::warn!("camera: could not set {name}={v}: {e}");
        }
    }

    /// Opens the camera and starts a capture + detection thread. `snapshot` (if set) receives the
    /// first raw YUYV frame, for debugging.
    pub fn start(
        cfg: &CameraConfig,
        homography: Option<Homography>,
        snapshot: Option<std::sync::mpsc::Sender<Vec<u8>>>,
    ) -> Result<Camera> {
        let dev = Device::with_path(&cfg.device).with_context(|| {
            format!("opening {}; the station's usb-video driver holds the camera while it runs", cfg.device)
        })?;
        let mut fmt = dev.format()?;
        fmt.width = cfg.width;
        fmt.height = cfg.height;
        fmt.fourcc = v4l::FourCC::new(b"YUYV");
        let fmt = dev.set_format(&fmt)?;
        if &fmt.fourcc.repr != b"YUYV" || fmt.width != cfg.width || fmt.height != cfg.height {
            anyhow::bail!("camera gave {}x{} {} instead of {}x{} YUYV", fmt.width, fmt.height, fmt.fourcc, cfg.width, cfg.height);
        }
        let mut params = dev.params()?;
        params.interval = v4l::Fraction::new(1, cfg.fps);
        let _ = dev.set_params(&params);
        set(&dev, CID_AUTO_WHITE_BALANCE, 1, "white_balance_automatic");
        if let Some(exp) = cfg.exposure {
            set(&dev, CID_EXPOSURE_AUTO, 1, "auto_exposure");
            set(&dev, CID_EXPOSURE_ABSOLUTE, exp, "exposure_time_absolute");
            set(&dev, CID_EXPOSURE_AUTO_PRIORITY, 0, "exposure_dynamic_framerate");
        }

        let (mut proc_, mut cam) = super::new_pipeline(cfg, homography);
        let stop2 = cam.stop.clone();
        let handle = std::thread::Builder::new().name("camera".into()).spawn(move || {
            // Two buffers: with one, every other frame is dropped (15 fps instead of 29, RoboSim #69).
            let mut stream = match MmapStream::with_buffers(&dev, Type::VideoCapture, 2) {
                Ok(s) => s,
                Err(e) => {
                    log::error!("camera stream: {e}");
                    return;
                }
            };
            let mut snapshot = snapshot;
            while !stop2.load(Ordering::Relaxed) {
                let (buf, _meta) = match stream.next() {
                    Ok(f) => f,
                    Err(e) => {
                        log::error!("camera frame: {e}");
                        std::thread::sleep(std::time::Duration::from_millis(10));
                        continue;
                    }
                };
                if let Some(s) = snapshot.take() {
                    let _ = s.send(buf.to_vec());
                }
                let (alive, exposure) = proc_.handle(buf);
                if !alive {
                    break;
                }
                if let Some(e) = exposure {
                    set(&dev, CID_EXPOSURE_AUTO, 1, "auto_exposure");
                    set(&dev, CID_EXPOSURE_ABSOLUTE, e, "exposure_time_absolute");
                }
            }
        })?;
        cam.handle = Some(handle);
        Ok(cam)
    }
}

/// Write a YUYV frame as a binary PPM, marking the detected centroid with a cross.
pub fn save_ppm(path: &std::path::Path, frame: &[u8], w: usize, h: usize, mark: Option<Vector2<f64>>) -> std::io::Result<()> {
    let mut rgb = vec![0u8; w * h * 3];
    for i in 0..w * h / 2 {
        let p = &frame[i * 4..i * 4 + 4];
        for (k, yy) in [p[0], p[2]].into_iter().enumerate() {
            let (r, g, b) = yuv_to_rgb(yy, p[1], p[3]);
            let o = (2 * i + k) * 3;
            rgb[o..o + 3].copy_from_slice(&[r, g, b]);
        }
    }
    if let Some(m) = mark {
        let (cx, cy) = (m.x as i64, m.y as i64);
        for d in -12i64..=12 {
            for (x, y) in [(cx + d, cy), (cx, cy + d)] {
                if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
                    let o = (y as usize * w + x as usize) * 3;
                    rgb[o..o + 3].copy_from_slice(&[0, 255, 0]);
                }
            }
        }
    }
    let mut out = format!("P6\n{w} {h}\n255\n").into_bytes();
    out.extend_from_slice(&rgb);
    std::fs::write(path, out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hsv_matches_opencv_for_orange() {
        // OpenCV: (R 255, G 128, B 0) -> H 15, S 255, V 255
        assert_eq!(rgb_to_hsv(255, 128, 0), [15, 255, 255]);
        assert_eq!(rgb_to_hsv(0, 0, 255), [120, 255, 255]);
        assert_eq!(rgb_to_hsv(10, 10, 10), [0, 0, 10]);
    }

    #[test]
    fn synthetic_disc_is_found_at_its_centre() {
        let cfg = CameraConfig::default();
        let f = synthetic_frame(640, 480, 332.0, 165.0, 72.0);
        let mut d = Detector::new(&cfg, None);
        let (blob, n) = d.detect_yuyv(&f);
        let b = blob.expect("ball");
        assert_eq!(n, 1);
        assert!((b.px - Vector2::new(332.0, 165.0)).norm() < 2.0, "{:?}", b.px);
        let expected_area = std::f64::consts::PI * 36.0 * 36.0;
        assert!((b.area as f64 - expected_area).abs() / expected_area < 0.1);
        assert!(b.hsv[0] >= 5.0 && b.hsv[0] <= 25.0, "{:?}", b.hsv);
    }

    #[test]
    fn simulated_camera_sees_the_ball_where_it_is() {
        let cfg = crate::config::Config::example();
        let h = Homography::from_row_major(&cfg.camera.homography.unwrap());
        let ball = std::sync::Arc::new(std::sync::Mutex::new(Some(Vector2::new(12.0, -8.0))));
        let mut cam = start_sim_camera(&cfg.camera, Some(h), h, ball.clone(), 30.0);
        cam.set_tuning(|t| t.preview = true);
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        let det = rt.block_on(async {
            for _ in 0..5 {
                cam.rx.changed().await.unwrap();
            }
            *cam.rx.borrow()
        });
        let b = det.blob.expect("ball found");
        let mm = h.apply(b.px);
        // The simulator draws the ball as a flat disc on a strongly slanted plane (0.66 px/mm near/far), whose image
        // centroid sits ~2.5 mm off the projected centre. A real (spherical) ball doesn't have this bias.
        assert!((mm - Vector2::new(12.0, -8.0)).norm() < 3.5, "{mm:?}");
        assert!(cam.preview.borrow().is_some(), "preview produced");
        *ball.lock().unwrap() = None;
        let det = rt.block_on(async {
            for _ in 0..3 {
                cam.rx.changed().await.unwrap();
            }
            *cam.rx.borrow()
        });
        assert!(det.blob.is_none());
    }

    #[test]
    fn empty_frame_finds_nothing() {
        let cfg = CameraConfig::default();
        let f = synthetic_frame(640, 480, -500.0, -500.0, 1.0);
        let mut d = Detector::new(&cfg, None);
        assert!(d.detect_yuyv(&f).0.is_none());
    }
}
