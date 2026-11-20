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

    /// Hue may wrap (low > high), e.g. 170..10 for red.
    fn in_range(&self, hsv: [u8; 3]) -> bool {
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

#[cfg(target_os = "linux")]
pub mod capture {
    use super::*;
    use anyhow::{Context, Result};
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use tokio::sync::watch;
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

    pub struct Camera {
        pub rx: watch::Receiver<Detection>,
        stop: Arc<AtomicBool>,
        handle: Option<std::thread::JoinHandle<()>>,
    }

    impl Drop for Camera {
        fn drop(&mut self) {
            self.stop.store(true, Ordering::Relaxed);
            if let Some(h) = self.handle.take() {
                let _ = h.join();
            }
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

        let mut detector = Detector::new(cfg, homography);
        let (tx, rx) = watch::channel(Detection { seq: 0, t: Instant::now(), blob: None, n_blobs: 0, detect_ms: 0.0 });
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
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
            let mut seq = 0;
            while !stop2.load(Ordering::Relaxed) {
                let (buf, _meta) = match stream.next() {
                    Ok(f) => f,
                    Err(e) => {
                        log::error!("camera frame: {e}");
                        std::thread::sleep(std::time::Duration::from_millis(10));
                        continue;
                    }
                };
                let t = Instant::now();
                if let Some(s) = snapshot.take() {
                    let _ = s.send(buf.to_vec());
                }
                let (blob, n_blobs) = detector.detect_yuyv(buf);
                seq += 1;
                let detect_ms = t.elapsed().as_secs_f64() * 1e3;
                if tx.send(Detection { seq, t, blob, n_blobs, detect_ms }).is_err() {
                    break;
                }
            }
        })?;
        Ok(Camera { rx, stop, handle: Some(handle) })
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
    fn empty_frame_finds_nothing() {
        let cfg = CameraConfig::default();
        let f = synthetic_frame(640, 480, -500.0, -500.0, 1.0);
        let mut d = Detector::new(&cfg, None);
        assert!(d.detect_yuyv(&f).0.is_none());
    }
}
