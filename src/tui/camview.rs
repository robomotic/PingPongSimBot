//! Shared camera plumbing for the Camera and Calibrate screens: open a real or simulated camera, and draw a preview.

use super::Ctx;
use crate::config::Config;
use crate::estimator::Homography;
use crate::vision::{Camera, Preview};
use anyhow::Result;
use nalgebra::Vector2;
use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::Color;
use std::sync::{Arc, Mutex};

pub type SimBall = Arc<Mutex<Option<Vector2<f64>>>>;

/// Opens the camera with previews on. In simulation, returns the handle that places the simulated ball (blade mm).
/// `roi`: restrict detection to the blade (needs the camera map).
pub fn open(ctx: &Ctx, roi: bool) -> Result<(Camera, Option<SimBall>)> {
    let cfg = &ctx.cfg;
    let map = cfg.camera.homography.map(|h| Homography::from_row_major(&h));
    let roi_h = if roi { map } else { None };
    let cam = if ctx.sim {
        let world = map.unwrap_or_else(|| Homography::from_row_major(&Config::example().camera.homography.unwrap()));
        let ball: SimBall = Arc::new(Mutex::new(Some(Vector2::new(6.0, -4.0))));
        let cam = crate::vision::start_sim_camera(&cfg.camera, roi_h, world, ball.clone(), 15.0);
        cam.set_tuning(|t| t.preview = true);
        return Ok((cam, Some(ball)));
    } else {
        #[cfg(target_os = "linux")]
        {
            crate::vision::capture::start(&cfg.camera, roi_h, None)?
        }
        #[cfg(not(target_os = "linux"))]
        {
            anyhow::bail!("the camera needs Linux")
        }
    };
    cam.set_tuning(|t| t.preview = true);
    Ok((cam, None))
}

/// Draws `p` into `area` with half-block cells (2 pixels per cell), keeping the aspect ratio.
/// `mask`: tint thresholded pixels magenta. `marks`: (x, y) in full-resolution pixels (4× the preview), with colours.
pub fn draw_preview(buf: &mut Buffer, area: Rect, p: &Preview, mask: bool, marks: &[(f64, f64, Color)]) {
    if area.width == 0 || area.height == 0 || p.w == 0 {
        return;
    }
    // Pixel grid of the area: width × 2·height. Fit the 4:3 preview inside it.
    let (aw, ah) = (area.width as f64, area.height as f64 * 2.0);
    let scale = (aw / p.w as f64).min(ah / p.h as f64);
    let (dw, dh) = ((p.w as f64 * scale) as u16, (p.h as f64 * scale) as u16);
    let x0 = area.x + (area.width - dw.min(area.width)) / 2;
    let y0 = area.y;
    let sample = |px: u16, py: u16| -> [u8; 3] {
        let sx = ((px as f64 / scale) as usize).min(p.w - 1);
        let sy = ((py as f64 / scale) as usize).min(p.h - 1);
        let i = sy * p.w + sx;
        if mask && p.mask[i] {
            [255, 0, 200]
        } else {
            p.rgb[i]
        }
    };
    for cy in 0..(dh / 2).min(area.height) {
        for cx in 0..dw.min(area.width) {
            let top = sample(cx, cy * 2);
            let bot = sample(cx, cy * 2 + 1);
            if let Some(cell) = buf.cell_mut((x0 + cx, y0 + cy)) {
                cell.set_char('▀').set_fg(Color::Rgb(top[0], top[1], top[2])).set_bg(Color::Rgb(bot[0], bot[1], bot[2]));
            }
        }
    }
    for &(mx, my, c) in marks {
        // full-res px -> preview px -> cell
        let cx = (mx / 4.0 * scale) as i32;
        let cy = (my / 4.0 * scale / 2.0) as i32;
        for (dx, dy, ch) in [(0, 0, '╋'), (-1, 0, '━'), (1, 0, '━'), (0, -1, '┃'), (0, 1, '┃')] {
            let (x, y) = (cx + dx, cy + dy);
            if x >= 0 && y >= 0 && (x as u16) < dw.min(area.width) && (y as u16) < (dh / 2).min(area.height) {
                if let Some(cell) = buf.cell_mut((x0 + x as u16, y0 + y as u16)) {
                    cell.set_char(ch).set_fg(c);
                }
            }
        }
    }
}
