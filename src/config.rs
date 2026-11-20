//! `paddle.toml`: every value the balancer needs, with defaults for the ElRobot follower and the v2 paddle.
//! `jog`, `calib-camera` and `calib-axes` write their results back into this file.

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Motor IDs 1..=8 map to index 0..=7. M8 (gripper) is only written by `grip`, and at startup to hold the saved grip.
pub const N_MOTORS: usize = 8;
/// Wrist joints the controller moves (roll-pitch-roll).
pub const WRIST: [usize; 3] = [4, 5, 6]; // M5, M6, M7

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    pub bus: BusConfig,
    pub poses: Poses,
    pub joints: Joints,
    pub camera: CameraConfig,
    pub control: ControlConfig,
    pub safety: Safety,
    pub system: SystemConfig,
}

/// The Pi around the robot: the NormaCore station service and the wrist camera's USB identity.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct SystemConfig {
    /// systemd unit of the NormaCore station (owns the bus and the camera while it runs).
    pub station_service: String,
    /// USB vendor ID of the wrist camera (U20CAM: 0c45), for re-attaching it to the kernel.
    pub camera_usb_vendor: String,
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self { station_service: "normacore-station".into(), camera_usb_vendor: "0c45".into() }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct BusConfig {
    /// Follower bus. A /dev/serial/by-id path survives reboots and USB re-plugs.
    pub port: String,
    pub timeout_ms: u64,
    /// Servo acceleration register for M5-M7 while balancing (0 = max). Lower is gentler on current.
    pub wrist_acc: u8,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Poses {
    /// Rest pose ticks M1..M8. `run` refuses to start if the arm is far from it.
    pub rest: Option<[u16; N_MOTORS]>,
    /// Level pose ticks M1..M8 (paddle blade horizontal). Taught with `jog`.
    pub level: Option<[u16; N_MOTORS]>,
    /// M8 (gripper) goal that squeezes the paddle firmly. Taught with `grip`.
    pub grip: Option<u16>,
    /// Seconds for rest -> level and level -> rest moves.
    pub move_time_s: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Joints {
    /// rad = sign * (tick - zero) * 2pi/4096. The web viewer maps higher ticks to lower URDF angles, hence -1.
    pub sign: [f64; 7],
    pub zero: [f64; 7],
    /// Relative effort cost of M5, M6, M7 in the redundancy solver (higher = used less).
    pub wrist_weight: [f64; 3],
    /// Max excursion of M5-M7 from the level pose while balancing, ticks.
    pub wrist_range_ticks: [u16; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct CameraConfig {
    pub device: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    /// Manual exposure in 100 us units (40 = 4 ms, from RoboSim #69). None leaves auto exposure.
    pub exposure: Option<i64>,
    /// OpenCV-style HSV: H 0..179, S/V 0..255.
    pub hsv_low: [u8; 3],
    pub hsv_high: [u8; 3],
    /// Minimum blob area in half-resolution pixels.
    pub min_area: u32,
    /// Row-major 3x3 homography: full-res pixel -> blade mm. Set by `calib-camera`.
    pub homography: Option<[f64; 9]>,
    /// Only pixels that map inside this radius on the blade are searched.
    pub roi_radius_mm: f64,
    /// Ball-plane axes calibration (2x2, row-major): ball acceleration direction in blade mm
    /// for a unit tilt along the solver's (u, v) axes. Set by `calib-axes`.
    pub axes: Option<[f64; 4]>,
    /// Use only the directions of the measured axes (unit length). The measured magnitudes vary with the
    /// servos' backlash and the ball's start, so they make a poor gain; kp/kd set the gain instead.
    pub axes_normalize: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct ControlConfig {
    /// Tilt (rad) per mm of ball offset.
    pub kp: f64,
    /// Tilt (rad) per mm/s of ball velocity.
    pub kd: f64,
    /// Tilt (rad) per mm*s of integrated offset.
    pub ki: f64,
    pub i_limit_rad: f64,
    pub max_tilt_deg: f64,
    /// Max change of commanded tilt per second.
    pub max_tilt_rate_deg_s: f64,
    /// No correction while the ball is inside this radius and slower than `deadband_speed_mm_s`.
    pub deadband_mm: f64,
    pub deadband_speed_mm_s: f64,
    /// Alpha-beta filter gains.
    pub alpha: f64,
    pub beta: f64,
    /// Look-ahead to compensate camera + bus delay.
    pub predict_ms: f64,
    /// Goal changes smaller than this are not sent.
    pub min_goal_step_ticks: u16,
    /// Servo slop compensation for M5, M6, M7, in ticks: when a joint reverses direction, its goal is pushed this
    /// far past the target (half on each side of the dead zone) so the correction starts at once. 0 = off.
    pub backlash_ticks: [f64; 3],
    /// Null-space pull toward the level pose (fraction per tick).
    pub null_space_gain: f64,
    /// Damping of the pseudo-inverse near the M5/M7 singularity.
    pub damping: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Safety {
    /// Max distance of any joint from the rest pose at startup.
    pub start_tolerance_ticks: u16,
    pub ball_lost_s: f64,
    pub edge_radius_mm: f64,
    pub no_frame_s: f64,
    /// Frames in a row with a ball before balancing starts.
    pub ball_confirm_frames: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            bus: BusConfig::default(),
            poses: Poses::default(),
            joints: Joints::default(),
            camera: CameraConfig::default(),
            control: ControlConfig::default(),
            safety: Safety::default(),
            system: SystemConfig::default(),
        }
    }
}

impl Default for BusConfig {
    fn default() -> Self {
        Self {
            port: "/dev/ttyACM1".into(),
            timeout_ms: 20,
            wrist_acc: 100,
        }
    }
}

impl Default for Poses {
    fn default() -> Self {
        Self {
            rest: None,
            level: None,
            grip: None,
            move_time_s: 3.0,
        }
    }
}

impl Default for Joints {
    fn default() -> Self {
        Self {
            sign: [-1.0; 7],
            zero: [2048.0; 7],
            wrist_weight: [3.0, 1.5, 1.0],
            wrist_range_ticks: [250, 250, 250],
        }
    }
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            device: "/dev/video0".into(),
            width: 640,
            height: 480,
            fps: 30,
            exposure: Some(40),
            hsv_low: [5, 100, 40],
            hsv_high: [25, 255, 255],
            min_area: 60,
            homography: None,
            roi_radius_mm: 80.0,
            axes: None,
            axes_normalize: true,
        }
    }
}

impl Default for ControlConfig {
    fn default() -> Self {
        // Hollow ball: a = 0.6 g theta. PD at ~0.5 Hz, zeta 0.7 gives kp ~ 1.7e-3 rad/mm, kd ~ 0.8e-3 rad/(mm/s).
        // The first real run (2026-11-19) showed ~150 ms servo lag and a 1 Hz limit cycle; a sim fitted to it
        // prefers kd ~ 2.0e-3 (median ball offset 22 mm -> ~9 mm).
        Self {
            kp: 0.0017,
            kd: 0.002,
            ki: 0.0,
            i_limit_rad: 0.03,
            max_tilt_deg: 10.0,
            max_tilt_rate_deg_s: 60.0,
            deadband_mm: 4.0,
            deadband_speed_mm_s: 10.0,
            alpha: 0.6,
            beta: 0.2,
            predict_ms: 60.0,
            min_goal_step_ticks: 2,
            backlash_ticks: [0.0, 0.0, 0.0],
            null_space_gain: 0.05,
            damping: 0.02,
        }
    }
}

impl Default for Safety {
    fn default() -> Self {
        Self {
            start_tolerance_ticks: 150,
            ball_lost_s: 0.5,
            edge_radius_mm: 65.0,
            no_frame_s: 0.2,
            ball_confirm_frames: 10,
        }
    }
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        if !path.exists() {
            log::warn!("{} not found, using defaults", path.display());
            return Ok(Self::default());
        }
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))
    }

    /// The calibrated config of the real NormaStation arm (config/paddle.example.toml), built in. Used by the
    /// simulator when the local config lacks a level pose or calibrations.
    pub fn example() -> Self {
        toml::from_str(include_str!("../config/paddle.example.toml")).expect("built-in example config parses")
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        let text = toml::to_string_pretty(self)?;
        std::fs::write(path, text).with_context(|| format!("writing {}", path.display()))?;
        log::info!("saved {}", path.display());
        Ok(())
    }

    pub fn tick_to_rad(&self, joint: usize, tick: f64) -> f64 {
        self.joints.sign[joint] * (tick - self.joints.zero[joint]) * std::f64::consts::TAU / 4096.0
    }

    pub fn rad_to_tick(&self, joint: usize, rad: f64) -> f64 {
        rad / (self.joints.sign[joint] * std::f64::consts::TAU / 4096.0) + self.joints.zero[joint]
    }

    pub fn level(&self) -> Result<[u16; N_MOTORS]> {
        self.poses
            .level
            .context("no level pose in config: teach it with `paddle-balance jog`, then `save level`")
    }
}
