//! paddle-balance: keep an orange ball on a paddle held by the ElRobot follower.
//!
//! Talks to the follower's ST3215 bus directly (stop the station first) and reads the wrist camera with V4L2.
//! Typical session on the Pi:
//!
//!   sudo systemctl stop normacore-station
//!   paddle-balance scan --save-rest       # check the bus, remember the rest pose
//!   paddle-balance jog                    # teach the level pose: 'save level'
//!   paddle-balance grip                   # squeeze the paddle firmly: 'save'
//!   paddle-balance vision-test            # ball found, fps
//!   paddle-balance calib-camera           # pixel -> blade mm
//!   paddle-balance calib-axes             # which way the ball rolls for each wrist tilt
//!   paddle-balance run                    # rest -> level -> wait for ball -> balance; Ctrl-C -> rest
//!   paddle-balance                        # full-screen app (Balance + Results); `tui --sim` without the robot
//!   sudo systemctl start normacore-station

use paddle_balance::{analysis, arm, balance, bus, calib, config, control, estimator, kinematics, sim, vision};

#[cfg(target_os = "linux")]
mod app;
#[cfg(target_os = "linux")]
mod grip_tui;
#[cfg(target_os = "linux")]
mod tui;

use clap::{Parser, Subcommand};
use std::path::PathBuf;

#[derive(Parser)]
#[command(version, about)]
struct Cli {
    /// Config file (created/updated by the calibration commands).
    #[arg(short, long, default_value = "paddle.toml", global = true)]
    config: PathBuf,
    /// No command: open the full-screen app.
    #[command(subcommand)]
    cmd: Option<Cmd>,
}

#[derive(Subcommand)]
enum Cmd {
    /// Full-screen app (the default when no command is given).
    Tui {
        /// Simulated arm and ball: nothing moves, no hardware needed.
        #[arg(long)]
        sim: bool,
        #[arg(long, default_value = "paddle_logs")]
        log_dir: PathBuf,
    },
    /// Ping M1-M8 and print their state (no motion).
    Scan {
        /// Store the present pose as the rest pose.
        #[arg(long)]
        save_rest: bool,
    },
    /// Interactively nudge M1-M7 to teach the level pose.
    Jog,
    /// Turn gripper (M8) torque on and tighten/loosen it in small steps; 'save' stores the grip.
    Grip,
    /// Run the ball detector and report fps / detections (no motion).
    VisionTest {
        #[arg(long, default_value_t = 10.0)]
        seconds: f64,
        /// Save the first frame as a PPM with the detection marked.
        #[arg(long)]
        snapshot: Option<PathBuf>,
    },
    /// Fit the pixel -> blade-mm map from 5 ball placements (no motion).
    CalibCamera {
        #[arg(long, default_value_t = 50.0)]
        offset_mm: f64,
    },
    /// Tilt each solver axis once and measure which way the ball rolls.
    CalibAxes {
        #[arg(long, default_value_t = 3.0)]
        tilt_deg: f64,
    },
    /// Move M5, M6, M7 one at a time and check the model's joint signs against the ball's roll.
    ProbeJoints {
        #[arg(long, default_value_t = 35)]
        ticks: i32,
        /// Write the suggested sign changes to the config.
        #[arg(long)]
        apply: bool,
    },
    /// Rest -> level -> wait for ball -> balance. Ctrl-C returns to rest.
    Run {
        #[arg(long, default_value = "paddle_logs")]
        log_dir: PathBuf,
    },
}

#[cfg(target_os = "linux")]
#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    let cmd = cli.cmd.unwrap_or(Cmd::Tui { sim: false, log_dir: PathBuf::from("paddle_logs") });
    // Log lines would tear the full-screen app; keep the logger for the line-based commands only.
    if !matches!(cmd, Cmd::Tui { .. }) {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    }
    match cmd {
        Cmd::Tui { sim, log_dir } => tui::run(&cli.config, &log_dir, sim).await,
        Cmd::Scan { save_rest } => app::scan(&cli.config, save_rest).await,
        Cmd::Jog => app::jog(&cli.config).await,
        Cmd::Grip => app::grip(&cli.config).await,
        Cmd::VisionTest { seconds, snapshot } => app::vision_test(&cli.config, seconds, snapshot).await,
        Cmd::CalibCamera { offset_mm } => app::calib_camera(&cli.config, offset_mm).await,
        Cmd::CalibAxes { tilt_deg } => app::calib_axes(&cli.config, tilt_deg).await,
        Cmd::ProbeJoints { ticks, apply } => app::probe_joints(&cli.config, ticks, apply).await,
        Cmd::Run { log_dir } => app::run(&cli.config, &log_dir).await,
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    let _ = Cli::parse;
    eprintln!("paddle-balance needs Linux (V4L2 camera).");
    std::process::exit(1);
}
