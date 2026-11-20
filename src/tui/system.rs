//! The Pi around the robot: station service, camera attachment, and a read-only bus check.
//! Everything here runs in background tasks and reports back through a channel, so the UI never blocks.

use crate::bus::Bus;
use crate::config::Config;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;
use tokio::sync::mpsc::UnboundedSender;

#[derive(Debug, Clone)]
pub struct BusInfo {
    pub answering: Vec<u8>,
    pub voltage: f64,
    pub max_temp_c: u8,
    pub m8_load: i16,
    pub m8_torque: bool,
}

#[derive(Debug, Clone)]
pub enum SysMsg {
    /// Station active? (None: unknown, e.g. no systemd unit); camera V4L2 node present; camera USB device dir.
    Status { station: Option<bool>, camera_node: bool, camera_usb: Option<PathBuf> },
    Bus(Result<BusInfo, String>),
    /// A long action finished (or failed) with this message.
    Done(String),
    Failed(String),
}

fn run(cmd: &str, args: &[&str]) -> Result<String, String> {
    let out = Command::new(cmd).args(args).output().map_err(|e| format!("{cmd}: {e}"))?;
    if out.status.success() {
        Ok(String::from_utf8_lossy(&out.stdout).trim().to_string())
    } else {
        let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
        Err(if err.contains("password is required") || err.contains("a terminal is required") {
            format!("{cmd} {}: needs passwordless sudo", args.join(" "))
        } else {
            format!("{cmd} {}: {}", args.join(" "), if err.is_empty() { out.status.to_string() } else { err })
        })
    }
}

/// `systemctl is-active` → Some(true/false); None if the unit doesn't exist or systemctl is missing.
pub fn station_active(service: &str) -> Option<bool> {
    let out = Command::new("systemctl").args(["is-active", service]).output().ok()?;
    let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
    match s.as_str() {
        "active" | "activating" | "reloading" => Some(true),
        "inactive" | "failed" | "deactivating" => {
            // `inactive` is also what systemd says for a unit that doesn't exist; tell them apart.
            let load = Command::new("systemctl").args(["show", "-p", "LoadState", "--value", service]).output().ok()?;
            if String::from_utf8_lossy(&load.stdout).trim() == "not-found" {
                None
            } else {
                Some(false)
            }
        }
        _ => None,
    }
}

/// USB device directory (/sys/bus/usb/devices/X) of the first device with this vendor ID.
pub fn camera_usb_dir(vendor: &str) -> Option<PathBuf> {
    std::fs::read_dir("/sys/bus/usb/devices").ok()?.flatten().map(|e| e.path()).find(|p| {
        std::fs::read_to_string(p.join("idVendor")).is_ok_and(|v| v.trim().eq_ignore_ascii_case(vendor))
    })
}

pub fn status(cfg: &Config) -> SysMsg {
    SysMsg::Status {
        station: station_active(&cfg.system.station_service),
        camera_node: Path::new(&cfg.camera.device).exists(),
        camera_usb: camera_usb_dir(&cfg.system.camera_usb_vendor),
    }
}

fn set_station(service: &str, start: bool) -> Result<(), String> {
    run("sudo", &["-n", "systemctl", if start { "start" } else { "stop" }, service]).map(|_| ())
}

/// The station's libuvc driver keeps the camera detached from the kernel; re-authorizing the USB device brings
/// /dev/videoN back.
fn reattach_camera(dir: &Path) -> Result<(), String> {
    let a = dir.join("authorized");
    let a = a.to_string_lossy();
    run("sudo", &["-n", "sh", "-c", &format!("echo 0 > {a} && sleep 1 && echo 1 > {a}")]).map(|_| ())
}

/// Read-only bus check: ping M1..M8, read voltage, temperatures and M8 load. Sends no goals, changes no torque.
pub async fn bus_check(cfg: &Config) -> Result<BusInfo, String> {
    let mut bus = Bus::open(&cfg.bus.port, cfg.bus.timeout_ms).map_err(|e| format!("{}: {}", cfg.bus.port, e.root_cause()))?;
    let mut info = BusInfo { answering: Vec::new(), voltage: 0.0, max_temp_c: 0, m8_load: 0, m8_torque: false };
    for id in 1..=8u8 {
        if bus.ping(id).await {
            info.answering.push(id);
            if let Ok(s) = bus.read_state(id).await {
                info.max_temp_c = info.max_temp_c.max(s.temperature_c);
                if id == 1 {
                    info.voltage = s.voltage_dv as f64 / 10.0;
                }
                if id == 8 {
                    info.m8_load = s.load;
                }
            }
        }
    }
    info.m8_torque = bus.read_torque_enabled(8).await.unwrap_or(false);
    Ok(info)
}

/// Background actions. Each reports progress with Done/Failed and a fresh Status at the end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SysAction {
    Refresh,
    CheckBus,
    /// Stop the station, then re-attach the camera if needed, then check the bus.
    StopStation,
    StartStation,
    ReattachCamera,
}

pub fn spawn(action: SysAction, cfg: Config, tx: UnboundedSender<SysMsg>) {
    tokio::spawn(async move {
        let blocking = |cfg: Config, f: fn(&Config) -> Result<String, String>| async move {
            tokio::task::spawn_blocking(move || f(&cfg)).await.unwrap_or_else(|e| Err(e.to_string()))
        };
        let send = |m: SysMsg| {
            let _ = tx.send(m);
        };
        match action {
            SysAction::Refresh => {}
            SysAction::CheckBus => send(SysMsg::Bus(bus_check(&cfg).await)),
            SysAction::StopStation => {
                match blocking(cfg.clone(), |c| set_station(&c.system.station_service, false).map(|_| "station stopped".into())).await {
                    Ok(m) => {
                        send(SysMsg::Done(m));
                        tokio::time::sleep(Duration::from_secs(2)).await;
                        let st = status(&cfg);
                        if let SysMsg::Status { camera_node: false, camera_usb: Some(_), .. } = &st {
                            match blocking(cfg.clone(), |c| {
                                let d = camera_usb_dir(&c.system.camera_usb_vendor).ok_or("camera not on USB")?;
                                reattach_camera(&d).map(|_| "camera re-attached".into())
                            })
                            .await
                            {
                                Ok(m) => send(SysMsg::Done(m)),
                                Err(e) => send(SysMsg::Failed(e)),
                            }
                            tokio::time::sleep(Duration::from_secs(2)).await;
                        }
                        send(SysMsg::Bus(bus_check(&cfg).await));
                    }
                    Err(e) => send(SysMsg::Failed(e)),
                }
            }
            SysAction::StartStation => {
                match blocking(cfg.clone(), |c| set_station(&c.system.station_service, true).map(|_| "station started".into())).await {
                    Ok(m) => send(SysMsg::Done(m)),
                    Err(e) => send(SysMsg::Failed(e)),
                }
                tokio::time::sleep(Duration::from_secs(2)).await;
            }
            SysAction::ReattachCamera => {
                match blocking(cfg.clone(), |c| {
                    let d = camera_usb_dir(&c.system.camera_usb_vendor).ok_or("camera not found on USB")?;
                    reattach_camera(&d).map(|_| "camera re-attached".into())
                })
                .await
                {
                    Ok(m) => send(SysMsg::Done(m)),
                    Err(e) => send(SysMsg::Failed(e)),
                }
                tokio::time::sleep(Duration::from_secs(2)).await;
            }
        }
        let st = tokio::task::spawn_blocking(move || status(&cfg)).await;
        if let Ok(st) = st {
            send(st);
        }
    });
}
