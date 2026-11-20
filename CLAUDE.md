# CLAUDE.md: PingPongSimBot

Context for continuing this work in a new session. Read README.md for the architecture and the commands.

## What this is
A Rust program (`paddle-balance`) that balances a 40 mm, 2.7 g orange ping-pong ball on a 3D-printed paddle (Ø150 mm blade,
v2, face-to-camera layout) held in the ElRobot follower's gripper. It uses the U20CAM wrist camera and a PD on wrist joints M5–M7.
Tracking issue: spinworks-tech/RoboSim#69 (progress updates are posted there as comments). Paddle design: RoboSim#73.
Sibling repo: robomotic/ElRobotSysId (servo system ID for the MuJoCo model).

## Hardware and access
- **Pi:** NormaStation (Raspberry Pi 4, Debian 13, aarch64). SSH as `robomotic@normastation.local`. mDNS breaks when the Wi-Fi
  drops; then use the Pi's LAN IP (192.168.68.56 on 2026-11-19; may change). Passwordless sudo works.
- **Working copy on the Pi:** this code was first built in `~/paddle-ws` (a norma-core tree). The live `paddle.toml` (with all
  calibrations) and `paddle_logs/` are there. Clone this repo on the Pi and copy `paddle.toml` across before switching.
- **Station:** systemd unit `normacore-station` (not `station`). It owns both buses and the camera. Stop it before running
  anything here; start it again afterwards. Stopping it leaves servo torque on.
- **Buses (CH343, 1a86:55d3, 1 Mbps):** follower = serial `5B14033257` = `/dev/ttyACM1`
  (`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14033257-if00`); leader = `5B14112350` = `ttyACM0`.
  Follower servos run on 12 V.
- **Camera:** U20CAM (USB 0c45:6366), `/dev/video0`, YUYV 640×480 at about 29 fps. After the station stops, the camera has no
  V4L2 node until the USB device is re-authorized (`authorized` 0 → 1). The station's libuvc driver detaches `uvcvideo` and does
  not give it back. Settings: manual exposure 4 ms, auto white balance, 2 buffers.
- **Reading positions while the station runs:** `~/pqenv/bin/python tools/station_read_positions.py` on the Pi (read-only,
  through station_py in `~/ncpr`). This is how the level pose was captured after teleop.

## Key numbers (all measured)
- Level pose (paddle flat), ticks M1–M8: `2003 1002 1942 3185 1864 1892 2226 1440`. The user taught it; it's important.
- Grip: M8 goal 1396 (← closes), with the jaws at about 1436: 40 ticks of squeeze, load 17%, 117 mA.
- Web UI pitfall: the 4-digit zero-padded numbers are the calibration **Range Min** (M1 1087, M2 1004, M3 935, M5 141, M6 1040,
  M7 134, M8 1237), not positions.
- Camera map: ≈3.0 px/mm sideways, ≈0.66 px/mm near/far (blade seen at a slant).
- Axis directions repeat well; the magnitudes are unreliable (0.23–1.01 of the ideal), so `axes_normalize = true`.
- Joint signs match the URDF (`probe-joints`: M5 and M7 push the ball the same way). Don't flip signs because of the probe's
  "best fit" line when the pair angles are near 90°: such angles look the same either way.
- First run: longest balance 125.3 s; ball median 20 mm from the centre; ~1 Hz limit cycle (7 mm near/far, 2 mm sideways);
  software latency 2.9 ms; **servo lag 4–5 frames (~150 ms)**; M6 tracking error about 20 ticks.

## Current state and next steps
1. **Pending robot tests**, tracked as GitHub issues with the exact steps: **#1 test A** (`kd = 0.002`, already set on the Pi;
   sim: 22 → 9 mm) and **#2 test B** (`control.backlash_ticks = [4, 16, 6]`; sim: about 4.5 mm). The binary on the Pi
   (`~/paddle-ws`, built 2026-11-19 20:43) supports both; don't rebuild on the Pi unless the user asks. Compare with
   `tools/analyze_run.py`.
2. If near/far still wobbles more: separate PD gains per blade axis (not per servo).
3. The MuJoCo model (RoboSim #69 Phase 1) needs the servo parameters from ElRobotSysId.
4. Paddle weight is still unknown (also open in #73).

## Working agreements with the user
- **Never move the arm without the user's explicit OK** at the time, and say exactly which joints move and by how much. The
  user may be away. Read-only commands (`scan`, `vision-test`, reading positions) are fine.
- The user runs the interactive commands (`jog`, `grip`, `calib-*`, `probe-joints`, `run`) in their own SSH terminal and
  pastes the output.
- Prefer plain explanations with tables; the user likes visual TUIs over line prompts.
- Robot control work: say what will move before it moves.
- Build on the Pi with `cargo build --release` (incremental builds take about 30–110 s). Long builds: run them with
  `nohup … &` and an exit marker, because SSH drops with the Wi-Fi. Don't use `pgrep -f` over SSH: it matches its own command
  line.

## Full-screen app (in progress, 2026-11-20)
The user dislikes typing numbers and commands, so everything is moving into one ratatui app with the same keys on every screen
(see README "Full-screen app"). Done: **Balance** (live tuning, live stats), **Results** (compare runs), **Home** (station stop/start, camera re-attach,
read-only bus check). Screenshots in docs/screenshots (regenerate with tools/tui_drive.py `p:` steps; locally pyte lives in
the pyenv env `~/.pyenv/versions/antifake/bin/python3`).
Next, one at a time: Pose (replaces `jog`) → Calibration wizard (auto-capture
when the ball is still) → Camera (half-block live preview, HSV tuning) → Gripper (move grip_tui in) → System ID in ElRobotSysId.
- Engine: `balance::Balancer` is pure (frame in → action + telemetry out). `balance::run_loop` drives any `Plant`
  (`RealPlant`, `sim::SimPlant`) and reports `Event`s; the CLI `run` and the TUI share it.
- Test every screen in simulation: `paddle-balance tui --sim`, driven headless with `tools/tui_drive.py` (pyte). The user
  said not to rebuild on the Pi unless asked: develop and test locally in sim.
- Known gap: the simulator matches the real offset size but wobbles slower (0.27 Hz vs 0.95 Hz real). Refit it once the
  ElRobotSysId servo data exists.

## Code notes
- `src/main.rs` re-exports the library modules (`use paddle_balance::{bus, …}`) so that `crate::bus` paths in `app.rs` work.
- `vision::capture` and `app` are Linux-only (`cfg(target_os = "linux")`); other platforms build a stub binary.
- Tests: `cargo test` (25 tests, including `default_gains_balance_a_delayed_ball`, `controller_balances_the_simulated_ball`
  (the real engine on SimPlant), and `first_real_run_matches_the_python_analysis`).
- `analysis::lag_samples` correlates frame-to-frame changes (raw signals saturate on drift); it gives 5/5/4 frames on the real run.
- `bus.rs` is duplicated in ElRobotSysId. Keep fixes in sync, or extract a shared crate if it grows.
