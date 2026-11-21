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
read-only bus check), **Pose** (joints + gripper; replaces jog and grip; safety.pose_limit_ticks box around level), **Camera** (live
preview, detector view, live HSV/exposure tuning), **Calibrate** (camera-map wizard with auto-capture). Screenshots in docs/screenshots (regenerate with tools/tui_drive.py `p:` steps; locally pyte lives in
the pyenv env `~/.pyenv/versions/antifake/bin/python3`).
Axis calibration is in Calibrate too (`t`, ARMED; engine in src/calib.rs, runs on any Plant). Next: joint probe in
Calibrate (optional) → System ID in ElRobotSysId
(needs the table safety area first: ElRobotSysId issue #1). Nothing new has run on the real arm yet: first real session
should walk through every screen with the user watching.
- Camera: `vision::Camera` handle (real V4L2 thread or `start_sim_camera`), `Tuning` shared with the capture thread
  (versioned), low-res `Preview` only while a screen asks. The simulated camera renders blade + ball through the inverse
  map and runs the real detector (flat-disc perspective bias ~2.5 mm near/far).
- Pose uses `tui/device.rs`: a background task that owns the arm (real `Arm` or `SimArm`) and streams all 8 servo states at
  10 Hz. Leaving the screen drops it and releases the bus. Pose and Balance never own the bus at the same time.
- Engine: `balance::Balancer` is pure (frame in → action + telemetry out). `balance::run_loop` drives any `Plant`
  (`RealPlant`, `sim::SimPlant`) and reports `Event`s; the CLI `run` and the TUI share it.
- Test every screen in simulation: `paddle-balance tui --sim`, driven headless with `tools/tui_drive.py` (pyte). The user
  said not to rebuild on the Pi unless asked: develop and test locally in sim.
- Known gap: the simulator matches the real offset size but wobbles slower (0.27 Hz vs 0.95 Hz real). Refit it once the
  ElRobotSysId servo data exists.

## Throw back (2026-11-20, built, not yet run on the real arm)
Balance `t` (ARMED, 2 s countdown, throw when the ball is within 25 mm), Pose `t` preview 10/25/50/100 % without the ball,
`T` saves `throw.end`. Engine: `throw.rs` (plan: per-joint Goal Speed/Acc scaled by distance), `Arm::throw_move` (restores
the profiles; aborts on stall), `Plant::throw`. MuJoCo model: `tools/throw_sim.py` (run with
`MUJOCO_GL=glfw ~/.pyenv/versions/antifake/bin/python3`). Full speed: ball 2.0 m/s, apex 55 cm, lands 50 cm toward the
camera side; <75 % hits the camera. **Absolute joint angles**: use the station viewer mapping (angle = lower + (tick −
range_min)/(range_max − range_min)·(upper − lower), follower ranges in throw_sim.py), not `joints.zero/sign`, which puts
the arm under the table. At level M2 and M4 sit at the ends of their calibrated ranges. Station calibration store is
encrypted; read the ranges through station_py (MotorState.get_range_min/max) while the station runs.
First real session: previews at 10 → 100 % with the user watching the grip, then a ball throw at 100 %.
**Follower servo registers (read 2026-11-21, all M1–M8):** EEPROM Max Torque 500 and RAM Torque Limit 500 (= 50 %),
P 16 / D 0 / I 0 (Feetech default P 32), protection current 500, overload 25 % for 2 s → 20 %. The 50 % cap halved the
first real throws (load flat at 50 %, move 0.58 s vs 0.25 s planned); the throw now raises the RAM Torque Limit of the
moving joints to `throw.torque_limit` (1000) and restores it. P 16 probably also explains the ~150 ms servo lag and
M6's tracking error while balancing; changing it means an EEPROM write (persistent, affects the station): ask first.
Read-only register dump: a READ-packet script with `~/lrenv/bin/python` (pyserial), app and station stopped.

## State at the end of 2026-11-21 (next session starts here)
- **Recalibrate first**: the grip went 1396 → 1360 (M8 overload, 55 °C) → 1432 today, so the paddle has probably moved in
  the jaws. Redo the camera map (Calibrate; Space captures by hand) and then the axes (`t`). The user re-taught level
  (now M3 2057, M4 3174, M6 1847) and re-taught the throw end for it (M3 −300, M4 −300, M6 −400 from level).
- **50 Hz flicker** is real (still-ball blob area oscillates at 11.6 Hz = 100 Hz aliased at 29.5 fps). Exposure is now
  100 (10 ms, saved); keep exposure a multiple of 10 ms. Re-check the HSV limits in the Camera screen after recalibrating.
- **Throws get cancelled** when the stuck ball rests outside `throw.release_mm` (8 mm) of the aim: last run it sat 12 mm
  away. Offered: release_mm 15, and showing the ball's distance to the aim in the countdown title. The ball looks "on" the
  aim on screen because the drawn ball is 40 mm wide. User has not decided; slop compensation was 0 (profile 1).
- `rest` still has M8 = 0 (saved while M8 was in overload): re-save R. Servos screen (P gain) not yet used on the arm.
- Wi-Fi drops kill the app with the SSH session: suggest `tmux` (not installed on the Pi).
- PRs: #3 (tested settings) and #4 (throw, stacked on #3, now also P gain screen, servo-fault reads) are open; the Pi
  runs branch `throw-back`.

## Code notes
- `src/main.rs` re-exports the library modules (`use paddle_balance::{bus, …}`) so that `crate::bus` paths in `app.rs` work.
- `vision::capture` and `app` are Linux-only (`cfg(target_os = "linux")`); other platforms build a stub binary.
- Tests: `cargo test` (31 tests, including the throw planner in `throw.rs`, including `default_gains_balance_a_delayed_ball`, `controller_balances_the_simulated_ball`
  (the real engine on SimPlant), and `first_real_run_matches_the_python_analysis`).
- `analysis::lag_samples` correlates frame-to-frame changes (raw signals saturate on drift); it gives 5/5/4 frames on the real run.
- `bus.rs` is duplicated in ElRobotSysId. Keep fixes in sync, or extract a shared crate if it grows.

## Releases
v0.1.0 (2026-11-20): aarch64 binary built on the Pi, stripped, `.sha256` attached, tag at the built commit.
To repeat: on the Pi `bash ~/pp-build.sh` (pulls and builds both repos, log in ~/pp-build.log), then `strip` a copy, run
`sha256sum`, scp it down, and `gh release create vX.Y.Z --target <full sha> --notes-file ... <binary> <binary>.sha256`.
Check the minimum glibc with `objdump -T <bin> | grep -o 'GLIBC_[0-9.]*' | sort -V | tail -1`.
