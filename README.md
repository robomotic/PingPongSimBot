# PingPongSimBot

Balances an orange 40 mm ping-pong ball on a 3D-printed paddle held by the **ElRobot** follower arm. It uses the wrist camera
and a PID on the three wrist joints, and runs on the Raspberry Pi 4 of a NormaCore station.

It is part of [RoboSim #69](https://github.com/spinworks-tech/RoboSim/issues/69) (ping-pong sim2real via PID); the paddle
design is [#73](https://github.com/spinworks-tech/RoboSim/issues/73). The servo identification for the MuJoCo model is in
[ElRobotSysId](https://github.com/robomotic/ElRobotSysId).

**Status (2026-11-19):** the first real run balanced for **125 s**, with a mean wrist current of 3–13 mA. A ~1 Hz limit cycle,
caused by servo lag and slop, keeps the median ball offset at about 20 mm. A fix (more damping and slop compensation) is
implemented and waiting for a test on the robot. See [docs/2026-11-19-first-real-balance.md](docs/2026-11-19-first-real-balance.md).

```
camera (V4L2 YUYV 640x480, ~29 fps) -> HSV threshold + largest blob (pure Rust, half resolution, blade ROI)
  -> homography px -> blade mm -> alpha-beta filter + delay look-ahead
  -> PD(+I) per blade axis with deadband, clamp and slew -> blade tilt
  -> measured axes map (directions) -> weighted damped pseudo-inverse over M5/M6/M7 + null-space pull to level
  -> per-joint slop compensation -> sync write of 3 goal positions on the follower bus (1 Mbps)
```

## Why it talks to the bus directly

Going through the NormaCore station adds about 160 ms of delay and stalls of up to 580 ms
([norma-core#185](https://github.com/norma-core/norma-core/issues/185)). This program owns the follower's serial port, so frame →
goal written takes about 3 ms. The station must be stopped while it runs. Servo torque stays on when the station stops, so the arm
holds its pose.

## Using 3 wrist joints with little effort

The wrist is roll-pitch-roll (M5, M6, M7) and the ball needs only 2 tilt angles. A weighted, damped pseudo-inverse uses the joints
that cost least: M5 moves the most mass, so it has the highest weight. A null-space term pulls the spare direction back to the
level pose.

Other measures that save energy:
- no new goal while the ball is within 4 mm of the centre and nearly still;
- goals that change by under 2 ticks are skipped;
- moderate servo acceleration;
- M1–M4 hold still.

Each run prints the mean wrist current.

## Build (on the Pi)

```bash
curl https://sh.rustup.rs -sSf | sh -s -- -y --profile minimal
sudo apt install protobuf-compiler libclang-dev libudev-dev   # st3215 build.rs, v4l bindgen, serial
cargo build --release            # first build ~5 min on a Pi 4
cargo test                       # 21 unit tests incl. a delayed closed-loop sim
```

`st3215` (the ST3215 packet codec) comes from [norma-core](https://github.com/norma-core/norma-core) as a pinned git dependency.

## Session on the robot

```bash
sudo systemctl stop normacore-station          # frees the follower bus and the camera
# The station's libuvc driver keeps the camera detached from the kernel; give it back:
D=$(grep -l 0c45 /sys/bus/usb/devices/*/idVendor | xargs dirname); echo 0 | sudo tee $D/authorized; echo 1 | sudo tee $D/authorized
cp config/paddle.example.toml paddle.toml       # or start empty: every command fills in its part

B=target/release/paddle-balance
$B scan                      # 8 servos answer? torque on? (no motion)
$B jog                       # teach poses: 'goto level', '<motor> <+/-ticks>', 'save level' / 'save rest'
$B grip                      # full-screen: arrows move the gripper, live load bar, 's' saves
$B vision-test --snapshot f.ppm
$B calib-camera --offset-mm 40   # ball on 5 marks: centre and +/-40 mm on two perpendicular lines
$B calib-axes                # 3 deg tilt per solver axis, measures where the ball rolls
$B probe-joints              # one joint at a time; checks the joint signs against the model
$B run                       # rest -> level -> place ball -> balance; Ctrl-C -> back to the start pose

sudo systemctl start normacore-station
```

All settings and calibrations live in `paddle.toml`. Each run writes `paddle_logs/run_<stamp>.csv`.

## Tools

| | |
|---|---|
| `tools/analyze_run.py <csv>` | balance segments, ball offset, limit-cycle frequency and amplitude, servo lag, latency, deadband use |
| `tools/tune_sim.py` | 2-axis sim fitted to the real run (delay, servo lag, slop, noise); grid-searches gains |
| `tools/station_read_positions.py` | read all servo positions through a running station (read-only), e.g. after teleop |
| `data/run_1795119100.csv` | the first real run (125 s balance) |

## Safety

- `run` refuses to start if any joint is more than 150 ticks from the saved rest pose.
- M5–M7 are clamped to ±250 ticks around level, and the tilt to ±10° (slew 60°/s).
- Ball lost for 0.5 s, or more than 65 mm out → back to level and wait for the ball.
- No camera frame for 1 s, or 3 failed servo reads in a row → stop and return.
- Ctrl-C, and any error, always ends with a smooth move back to the start pose.
- M8 (gripper) is written only by `grip`, and at startup to re-apply the saved grip (only if within 60 ticks of where it is).
  It never moves while balancing.

## Layout

```
src/bus.rs         ST3215 bus over tokio-serial (reads, sync writes, hold-in-place, grip)
src/config.rs      paddle.toml schema and defaults
src/vision.rs      YUYV -> HSV -> blob detector, V4L2 capture thread, PPM snapshots
src/estimator.rs   homography (DLT), alpha-beta filter
src/kinematics.rs  orientation FK from the URDF axes, weighted damped pseudo-inverse with null space
src/control.rs     ball PID, axes map, slop compensation, acceleration fit, min-jerk
src/app.rs         subcommands and the run state machine
src/grip_tui.rs    gripper adjuster (crossterm)
```
