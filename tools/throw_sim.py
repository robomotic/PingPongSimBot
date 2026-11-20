#!/usr/bin/env python3
"""MuJoCo model of the throw ("throw back") motion: ElRobot follower + 68 g paddle + 2.7 g ball.

Nothing here talks to the robot. It builds a scene from the norma-core URDF, plays a sequence of poses (in servo ticks)
through a model of the ST3215's own speed/acceleration profile, and reports where the ball goes.

Joint angles use the same mapping as the station's 3D viewer (checked against the real arm at level, 2026-11-20):
    angle = lower + (ticks - range_min) / (range_max - range_min) * (upper - lower)
with the URDF limits and the station's calibrated ranges (follower 5B14033257). The controller's own
`zero = 2048, sign = -1` mapping is only right for small relative moves; it puts the arm under the table.

    python tools/throw_sim.py --check            # ball resting on the paddle at level
    python tools/throw_sim.py --render out.png   # still image of the level pose
"""
import argparse
import os
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

import mujoco

URDF = os.path.expanduser("~/DevOps/norma-core/hardware/elrobot/simulation/elrobot_follower.urdf")

# Station calibration, follower bus 5B14033257, read through station_py on 2026-11-20.
RANGE_MIN = [1087, 1004, 935, 900, 141, 1040, 134]
RANGE_MAX = [3009, 3092, 3161, 3196, 3955, 3056, 3962]
LEVEL = [2003, 1002, 1942, 3185, 1864, 1892, 2226]

# ST3215 at 12 V (URDF limits): 2.94 N m stall, 4.71 rad/s no-load.
TORQUE_NM = 2.94
VMAX_RAD_S = 4.71

# Paddle v2 (RoboSim #73), in the gripper frame (y = along the fingers, x = jaw closing direction).
PADDLE_MASS = 0.068
BLADE_R = 0.075
BLADE_T = 0.0025
HANDLE_T = 0.008      # between the jaws
FINGER_TIP_Y = 0.104  # from the jaw meshes
NECK = 0.015
BALL_R = 0.020
BALL_MASS = 0.0027
CAMERA_POS = (0.0, 0.03, 0.055)  # rough: "about 55 mm off the jaw axis, looking along the fingers"


def ticks_to_q(model, ticks):
    q = []
    for i, t in enumerate(ticks[:7]):
        lo, hi = model.jnt_range[model.joint(f"rev_motor_0{i + 1}").id]
        p = (t - RANGE_MIN[i]) / (RANGE_MAX[i] - RANGE_MIN[i])
        q.append(lo + min(1.0, max(0.0, p)) * (hi - lo))
    return np.array(q)


def q_to_ticks(model, q):
    out = []
    for i, a in enumerate(q):
        lo, hi = model.jnt_range[model.joint(f"rev_motor_0{i + 1}").id]
        out.append(RANGE_MIN[i] + (a - lo) / (hi - lo) * (RANGE_MAX[i] - RANGE_MIN[i]))
    return np.array(out)


def _raw_model():
    u = open(URDF).read().replace('filename="assets/', 'filename="')
    u = u.replace(
        '<robot name="Elrobot" >',
        '<robot name="Elrobot"><mujoco><compiler meshdir="%s" balanceinertia="true" boundmass="0.001" '
        'boundinertia="1e-8" discardvisual="false" fusestatic="false"/></mujoco>' % os.path.join(os.path.dirname(URDF), "assets"),
        1,
    )
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(u)
    m = mujoco.MjModel.from_xml_path(f.name)
    os.unlink(f.name)
    return m


def _blade_normal_in_gripper(m):
    """World 'up' seen from the gripper at the level pose: the blade normal (level is level by definition)."""
    d = mujoco.MjData(m)
    for i, a in enumerate(ticks_to_q(m, LEVEL)):
        d.qpos[m.joint(f"rev_motor_0{i + 1}").qposadr[0]] = a
    mujoco.mj_forward(m, d)
    R = d.xmat[m.body("Gripper_Base_v1_1").id].reshape(3, 3)
    return R.T @ np.array([0.0, 0.0, 1.0])


def build(kp=40.0, kv=0.6):
    """Returns (model, info). Position actuators on M1..M7 with the stall torque as force limit."""
    raw = _raw_model()
    n = _blade_normal_in_gripper(raw)
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "raw.xml")
        mujoco.mj_saveLastXML(path, raw)
        tree = ET.parse(path)
    root = tree.getroot()
    comp = root.find("compiler")
    comp.set("meshdir", os.path.join(os.path.dirname(URDF), "assets"))
    comp.set("angle", "radian")
    opt = ET.SubElement(root, "option", timestep="0.0005", integrator="implicitfast")
    ET.SubElement(opt, "flag", warmstart="enable")
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", offwidth="1280", offheight="960")
    asset = root.find("asset") if root.find("asset") is not None else ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1=".85 .85 .85", rgb2=".75 .75 .75", width="512", height="512")
    ET.SubElement(asset, "material", name="grid", texture="grid", texrepeat="20 20")
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="0 0.3 1.5", dir="0 0 -1", diffuse=".8 .8 .8")
    ET.SubElement(world, "geom", name="table", type="plane", size="1 1 0.01", material="grid", contype="2", conaffinity="2")

    # Arm meshes: collide with the ball (and the table) only, not with each other.
    for g in root.iter("geom"):
        if g.get("name") != "table":
            g.set("contype", "4")
            g.set("conaffinity", "1")

    # Paddle, welded to the gripper base. Blade frame: z = blade normal n, y = handle direction in the blade plane.
    gb = next(b for b in root.iter("body") if b.get("name") == "Gripper_Base_v1_1")
    y = np.array([0.0, 1.0, 0.0])
    y = y - n * (y @ n)
    y /= np.linalg.norm(y)
    x = np.cross(y, n)
    Rb = np.column_stack([x, y, n])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, Rb.flatten())
    top = HANDLE_T / 2  # blade flush with the handle's top face; the ball rolls on it
    blade_c = np.array([0.0, FINGER_TIP_Y + NECK + BLADE_R, 0.0])
    blade_c = blade_c - n * (blade_c @ n) + n * (top - BLADE_T / 2)
    paddle = ET.SubElement(gb, "body", name="paddle", pos=" ".join(f"{v:.5f}" for v in blade_c), quat=" ".join(f"{v:.6f}" for v in quat))
    blade_mass = PADDLE_MASS * 0.8
    ET.SubElement(paddle, "geom", name="blade", type="cylinder", size=f"{BLADE_R} {BLADE_T / 2}", mass=f"{blade_mass}",
                  rgba=".1 .1 .1 1", contype="1", conaffinity="1", condim="6", friction="0.6 0.005 0.0005",
                  solref="0.002 1", solimp="0.95 0.99 0.001")
    handle_len = FINGER_TIP_Y + NECK - 0.045
    ET.SubElement(paddle, "geom", name="handle", type="box",
                  size=f"0.008 {handle_len / 2} {HANDLE_T / 2}", pos=f"0 {-(BLADE_R + handle_len / 2)} {-(HANDLE_T - BLADE_T) / 2}",
                  mass=f"{PADDLE_MASS - blade_mass}", rgba=".15 .15 .15 1", contype="0", conaffinity="0")
    ET.SubElement(gb, "geom", name="camera", type="box", size="0.016 0.012 0.016", pos=" ".join(map(str, CAMERA_POS)),
                  mass="0.02", rgba=".2 .4 .8 1", contype="4", conaffinity="1")
    ET.SubElement(paddle, "site", name="blade_centre", pos=f"0 0 {BLADE_T / 2}", size="0.004", rgba="1 0 0 1")

    ball = ET.SubElement(world, "body", name="ball", pos="0 0 1")
    ET.SubElement(ball, "freejoint", name="ball")
    I = 2.0 / 3.0 * BALL_MASS * BALL_R**2  # thin shell
    ET.SubElement(ball, "inertial", pos="0 0 0", mass=f"{BALL_MASS}", diaginertia=f"{I} {I} {I}")
    ET.SubElement(ball, "geom", name="ball", type="sphere", size=f"{BALL_R}", rgba="1 .5 0 1", contype="1", conaffinity="7",
                  condim="6", friction="0.6 0.005 0.0005", solref="0.002 1", solimp="0.95 0.99 0.001")

    act = ET.SubElement(root, "actuator")
    for i in range(7):
        ET.SubElement(act, "position", name=f"M{i + 1}", joint=f"rev_motor_0{i + 1}", kp=f"{kp}", kv=f"{kv}",
                      forcerange=f"{-TORQUE_NM} {TORQUE_NM}", forcelimited="true")
    for j in root.iter("joint"):
        if j.get("name", "").startswith("rev_motor_0"):
            j.set("armature", "0.002")
            j.set("damping", "0.05")
            j.set("frictionloss", "0.02")

    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return m, {"blade_normal_gripper": n}


class Sim:
    def __init__(self, **kw):
        self.m, self.info = build(**kw)
        self.d = mujoco.MjData(self.m)
        self.qadr = [self.m.joint(f"rev_motor_0{i + 1}").qposadr[0] for i in range(7)]
        self.vadr = [self.m.joint(f"rev_motor_0{i + 1}").dofadr[0] for i in range(7)]
        self.ball_q = self.m.joint("ball").qposadr[0]
        self.ball_v = self.m.joint("ball").dofadr[0]

    def set_pose(self, ticks):
        q = ticks_to_q(self.m, ticks)
        for a, v in zip(self.qadr, q):
            self.d.qpos[a] = v
        self.d.ctrl[:7] = q
        mujoco.mj_forward(self.m, self.d)

    def place_ball(self, dx=0.0, dy=0.0):
        sid = self.m.site("blade_centre").id
        R = self.d.site_xmat[sid].reshape(3, 3)
        p = self.d.site_xpos[sid] + R @ np.array([dx, dy, BALL_R + 0.0005])
        self.d.qpos[self.ball_q:self.ball_q + 3] = p
        self.d.qpos[self.ball_q + 3:self.ball_q + 7] = [1, 0, 0, 0]
        self.d.qvel[self.ball_v:self.ball_v + 6] = 0
        mujoco.mj_forward(self.m, self.d)

    def ball_in_blade(self):
        sid = self.m.site("blade_centre").id
        R = self.d.site_xmat[sid].reshape(3, 3)
        return R.T @ (self.d.qpos[self.ball_q:self.ball_q + 3] - self.d.site_xpos[sid])


def render(sim, path, azimuth=180, elevation=-12, distance=0.75, lookat=(0, 0.12, 0.1)):
    from PIL import Image
    r = mujoco.Renderer(sim.m, 480, 640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    r.update_scene(sim.d, cam)
    Image.fromarray(r.render()).save(path)


def check_rest(seconds=0.5):
    sim = Sim()
    sim.set_pose(LEVEL)
    for _ in range(int(1.0 / sim.m.opt.timestep)):  # let the joints settle under the paddle's weight first
        mujoco.mj_step(sim.m, sim.d)
    sim.place_ball()
    start = sim.ball_in_blade()
    steps = int(seconds / sim.m.opt.timestep)
    worst = 0.0
    for _ in range(steps):
        mujoco.mj_step(sim.m, sim.d)
        worst = max(worst, np.abs(sim.d.qpos[sim.qadr] - sim.d.ctrl[:7]).max())
    end = sim.ball_in_blade()
    print(f"ball on blade: start {np.round(start * 1000, 1)} mm, after {seconds:.1f} s {np.round(end * 1000, 1)} mm")
    print(f"max joint sag under gravity: {np.degrees(worst):.2f} deg")
    print(f"paddle mass {PADDLE_MASS * 1000:.0f} g; blade normal in gripper frame {np.round(sim.info['blade_normal_gripper'], 3)}")
    return sim


RAD_PER_TICK = 2 * np.pi / 4096
ACC_UNIT = 100 * RAD_PER_TICK  # ST3215 Acc register: 100 steps/s^2 per unit (max 254)
# End pose taught by hand on the real arm, 2026-11-20 (blade ~78 deg, face toward the user behind the camera).
TAUGHT_END = [1999, 1015, 1763, 3131, 1880, 1237, 2220]


class Profile:
    """The ST3215's own trapezoidal move: goal, speed limit (rad/s), acceleration limit (rad/s^2), per joint.
    All joints get speeds and accelerations scaled by their distance, so they start and arrive together."""

    def __init__(self, start_q, goal_q, vmax, amax):
        self.p = np.array(start_q, float)
        self.v = np.zeros_like(self.p)
        self.goal = np.array(goal_q, float)
        dist = np.abs(self.goal - self.p)
        k = dist / max(dist.max(), 1e-9)
        self.vmax = np.maximum(vmax * k, 1e-6)
        self.amax = np.maximum(amax * k, 1e-6)

    def step(self, dt):
        for i in range(len(self.p)):
            e = self.goal[i] - self.p[i]
            v = self.v[i]
            stop = v * v / (2 * self.amax[i])  # distance needed to stop
            if abs(e) < 1e-7 and abs(v) < 1e-5:
                self.v[i] = 0.0
                continue
            want = np.sign(e) * self.vmax[i] if abs(e) > stop else 0.0
            dv = np.clip(want - v, -self.amax[i] * dt, self.amax[i] * dt)
            v += dv
            if np.sign(e) != np.sign(e - v * dt) and abs(e) < abs(v * dt) + 1e-9:  # would overshoot
                self.p[i] = self.goal[i]
                self.v[i] = 0.0
                continue
            self.v[i] = v
            self.p[i] += v * dt
        return self.p


def throw(end_ticks, speed=1.0, acc_reg=254, hold_s=1.5, video=None, fps=60, slow=1.0):
    """Level + ball at the centre -> one fast move to end_ticks -> hold. speed scales the servo speed limit
    (1.0 = 4.71 rad/s); slow motion is speed s with acceleration s^2 (same path, longer)."""
    sim = Sim()
    m, d = sim.m, sim.d
    sim.set_pose(LEVEL)
    for _ in range(int(0.6 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    sim.place_ball()
    q0 = ticks_to_q(m, LEVEL)
    q1 = ticks_to_q(m, end_ticks)
    prof = Profile(q0, q1, VMAX_RAD_S * speed, acc_reg * ACC_UNIT * speed**2)
    ball_geom = m.geom("ball").id
    arm_bodies = {m.body(n).id for n in ("Gripper_Base_v1_1", "ST3215_7_v1_1", "Joint_06_v1_1", "ST3215_8_v1_1",
                                          "Gripper_Jaw_01_v1_1", "Gripper_Jaw_02_v1_1")}
    blade = m.geom("blade").id
    table = m.geom("table").id
    out = {"release_t": None, "hit_arm": None, "landed": None, "apex": -1.0, "peak_torque": np.zeros(7),
           "rolled_mm": 0.0, "move_s": None}
    frames = []
    renderer = None
    if video:
        renderer = mujoco.Renderer(m, 480, 640)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (0, -0.1, 0.28)
        cam.distance, cam.azimuth, cam.elevation = 1.25, 180, -6
    dt = m.opt.timestep
    n = int((hold_s + 2.0 / max(speed, 0.1)) / dt)
    next_frame = 0.0
    on_blade_prev = True
    for k in range(n):
        t = k * dt
        d.ctrl[:7] = prof.step(dt)
        # DC motor: torque available falls linearly with speed (stall torque at rest, none at no-load speed).
        w = np.abs(d.qvel[sim.vadr])
        lim = TORQUE_NM * np.clip(1.0 - w / VMAX_RAD_S, 0.05, 1.0)
        m.actuator_forcerange[:7, 0] = -lim
        m.actuator_forcerange[:7, 1] = lim
        if out["move_s"] is None and np.allclose(prof.p, prof.goal) and not prof.v.any():
            out["move_s"] = t
        mujoco.mj_step(m, d)
        out["peak_torque"] = np.maximum(out["peak_torque"], np.abs(d.actuator_force[:7]))
        touching_blade = False
        for c in d.contact[:d.ncon]:
            pair = {c.geom1, c.geom2}
            if ball_geom in pair:
                other = (pair - {ball_geom}).pop()
                if other == blade:
                    touching_blade = True
                elif other == table and out["landed"] is None:
                    out["landed"] = d.qpos[sim.ball_q:sim.ball_q + 3].copy()
                elif m.geom_bodyid[other] in arm_bodies or m.geom(other).name == "camera":
                    if out["hit_arm"] is None:
                        out["hit_arm"] = (t, m.body(m.geom_bodyid[other]).name if m.geom(other).name != "camera" else "camera")
        if on_blade_prev and not touching_blade and out["release_t"] is None and t > 0.002:
            out["release_t"] = t
            out["release_v"] = d.qvel[sim.ball_v:sim.ball_v + 3].copy()
            out["release_p"] = d.qpos[sim.ball_q:sim.ball_q + 3].copy()
            out["release_in_blade"] = sim.ball_in_blade().copy()
        if touching_blade and out["release_t"] is not None and out["landed"] is None:
            out["release_t"] = None  # it bounced back onto the blade: wait for the real release
        on_blade_prev = touching_blade
        if out["release_t"] is not None:
            out["apex"] = max(out["apex"], d.qpos[sim.ball_q + 2])
        if renderer is not None and t >= next_frame:
            renderer.update_scene(d, cam)
            frames.append(renderer.render())
            next_frame += 1.0 / (fps * slow)
        if out["landed"] is not None and t > (out["move_s"] or 0) + 0.3:
            break
    if video and frames:
        import imageio
        imageio.mimsave(video, frames, fps=fps)
    return out


def report(name, o):
    lines = [f"{name}:"]
    if o["move_s"] is not None:
        lines.append(f"  move takes {o['move_s']:.2f} s")
    if o["release_t"] is None:
        lines.append("  ball never left the paddle")
    else:
        v = o["release_v"]
        sp = np.linalg.norm(v)
        elev = np.degrees(np.arctan2(v[2], np.hypot(v[0], v[1])))
        lines.append(f"  release at {o['release_t']:.3f} s, speed {sp:.2f} m/s, {elev:.0f} deg up, "
                     f"heading {'toward you (-y)' if v[1] < 0 else 'away (+y)'}; ball was {np.round(o['release_in_blade'][:2] * 1000)} mm from centre")
        lines.append(f"  apex {o['apex'] * 100:.0f} cm above the table")
        if o["landed"] is not None:
            lines.append(f"  lands at y = {o['landed'][1] * 100:.0f} cm (base at 0; negative = your side), x = {o['landed'][0] * 100:.0f} cm")
    if o["hit_arm"]:
        lines.append(f"  !! ball hits the {o['hit_arm'][1]} at {o['hit_arm'][0]:.2f} s")
    lines.append("  peak torque N m: " + " ".join(f"M{i + 1} {x:.2f}" for i, x in enumerate(o["peak_torque"])) +
                 ("  (at the stall limit)" if (o["peak_torque"] > TORQUE_NM * 0.98).any() else ""))
    print("\n".join(lines))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--render")
    ap.add_argument("--throw", action="store_true", help="simulate the throw to the taught end pose")
    ap.add_argument("--end", help="end pose ticks M1..M7, comma separated (default: the taught one)")
    ap.add_argument("--speed", type=float, nargs="+", default=[0.25, 0.5, 0.75, 1.0])
    ap.add_argument("--acc", type=int, default=254, help="ST3215 Acc register (1..254)")
    ap.add_argument("--video", help="write an mp4/gif of the last speed")
    ap.add_argument("--slow", type=float, default=1.0, help="video slow-motion factor (4 = 4x slower)")
    a = ap.parse_args()
    if a.check or a.render:
        s = check_rest()
        if a.render:
            render(s, a.render)
            print("wrote", a.render)
    if a.throw:
        end = [int(x) for x in a.end.split(",")] if a.end else TAUGHT_END
        for i, sp in enumerate(a.speed):
            o = throw(end, speed=sp, acc_reg=a.acc, video=a.video if i == len(a.speed) - 1 else None, slow=a.slow)
            report(f"speed {sp:.2f}, acc {a.acc}", o)
