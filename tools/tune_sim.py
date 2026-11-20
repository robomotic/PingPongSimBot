#!/usr/bin/env python3
"""2-axis ball-on-paddle simulation for tuning the paddle-balance controller against the real servo behaviour.

The plant was fitted to the first real run (data/run_1795119100.csv, 2026-11-19): with the original gains
(kp 0.0017, kd 0.0008, look-ahead 60 ms) it reproduces the real ~22 mm median ball offset and ~1 Hz limit cycle when:
  - 3 frames of loop delay + a first-order servo (tau 0.1 s)   -> ~150 ms goal->present lag, as measured
  - 0.03 rad (~1.7 deg) of servo slop (backlash / dead zone)   -> M6 tracking error ~20 ticks, as measured
  - 0.5 of the ideal ball acceleration (0.6 g theta)           -> from calib-axes / probe-joints
  - detection noise 0.3 mm sideways, 0.8 mm near/far           -> 3.0 vs 0.66 px/mm camera resolution
The controller copies src/control.rs + estimator.rs: alpha-beta filter, look-ahead, PD, clamp 10 deg, slew 60 deg/s,
and the per-axis slop compensation (control.backlash_ticks).

    python3 tools/tune_sim.py              # calibration check + grid search
    python3 tools/tune_sim.py --check      # only the calibration check
"""
import argparse

import numpy as np

G_BALL = 0.6 * 9810  # mm/s^2 per rad, hollow ball rolling without slip


def sim(kp, kd, pred, bc=0.0, eff=0.5, delay_frames=3, servo_tau=0.10, backlash=0.03, noise=(0.3, 0.8), T=40.0,
        seed=0, alpha=0.6, beta=0.2, fps=29.0):
    """Returns (median, p95) ball distance from centre (mm) after 10 s, or (1e3, 0) if the ball falls off."""
    rng = np.random.default_rng(seed)
    dt = 1 / fps
    p = np.array([30.0, -20.0])
    v = np.zeros(2)
    tilt = np.zeros(2)
    act = np.zeros(2)
    queue = [np.zeros(2)] * delay_frames
    fp, fv = None, np.zeros(2)
    out, prev, last_dir = np.zeros(2), np.zeros(2), np.zeros(2)
    maxt, rate = np.radians(10), np.radians(60) * dt
    rs = []
    for i in range(int(T / dt)):
        for _ in range(10):  # physics substeps
            v += act * G_BALL * eff * dt / 10
            p += v * dt / 10
        if np.hypot(*p) > 75:
            return 1e3, 0.0
        z = p + rng.normal(0, noise)
        acc = out * G_BALL * eff
        if fp is None:
            fp = z.copy()
        else:
            pr = fp + fv * dt + 0.5 * acc * dt * dt
            pv = fv + acc * dt
            res = z - pr
            fp, fv = pr + alpha * res, pv + beta / dt * res
        pp = fp + fv * pred + 0.5 * acc * pred ** 2
        vv = fv + acc * pred
        raw = -(kp * pp + kd * vv)
        n = np.hypot(*raw)
        if n > maxt:
            raw *= maxt / n
        d = raw - out
        n = np.hypot(*d)
        out = out + (d * rate / n if n > rate else d)
        dd = out - prev
        prev = out.copy()
        for k in range(2):
            if abs(dd[k]) > 1e-4:
                last_dir[k] = np.sign(dd[k])
        queue.append(out + last_dir * bc / 2)
        cmd = queue.pop(0)
        tilt += (cmd - tilt) * min(1.0, dt / servo_tau)
        for k in range(2):  # slop: the blade follows only outside the dead zone
            if tilt[k] - act[k] > backlash / 2:
                act[k] = tilt[k] - backlash / 2
            elif act[k] - tilt[k] > backlash / 2:
                act[k] = tilt[k] + backlash / 2
        if i * dt > 10:
            rs.append(np.hypot(*p))
    rs = np.array(rs)
    return float(np.median(rs)), float(np.percentile(rs, 95))


def ev(seeds=6, **kw):
    r = [sim(seed=s, **kw) for s in range(seeds)]
    return np.mean([a for a, _ in r]), np.mean([b for _, b in r])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()

    print("calibration check, original gains (real run: median ~20 mm):")
    print("  median %.1f mm, p95 %.1f mm" % ev(kp=0.0017, kd=0.0008, pred=0.06))
    if a.check:
        return
    res = []
    for kp in (0.0008, 0.0012, 0.0017):
        for kd in (0.0008, 0.0012, 0.0016, 0.002):
            for pred in (0.06, 0.15, 0.25):
                for bc in (0.0, 0.015, 0.03):
                    m, p95 = ev(kp=kp, kd=kd, pred=pred, bc=bc)
                    res.append((m, p95, kp, kd, pred, bc))
    res.sort()
    print("\nbest settings (slop comp in rad of blade tilt; 0.03 rad ~ 20 ticks on M6):")
    for r in res[:10]:
        print("  median %.1f p95 %.1f | kp %.4f kd %.4f predict %.2f s slop_comp %.3f" % r)


if __name__ == "__main__":
    main()
