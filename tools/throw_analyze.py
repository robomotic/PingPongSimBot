#!/usr/bin/env python3
"""Summarise a throw log (paddle_logs/throw_<stamp>_<n>.csv, written by the Balance screen's throw).

    python3 tools/throw_analyze.py data/throw_<stamp>_1.csv [more logs ...]

Per moving joint: travel, time to reach the goal (within 20 ticks), peak speed (deg/s) and peak load; then the ball as
the wrist camera saw it (blade mm, valid while it is on the blade) and when it was last seen. Compare with the MuJoCo
prediction from tools/throw_sim.py (full speed: move 0.25 s, M6 peak ~270 deg/s, ball leaves ~0.18 s).
"""
import csv
import math
import sys

DEG_PER_TICK = 360.0 / 4096


def analyse(path):
    rows = list(csv.DictReader(open(path)))
    servo = [r for r in rows if r["src"] == "servo"]
    cam = [r for r in rows if r["src"] == "cam"]
    print(f"{path}: {len(servo)} servo reads ({len(servo) / max(float(servo[-1]['t']), 1e-3):.0f}/s), {len(cam)} camera frames"
          if servo else f"{path}: no servo reads")
    if not servo:
        return
    joints = [j for j in range(1, 8) if servo[0][f"goal{j}"]]
    for j in joints:
        goal = int(servo[0][f"goal{j}"])
        pts = [(float(r["t"]), int(r[f"pos{j}"]), int(r[f"load{j}"])) for r in servo if r[f"pos{j}"]]
        start = pts[0][1]
        arrive = next((t for t, p, _ in pts if abs(p - goal) <= 20), None)
        speeds = [abs(p2 - p1) / (t2 - t1) * DEG_PER_TICK for (t1, p1, _), (t2, p2, _) in zip(pts, pts[1:]) if t2 > t1]
        print(f"  M{j}: {start} -> {goal} ({(goal - start) * DEG_PER_TICK:+.0f} deg); "
              f"there after {arrive:.3f} s; " if arrive is not None else f"  M{j}: {start} -> {goal}: never within 20 ticks; ", end="")
        print(f"peak speed {max(speeds, default=0):.0f} deg/s; peak load {max(abs(l) for _, _, l in pts) / 10:.0f} %")
    seen = [(float(r["t"]), float(r["bx"]), float(r["by"])) for r in cam if r["bx"]]
    if seen:
        print("  ball on the camera (blade mm):")
        for t, x, y in seen[:12]:
            print(f"    {t:6.3f} s  ({x:6.1f}, {y:6.1f})  {math.hypot(x, y):5.1f} mm from centre")
        print(f"  last seen at {seen[-1][0]:.3f} s")
    else:
        print("  ball not seen by the camera during the throw")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyse(p)
