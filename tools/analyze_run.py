#!/usr/bin/env python3
"""Analyse a paddle-balance run log (paddle_logs/run_<stamp>.csv).

Reports balance segments, ball offset from the centre, limit-cycle frequency and amplitude, servo tracking lag,
loop latency and the energy deadband use. Needs numpy only.

    python3 tools/analyze_run.py data/run_1795119100.csv
    python3 tools/analyze_run.py run.csv --from 330 --to 449     # analyse one time window (s)
"""
import argparse
import csv

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--from", dest="t_from", type=float, help="window start (s); default: longest balance segment")
    ap.add_argument("--to", dest="t_to", type=float)
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.csv)))
    col = lambda k: np.array([float(r[k]) for r in rows])
    t = col("t")
    state = np.array([r["state"] for r in rows])
    bal = state == "Balance"
    print(f"{len(rows)} frames over {t[-1] - t[0]:.0f} s = {len(rows) / (t[-1] - t[0]):.1f} fps; {bal.sum()} balancing")

    segs, s = [], None
    for i, b in enumerate(bal):
        if b and s is None:
            s = i
        if not b and s is not None:
            segs.append((s, i))
            s = None
    if s is not None:
        segs.append((s, len(bal)))
    print("balance segments (s):", ", ".join(f"{t[b - 1] - t[a]:.1f}" for a, b in segs))
    if not segs:
        return

    if a.t_from is not None:
        i0, i1 = np.searchsorted(t, a.t_from), np.searchsorted(t, a.t_to if a.t_to else t[-1])
    else:
        i0, i1 = max(segs, key=lambda x: x[1] - x[0])
    T = t[i0:i1]
    X, Y = col("bx")[i0:i1], col("by")[i0:i1]
    r = np.hypot(X, Y)
    print(f"\nwindow {T[0]:.1f}-{T[-1]:.1f} s ({T[-1] - T[0]:.1f} s)")
    print(f"ball offset: median {np.median(r):.1f} mm, p95 {np.percentile(r, 95):.1f}, max {r.max():.1f}; "
          f"<5 mm {100 * (r < 5).mean():.0f}%, <10 mm {100 * (r < 10).mean():.0f}%, <20 mm {100 * (r < 20).mean():.0f}%")
    print(f"bias: x {X.mean():+.1f} mm, y {Y.mean():+.1f} mm")

    dt = np.median(np.diff(T))
    n_smooth = max(3, int(5 / dt))  # remove drift slower than ~0.2 Hz
    for name, sig in (("x (sideways)", X), ("y (near/far)", Y)):
        s = sig - np.convolve(sig, np.ones(n_smooth) / n_smooth, "same")
        F = np.fft.rfftfreq(len(s), dt)
        A = np.abs(np.fft.rfft(s))
        m = (F > 0.1) & (F < 3)
        k = np.argmax(A * m)
        print(f"oscillation {name}: {F[k]:.2f} Hz, amplitude {2 * A[k] / len(s):.1f} mm")

    tx, ty = np.degrees(col("tilt_x")[i0:i1]), np.degrees(col("tilt_y")[i0:i1])
    print(f"tilt (deg): x std {tx.std():.2f} max {np.abs(tx).max():.2f}; y std {ty.std():.2f} max {np.abs(ty).max():.2f}")

    print("\nservo tracking (goal -> present):")
    for j in ("5", "6", "7"):
        g, p = col("g" + j)[i0:i1], col("p" + j)[i0:i1]
        gg, pp = g - g.mean(), p - p.mean()
        c = [np.dot(gg[: len(gg) - k], pp[k:]) for k in range(15)]
        lag = int(np.argmax(c))
        print(f"  M{j}: lag {lag} frames ({lag * dt * 1e3:.0f} ms), |goal-present| mean {np.abs(g - p).mean():.1f} ticks, "
              f"current mean {col('i' + j)[i0:i1].mean():.0f} mA")

    lat = col("latency_ms")[i0:i1]
    print(f"\nloop latency (frame -> goal written): median {np.median(lat):.1f} ms, p99 {np.percentile(lat, 99):.1f} ms")
    vx, vy = col("vx")[i0:i1], col("vy")[i0:i1]
    print(f"energy deadband active: {100 * ((r < 4) & (np.hypot(vx, vy) < 10)).mean():.0f}% of frames")
    print(f"ball found: {100 * col('found')[i0:i1].mean():.1f}% of frames; max frame gap {np.diff(T).max() * 1e3:.0f} ms")


if __name__ == "__main__":
    main()
