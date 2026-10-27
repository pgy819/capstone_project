#!/usr/bin/env python3
"""Step 1 log visualisation (P2-P4): state timeline, LOS angles, measurement error,
NCC quality and processing latency.

    python3 track_plot.py run.rlog [-o run.png]

If the log has TRUTH records (synthetic scene or monitor test bench), the estimate
and measurement errors against the reference are plotted and summarised.
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from rcws_log import STATES, read_log


def los_from_pixel(u, v, cfg):
    """Pinhole + distortion via OpenCV if available, else pinhole only (for plotting truth)."""
    fx, fy = float(cfg["intrinsics.fx"]), float(cfg["intrinsics.fy"])
    cx, cy = float(cfg["intrinsics.cx"]), float(cfg["intrinsics.cy"])
    dist = np.array([float(cfg[f"intrinsics.{k}"]) for k in ("k1", "k2", "p1", "p2", "k3")])
    pts = np.stack([u, v], 1).astype(np.float64).reshape(-1, 1, 2)
    try:
        import cv2
        n = cv2.undistortPoints(pts, np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]]), dist).reshape(-1, 2)
    except ImportError:
        n = np.stack([(u - cx) / fx, (v - cy) / fy], 1)
    psi = np.arctan2(n[:, 0], 1.0)
    theta = np.arctan2(-n[:, 1], np.sqrt(1.0 + n[:, 0] ** 2))
    return psi, theta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("-o", "--out", default=None)
    a = ap.parse_args()
    L = read_log(a.log)
    fr, est, meas, ncc, tr = L["frame"], L["est"], L["meas"], L["ncc"], L["truth"]
    if len(fr) == 0:
        raise SystemExit("empty log")
    t0 = fr["t_sof"][0]
    ts = lambda x: (x - t0) * 1e-9

    fig, ax = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    ax[0].step(ts(fr["t_sof"]), fr["state"], where="post")
    ax[0].set_yticks(range(5), STATES)
    ax[0].set_title("track state")

    for k, (name, col) in enumerate((("psi", "C0"), ("theta", "C1"))):
        ax[1].plot(ts(est["t"]), np.degrees(est[name]), col, label=f"est {name}")
    acc = meas[meas["accepted"] == 1]
    if len(acc):
        mp, mt = los_from_pixel(acc["u"], acc["v"], L["config"])
        for src, mk in ((1, "."), (0, "x")):
            s = acc["src"] == src
            ax[1].plot(ts(acc["t_cap"][s]), np.degrees(mp[s]), "C0" + mk, ms=2, alpha=0.4)
            ax[1].plot(ts(acc["t_cap"][s]), np.degrees(mt[s]), "C1" + mk, ms=2, alpha=0.4)
    ax[1].set_ylabel("deg")
    ax[1].legend(loc="upper right")
    ax[1].set_title("LOS estimate (lines) and measurements (. NCC, x DET)")

    if len(tr) and len(est):
        vis = tr["visible"] == 1
        gp, gt = los_from_pixel(tr["u"][vis], tr["v"][vis], L["config"])
        tt = tr["t"][vis]
        ep = np.interp(tt, est["t"], est["psi"])
        et = np.interp(tt, est["t"], est["theta"])
        tracked = np.interp(tt, fr["t_sof"], fr["state"]) == 2
        err = 1e3 * np.hypot(ep - gp, et - gt)
        ax[2].plot(ts(tt[tracked]), err[tracked], ".", ms=2)
        ax[2].set_ylabel("mrad")
        ax[2].set_title(f"estimate error vs reference (TRACK): RMS {np.sqrt(np.mean(err[tracked] ** 2)):.3f} mrad, "
                        f"p95 {np.percentile(err[tracked], 95):.3f} mrad")
        print(f"estimate error (TRACK, visible): RMS {np.sqrt(np.mean(err[tracked] ** 2)):.3f} mrad  "
              f"p95 {np.percentile(err[tracked], 95):.3f}  max {err[tracked].max():.3f}")
        T = {int(s): (u, v) for s, u, v in zip(tr["seq"], tr["u"], tr["v"])}
        for src, name in ((1, "NCC"), (0, "DET")):
            mm = acc[acc["src"] == src]
            e = np.array([(m["u"] - T[int(m["seq"])][0], m["v"] - T[int(m["seq"])][1]) for m in mm if int(m["seq"]) in T])
            if len(e):
                print(f"{name} measurement error px: mean {e.mean(0).round(3)}  std {e.std(0).round(3)}  n={len(e)}")
    else:
        ax[2].text(0.5, 0.5, "no reference trajectory in log", ha="center", transform=ax[2].transAxes)

    if len(ncc):
        ok = ncc["ok"] == 1
        ax[3].plot(ts(ncc["t"][ok]), ncc["rho"][ok], ".", ms=2, label="peak")
        ax[3].plot(ts(ncc["t"][ok]), ncc["second"][ok], ".", ms=2, label="2nd peak")
        ax[3].plot(ts(ncc["t"][~ok]), ncc["rho"][~ok], "rx", ms=3, label="rejected")
        ax[3].legend(loc="lower right")
    ax[3].set_title("ZNCC quality")

    proc = (fr["t_out"] - fr["t_in"]) * 1e-6
    ax[4].plot(ts(fr["t_sof"]), proc, lw=0.6)
    ax[4].set_ylabel("ms")
    ax[4].set_xlabel("time [s]")
    ax[4].set_title(f"vision-thread processing per frame: p50 {np.percentile(proc, 50):.2f} ms, "
                    f"p99 {np.percentile(proc, 99):.2f} ms")
    fig.tight_layout()
    out = a.out or a.log.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=110)
    print("saved", out)


if __name__ == "__main__":
    main()
