#!/usr/bin/env python3
"""Intrinsic calibration for the Step 1 camera mode (calibration step 2, Sec 12.3).

Capture 30+ checkerboard images at exactly the runtime mode and fixed focus, covering
the whole frame including the corners (see README). Then:

    python3 calibrate_camera.py imgs/*.jpg --cols 9 --rows 6 --square 0.025 [--model rational]

Prints the [intrinsics] block for config/rcws.ini and the reprojection error overall
and at the image border (target: RMS <= 0.3 px).
"""
import argparse
import glob

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="+")
    ap.add_argument("--cols", type=int, required=True, help="inner corners per row")
    ap.add_argument("--rows", type=int, required=True, help="inner corners per column")
    ap.add_argument("--square", type=float, required=True, help="square size [m]")
    ap.add_argument("--model", choices=["k5", "rational"], default="k5",
                    help="k5: k1 k2 p1 p2 k3 (runtime default); rational: k4-k6 extra, report only")
    a = ap.parse_args()

    files = sorted(f for p in a.images for f in glob.glob(p))
    objp = np.zeros((a.rows * a.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    obj, img, size, used = [], [], None, []
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    for f in files:
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        if size is None:
            size = g.shape[::-1]
        elif g.shape[::-1] != size:
            print(f"skip {f}: size {g.shape[::-1]} != {size}")
            continue
        ok, c = cv2.findChessboardCornersSB(g, (a.cols, a.rows), cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if not ok:
            ok, c = cv2.findChessboardCorners(g, (a.cols, a.rows))
            if ok:
                c = cv2.cornerSubPix(g, c, (5, 5), (-1, -1), crit)
        if ok:
            obj.append(objp)
            img.append(c.astype(np.float32))
            used.append(f)
    print(f"{len(used)}/{len(files)} images usable, size {size}")
    if len(used) < 15:
        raise SystemExit("need at least 15 (recommended 30+) views")

    flags = cv2.CALIB_RATIONAL_MODEL if a.model == "rational" else 0
    rms, K, D, rv, tv = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
    errs, border = [], []
    w, h = size
    for o, i, r, t in zip(obj, img, rv, tv):
        p, _ = cv2.projectPoints(o, r, t, K, D)
        e = np.linalg.norm(p.reshape(-1, 2) - i.reshape(-1, 2), axis=1)
        errs.append(e)
        q = i.reshape(-1, 2)
        edge = (q[:, 0] < 0.15 * w) | (q[:, 0] > 0.85 * w) | (q[:, 1] < 0.15 * h) | (q[:, 1] > 0.85 * h)
        border.append(e[edge])
    errs, border = np.concatenate(errs), np.concatenate(border)
    print(f"reprojection RMS {rms:.3f} px   p95 {np.percentile(errs, 95):.3f}   "
          f"border RMS {np.sqrt(np.mean(border ** 2)) if len(border) else float('nan'):.3f} px (n={len(border)})")
    if rms > 0.3:
        print("WARNING: RMS > 0.3 px -> add views (corners!), check focus is fixed, or try --model rational")
    d = D.ravel()
    print("\n[intrinsics]")
    print(f"fx = {K[0, 0]:.4f}\nfy = {K[1, 1]:.4f}\ncx = {K[0, 2]:.4f}\ncy = {K[1, 2]:.4f}")
    for k, v in zip(("k1", "k2", "p1", "p2", "k3"), d[:5]):
        print(f"{k} = {v:.8f}")
    if a.model == "rational":
        print(f"# rational terms k4..k6 = {d[5:8]} (runtime uses k1..k3 only; compare border RMS)")


if __name__ == "__main__":
    main()
