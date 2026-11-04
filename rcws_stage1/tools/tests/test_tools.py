#!/usr/bin/env python3
"""Self-test for tools/model/export_dataset.py and tools/det_report.py on a synthetic run.
Writes .rclip/.rlog in the exact binary formats of record/clip.cpp and record/binlog.cpp.
    python3 tools/tests/test_tools.py        (needs numpy + opencv-python only)
"""
import os
import struct
import subprocess
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
sys.path.insert(0, TOOLS)
import rcws_log as L  # noqa: E402

MW, MH, LW, LH, N = 1536, 864, 640, 360, 240
TW, TH = 40, 80


def truth_uv(k):
    return 768 + 300 * np.sin(2 * np.pi * k / 120), 432 + 120 * np.sin(2 * np.pi * k / 90)


def write_clip(path):
    with open(path, "wb") as f:
        f.write(struct.pack("<8s6I", b"RCWSCLIP", 1, MW, MH, LW, LH, N))
        for k in range(N):
            u, v = truth_uv(k)
            bgr = np.full((MH, MW, 3), (60, 90, 70), np.uint8)
            cv2.rectangle(bgr, (int(u - TW / 2), int(v - TH / 2)), (int(u + TW / 2) - 1, int(v + TH / 2) - 1),
                          (40, 220, 250), -1)
            yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
            lo = cv2.cvtColor(cv2.resize(bgr, (LW, LH), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2YUV_I420)
            f.write(struct.pack("<Iqiff", k, k * 8_333_333, 3000, 4.0, 0.81))
            f.write(yuv.tobytes())
            f.write(lo.tobytes())


def rec(ty, arr):
    b = arr.tobytes()
    return struct.pack("<HH", ty, len(b)) + b


def write_log(path, noise_px, miss_every, truth=True, seed=0):
    rng = np.random.default_rng(seed)
    out = bytearray(b"RCWSLOG\0" + struct.pack("<II", 1, 0))
    for k in range(N):
        u, v = truth_uv(k)
        fr = np.zeros(1, L.FRAME)
        fr["seq"], fr["t_sof"], fr["t_in"], fr["t_out"], fr["state"] = k, k * 8_333_333, k * 8_333_333, k * 8_333_333 + int(3e6 + 1e6 * noise_px), 2
        out += rec(1, fr)
        if truth:
            t = np.zeros(1, L.TRUTH)
            t["seq"], t["u"], t["v"], t["w"], t["h"], t["visible"] = k, u, v, TW, TH, 1
            out += rec(8, t)
        miss = miss_every and k % miss_every == 0
        h = np.zeros(1, L.DETHEAD)
        h["seq"], h["consumed_seq"], h["t_submit"], h["t_done"] = k, k + 2, k * 8_333_333, k * 8_333_333 + 9_000_000
        h["crop"] = (u - 160, v - 160, 320, 320)
        h["kind"] = 2
        boxes = np.zeros(0 if miss else 2, L.DET)
        if not miss:
            e = rng.normal(0, noise_px, 2)
            boxes[0]["box"] = (u + e[0] - TW / 2, v + e[1] - TH / 2, TW, TH)
            boxes[0]["score"], boxes[0]["cls"] = 0.8, 0
            boxes[1]["box"] = (100, 100, 200, 500)                   # a person
            boxes[1]["score"], boxes[1]["cls"] = 0.9, 1
        h["n"] = len(boxes)
        out += struct.pack("<HH", 2, h.itemsize + boxes.nbytes) + h.tobytes() + boxes.tobytes()
        m = np.zeros(1, L.MEAS)                                      # NCC every frame
        m["seq"], m["u"], m["v"], m["w"], m["h"], m["src"], m["accepted"] = k, u + 0.2, v - 0.2, TW, TH, 1, 1
        out += rec(3, m)
    s = np.zeros(1, L.STATE)
    s["from"], s["to"] = 2, 3
    out += rec(5, s)
    with open(path, "wb") as f:
        f.write(out)


def check(c, msg):
    print(("  ok   " if c else "  FAIL ") + msg)
    return bool(c)


def main():
    ok = True
    with tempfile.TemporaryDirectory() as d:
        clip, la, lb = os.path.join(d, "r.rclip"), os.path.join(d, "a.rlog"), os.path.join(d, "b.rlog")
        write_clip(clip)
        write_log(la, noise_px=0.5, miss_every=0)
        write_log(lb, noise_px=3.0, miss_every=4, truth=True, seed=1)

        # ---- export: pseudo-labels from DET + NCC fill (b misses every 4th frame), no truth used
        lnt = os.path.join(d, "nt.rlog")
        write_log(lnt, noise_px=0.0, miss_every=4, truth=False)
        ds = os.path.join(d, "ds")
        r = subprocess.run([sys.executable, os.path.join(TOOLS, "model", "export_dataset.py"), "--run", f"{clip}:{lnt}",
                            "-o", ds, "--ncc-fill", "--roi-crops", "2", "--stride", "1", "--block", "60",
                            "--calib", "50"], capture_output=True, text=True)
        print(r.stdout.strip(), r.stderr.strip())
        ok &= check(r.returncode == 0, "export_dataset runs")
        ok &= check(os.path.exists(os.path.join(ds, "data.yaml")), "data.yaml written")
        ok &= check(len(os.listdir(os.path.join(ds, "calib"))) == 50, "50 calibration images")
        ok &= check(len(os.listdir(os.path.join(ds, "images", "val"))) > 0, "val split non-empty")
        ok &= check("ncc 60" in r.stdout, "every 4th frame (60) recovered from NCC")
        # every ROI crop label must sit on the bright target in the crop
        bad = n = 0
        for split in ("train", "val"):
            for name in os.listdir(os.path.join(ds, "labels", split)):
                if "_roi" not in name:
                    continue
                img = cv2.imread(os.path.join(ds, "images", split, name[:-4] + ".jpg"))
                for line in open(os.path.join(ds, "labels", split, name)):
                    c, x, y, w, h = line.split()
                    if c != "0":
                        continue
                    H, W = img.shape[:2]
                    px = img[int(float(y) * H), int(float(x) * W)]
                    n += 1
                    bad += not (px[1] > 180 and px[2] > 180)
        ok &= check(n > 100 and bad == 0, f"ROI-crop phone labels on target: {n - bad}/{n}")
        cal = cv2.imread(os.path.join(ds, "calib", "calib_00000.jpg"))
        ok &= check(cal.shape == (640, 640, 3), "calibration image is 640x640 letterboxed")

        # ---- session split + hard negatives
        clip2 = os.path.join(d, "r2.rclip")
        os.link(clip, clip2) if hasattr(os, "link") else None
        ds2 = os.path.join(d, "ds2")
        r = subprocess.run([sys.executable, os.path.join(TOOLS, "model", "export_dataset.py"), "--run", f"{clip}:{lnt}",
                            "--val-run", f"{clip2}:{lnt}", "--neg-run", clip, "-o", ds2, "--roi-crops", "1",
                            "--stride", "4", "--full", "none", "--ncc-fill", "--negatives", "0"], capture_output=True, text=True)
        print(r.stdout.strip())
        tr = os.listdir(os.path.join(ds2, "images", "train"))
        va = os.listdir(os.path.join(ds2, "images", "val"))
        ok &= check(r.returncode == 0 and len(va) == 60 and all(n.startswith("r1_") for n in va),
                    "val = whole second session (60 labelled ROI crops)")
        ok &= check(sum(1 for n in tr if n.startswith("r0_")) == 60, "train = first session (60 labelled ROI crops)")
        ok &= check(not any(n.startswith("r1_") for n in tr), "no val-session frame in train")
        negs = [n for n in tr if n.startswith("n0_")]
        ok &= check(len(negs) == 2 * 60 and all(
            os.path.getsize(os.path.join(ds2, "labels", "train", n[:-4] + ".txt")) == 0 for n in negs),
            "hard-negative session exported unlabelled (lores + ROI crop per frame)")

        # ---- det_report A/B
        r = subprocess.run([sys.executable, os.path.join(TOOLS, "det_report.py"), la, lb], capture_output=True, text=True)
        print(r.stdout)
        ok &= check(r.returncode == 0, "det_report runs")
        lines = {l.split()[0] + " " + l.split()[1]: l for l in r.stdout.splitlines() if l.startswith(("ctr", "roi"))}
        ok &= check("WORSE" in lines.get("ctr err", ""), "noisier run flagged WORSE on centre error")
        ok &= check("WORSE" in lines.get("roi hit%", ""), "missing detections flagged WORSE on ROI hit rate")
        ra = L.read_log(la)
        ok &= check(len(ra["det"]) == N and len(ra["truth"]) == N, "log round-trips through rcws_log")
    print("ALL OK" if ok else "FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
