#!/usr/bin/env python3
"""Detector-level metrics from one or more .rlog files, side by side (A/B of HEF / preprocessing).

    python3 det_report.py v2.rlog v3.rlog [--thr 0.30]

For a fair detector A/B, run both versions on the same frames:
    rcws_vision --source replay:run.rclip --detector hailo --log v3.rlog   (live NPU on recorded frames)
(--detector replay:... re-injects the recorded boxes and cannot compare detectors.)

Metrics
  proc p50/p99      vision-thread time per frame [ms] (includes preprocessing)
  det lat p50/p99   submit -> result [ms] (preprocessing is before submit, inference after)
  hit% Global/Tile/Roi  batches of that kind containing a phone >= thr
  roi hit% TRACK    ROI batches during TRACK with a phone >= thr  (detector recall where it matters)
  score med         median phone score in ROI batches during TRACK
  TRACK% / losses   state occupancy, number of TRACK -> COAST transitions
  with TRUTH (synthetic or monitor bench):
  recall            phone found in batches whose crop contained the visible truth
  ctr err p50/p95   detector box centre vs truth [px]
  static jitter     std of detector centre while truth moves < 0.5 px/frame [px]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rcws_log import read_log  # noqa: E402

TRACK, COAST = 2, 3


def metrics(log, thr):
    r = {}
    fr = log["frame"]
    if len(fr):
        proc = (fr["t_out"] - fr["t_in"]) * 1e-6
        r["proc p50"], r["proc p99"] = np.percentile(proc, 50), np.percentile(proc, 99)
        r["TRACK%"] = 100.0 * np.mean(fr["state"] == TRACK)
    st = log["state"]
    r["losses"] = int(np.sum((st["from"] == TRACK) & (st["to"] == COAST))) if len(st) else 0
    dets = log["det"]
    if not dets:
        return r
    lat = np.array([(h["t_done"] - h["t_submit"]) * 1e-6 for h, _ in dets])
    r["det lat p50"], r["det lat p99"] = np.percentile(lat, 50), np.percentile(lat, 99)

    # state at the time a batch was consumed
    seq2state = dict(zip(fr["seq"].tolist(), fr["state"].tolist())) if len(fr) else {}
    hits = {0: [], 1: [], 2: []}
    roi_track, scores = [], []
    for h, boxes in dets:
        ph = boxes[(boxes["cls"] == 0) & (boxes["score"] >= thr)] if len(boxes) else boxes
        hits[int(h["kind"])].append(len(ph) > 0)
        if int(h["kind"]) == 2 and seq2state.get(int(h["consumed_seq"])) == TRACK:
            roi_track.append(len(ph) > 0)
            if len(ph):
                scores.append(float(ph["score"].max()))
    for k, name in ((0, "Global"), (1, "Tile"), (2, "Roi")):
        if hits[k]:
            r[f"hit% {name}"] = 100.0 * np.mean(hits[k])
    if roi_track:
        r["roi hit% TRACK"] = 100.0 * np.mean(roi_track)
    if scores:
        r["score med"] = float(np.median(scores))

    tr = log["truth"]
    if len(tr):
        truth = {int(t["seq"]): t for t in tr}
        found, err, per_seq = [], [], {}
        for h, boxes in dets:
            t = truth.get(int(h["seq"]))
            if t is None or not t["visible"]:
                continue
            cx0, cy0, cw, ch = h["crop"]
            if not (cx0 <= t["u"] <= cx0 + cw and cy0 <= t["v"] <= cy0 + ch):
                continue
            ph = boxes[(boxes["cls"] == 0) & (boxes["score"] >= thr)] if len(boxes) else boxes
            if not len(ph):
                found.append(False)
                continue
            c = ph["box"][:, :2] + ph["box"][:, 2:] / 2
            d = np.hypot(c[:, 0] - t["u"], c[:, 1] - t["v"])
            i = int(np.argmin(d))
            ok = d[i] < max(t["w"], t["h"])
            found.append(ok)
            if ok:
                err.append(d[i])
                per_seq[int(h["seq"])] = (c[i, 0] - t["u"], c[i, 1] - t["v"])
        if found:
            r["recall"] = 100.0 * np.mean(found)
        if err:
            r["ctr err p50"], r["ctr err p95"] = np.percentile(err, 50), np.percentile(err, 95)
        # jitter: detector-minus-truth residual std over frames where truth is (nearly) static
        seqs = np.array(sorted(truth))
        uv = np.array([[truth[s]["u"], truth[s]["v"]] for s in seqs])
        if len(seqs) > 2:
            spd = np.r_[np.inf, np.hypot(*np.diff(uv, axis=0).T)]
            static = set(seqs[spd < 0.5].tolist())
            res = np.array([per_seq[s] for s in per_seq if s in static])
            if len(res) >= 10:
                r["static jitter"] = float(np.sqrt(np.mean(np.var(res, axis=0))))
    return r


ORDER = ["proc p50", "proc p99", "det lat p50", "det lat p99", "hit% Global", "hit% Tile", "hit% Roi",
         "roi hit% TRACK", "score med", "TRACK%", "losses", "recall", "ctr err p50", "ctr err p95", "static jitter"]
LOWER_BETTER = {"proc p50", "proc p99", "det lat p50", "det lat p99", "losses", "ctr err p50", "ctr err p95",
                "static jitter"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--thr", type=float, default=0.30, help="phone score threshold (detector.thr_track)")
    a = ap.parse_args()
    cols = [metrics(read_log(p), a.thr) for p in a.logs]
    names = [os.path.basename(p) for p in a.logs]
    w = max(12, *(len(n) for n in names))
    print(f"{'metric':<16}" + "".join(f"{n:>{w + 2}}" for n in names) + ("   (vs first)" if len(cols) > 1 else ""))
    for k in ORDER:
        if not any(k in c for c in cols):
            continue
        vals = [c.get(k) for c in cols]
        row = f"{k:<16}" + "".join(f"{'-' if v is None else f'{v:.2f}':>{w + 2}}" for v in vals)
        if len(cols) > 1 and vals[0] is not None and vals[-1] is not None and vals[-1] != vals[0]:
            better = (vals[-1] < vals[0]) == (k in LOWER_BETTER)
            row += "   better" if better else "   WORSE"
        print(row)


if __name__ == "__main__":
    main()
