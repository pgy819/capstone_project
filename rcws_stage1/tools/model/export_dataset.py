#!/usr/bin/env python3
"""Build a YOLO dataset (+ Hailo calibration set) from recorded runs (.rclip + .rlog).

Why: the runtime feeds the detector *scale-normalised ROI crops* (k_s x target, Sec 3.5) and
lores full frames, not the COCO photo distribution. Training and INT8 calibration on the same
input distribution is the largest accuracy lever left (README Sec 5: custom 2-class HEF).

Labels per frame, by priority:
  1. TRUTH records (synthetic scene / monitor test bench)            -> exact
  2. detector boxes with score >= --det-thr                           -> pseudo-label
  3. accepted NCC measurements in frames the detector missed (--ncc-fill)
     -> hard positives: the tracker held the target where YOLO did not
Split (design V2.6 Sec 3.11): one recording = one capture session. With two or more --run,
whole sessions go to val (--val-run, default: the last --run); with a single --run the split
falls back to time blocks so neighbouring frames never straddle it.
Hard negatives: --neg-run records of look-alike objects (remote, wallet, notebook, tablet ...)
with no phone in view; every frame is exported unlabelled (lores + ROI-sized crops).

  python3 export_dataset.py --run s1.rclip:s1.rlog --run s2.rclip:s2.rlog --val-run s3.rclip:s3.rlog \
          --neg-run decoys.rclip -o ds --roi-crops 3 --ncc-fill --calib 1024
Then: python3 train_yolo.py ds/data.yaml ; hailo/compile_hef.sh best.onnx ds/calib
"""
import argparse
import os
import struct
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from rcws_log import read_log  # noqa: E402

PHONE, PERSON = 0, 1           # class ids in the exported dataset (and the custom HEF)
LOG_CLS = {0: PHONE, 1: PERSON}  # ObjClass in the log
CLIP_HDR = struct.Struct("<8s6I")
CLIP_META = struct.Struct("<Iqiff")


def read_clip(path):
    """Yield (seq, t_sof, bgr_main, lores_bgr or None) for each frame of an .rclip."""
    with open(path, "rb") as f:
        magic, ver, mw, mh, lw, lh, n = CLIP_HDR.unpack(f.read(CLIP_HDR.size))
        if magic != b"RCWSCLIP":
            raise ValueError(f"{path}: not an RCWS clip")
        main_b, lo_b = mw * mh * 3 // 2, lw * lh * 3 // 2
        for _ in range(n):
            m = f.read(CLIP_META.size)
            if len(m) < CLIP_META.size:
                return
            seq, t_sof, _exp, _gain, _lens = CLIP_META.unpack(m)
            buf = f.read(main_b + lo_b)
            if len(buf) < main_b + lo_b:
                return
            a = np.frombuffer(buf, np.uint8)
            main = cv2.cvtColor(a[:main_b].reshape(mh * 3 // 2, mw), cv2.COLOR_YUV2BGR_I420)
            lo = cv2.cvtColor(a[main_b:].reshape(lh * 3 // 2, lw), cv2.COLOR_YUV2BGR_I420) if lw else None
            yield seq, t_sof, main, lo


def collect_labels(log, det_thr, person_thr, ncc_fill):
    """seq -> list of (cls, cx, cy, w, h, source) in main-stream pixels."""
    out, phone_seen = {}, set()
    truth_seqs = set()
    for t in log["truth"]:
        truth_seqs.add(int(t["seq"]))
        if t["visible"]:
            out.setdefault(int(t["seq"]), []).append((PHONE, t["u"], t["v"], t["w"], t["h"], "truth"))
            phone_seen.add(int(t["seq"]))
    for h, boxes in log["det"]:
        seq = int(h["seq"])
        for b in boxes:
            cls = LOG_CLS.get(int(b["cls"]))
            if cls is None:
                continue
            if cls == PHONE and (seq in truth_seqs or b["score"] < det_thr):
                continue
            if cls == PERSON and b["score"] < person_thr:
                continue
            x, y, w, hh = (float(v) for v in b["box"])
            lst = out.setdefault(seq, [])
            # the same frame can be detected twice (tile + ROI): keep one box per object
            if any(c == cls and abs(cx - (x + w / 2)) < 0.5 * w and abs(cy - (y + hh / 2)) < 0.5 * hh
                   for c, cx, cy, *_ in lst):
                continue
            lst.append((cls, x + w / 2, y + hh / 2, w, hh, "det"))
            if cls == PHONE:
                phone_seen.add(seq)
    n_ncc = 0
    if ncc_fill:
        m = log["meas"]
        m = m[(m["src"] == 1) & (m["accepted"] == 1)]
        for r in m:
            seq = int(r["seq"])
            if seq in phone_seen or seq in truth_seqs or r["w"] <= 0:
                continue
            out.setdefault(seq, []).append((PHONE, r["u"], r["v"], r["w"], r["h"], "ncc"))
            phone_seen.add(seq)
            n_ncc += 1
    return out, n_ncc


def yolo_lines(labels, x0, y0, side_w, side_h, min_vis=0.5):
    """Labels inside the crop [x0, x0+side_w) x [y0, y0+side_h) as normalised YOLO lines."""
    lines = []
    for cls, cx, cy, w, h, _ in labels:
        bx0, by0, bx1, by1 = cx - w / 2 - x0, cy - h / 2 - y0, cx + w / 2 - x0, cy + h / 2 - y0
        cx0, cy0, cx1, cy1 = max(bx0, 0), max(by0, 0), min(bx1, side_w), min(by1, side_h)
        if cx1 <= cx0 or cy1 <= cy0 or (cx1 - cx0) * (cy1 - cy0) < min_vis * w * h:
            continue
        lines.append(f"{cls} {(cx0 + cx1) / 2 / side_w:.6f} {(cy0 + cy1) / 2 / side_h:.6f} "
                     f"{(cx1 - cx0) / side_w:.6f} {(cy1 - cy0) / side_h:.6f}")
    return lines


def letterbox(img, size=640):
    """Same mapping as detector.cpp make_request: aspect-preserving resize, centred, grey 114."""
    h, w = img.shape[:2]
    s = min(size / w, size / h)
    nw, nh = max(1, round(w * s)), max(1, round(h * s))
    out = np.full((size, size, 3), 114, np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    out[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    return out


def motion_blur(img, rng, kmax):
    k = int(rng.integers(3, kmax + 1)) | 1
    ker = np.zeros((k, k), np.float32)
    ker[k // 2, :] = 1.0
    rot = cv2.getRotationMatrix2D((k / 2 - 0.5, k / 2 - 0.5), float(rng.uniform(0, 180)), 1.0)
    ker = cv2.warpAffine(ker, rot, (k, k))
    return cv2.filter2D(img, -1, ker / max(ker.sum(), 1e-6))


def roi_square(cx, cy, w, h, k_s, s_min, s_max, W, H, rng, jitter):
    """Track ROI as the runtime builds it (roi.cpp track_roi), with random size/centre jitter."""
    side = float(np.clip(k_s * max(w, h) * rng.uniform(0.8, 1.25), s_min, s_max))
    side = int(round(side)) & ~1
    side = min(side, W, H)
    x = int(round(cx - side / 2 + rng.uniform(-jitter, jitter) * side))
    y = int(round(cy - side / 2 + rng.uniform(-jitter, jitter) * side))
    return min(max(x, 0), W - side), min(max(y, 0), H - side), side


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="append", required=True, help="clip.rclip:log.rlog (repeatable, one per session)")
    ap.add_argument("--val-run", action="append", default=[], help="session exported entirely to val (repeatable)")
    ap.add_argument("--neg-run", action="append", default=[], help="clip.rclip of hard negatives, exported unlabelled")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--det-thr", type=float, default=0.5, help="min score for detector pseudo-labels (phone)")
    ap.add_argument("--person-thr", type=float, default=0.6)
    ap.add_argument("--ncc-fill", action="store_true", help="label frames the tracker held but YOLO missed")
    ap.add_argument("--full", choices=["main", "lores", "none"], default="lores",
                    help="full-frame images to export (lores = what the global search sees)")
    ap.add_argument("--roi-crops", type=int, default=3, help="ROI crops per labelled phone")
    ap.add_argument("--k-s", type=float, default=4.0)
    ap.add_argument("--s-min", type=int, default=96)
    ap.add_argument("--s-max", type=int, default=864)
    ap.add_argument("--jitter", type=float, default=0.2, help="ROI centre jitter (fraction of side)")
    ap.add_argument("--negatives", type=float, default=0.1, help="fraction of unlabelled frames kept as background")
    ap.add_argument("--blur", type=float, default=0.0, help="probability of extra motion blur per image")
    ap.add_argument("--blur-k", type=int, default=11)
    ap.add_argument("--block", type=int, default=120, help="frames per split block (120 = 1 s)")
    ap.add_argument("--val-every", type=int, default=5, help="every N-th block goes to val")
    ap.add_argument("--stride", type=int, default=2, help="use every N-th frame (neighbours are near-duplicates)")
    ap.add_argument("--calib", type=int, default=0, help="also write N letterboxed 640x640 calibration images")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(a.out, sub), exist_ok=True)
    calib_pool = []
    stats = {"frames": 0, "images": 0, "boxes": 0, "ncc": 0, "neg": 0, "val": 0}

    def write(img, lines, split, name):
        if a.blur > 0 and rng.random() < a.blur:
            img = motion_blur(img, rng, a.blur_k)
        cv2.imwrite(os.path.join(a.out, "images", split, name + ".jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(os.path.join(a.out, "labels", split, name + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        stats["images"] += 1
        stats["boxes"] += len(lines)
        stats["val"] += split == "val"

    runs = [(s, "train") for s in a.run] + [(s, "val") for s in a.val_run]
    if not a.val_run and len(a.run) >= 2:
        runs[-1] = (runs[-1][0], "val")          # session split: last session is validation
    by_session = len(runs) >= 2

    for ri, (spec, run_split) in enumerate(runs):
        clip_path, log_path = spec.rsplit(":", 1) if ":" in spec[2:] else (spec, spec.rsplit(".", 1)[0] + ".rlog")
        log = read_log(log_path)
        labels, n_ncc = collect_labels(log, a.det_thr, a.person_thr, a.ncc_fill)
        stats["ncc"] += n_ncc
        for k, (seq, _t, main, lo) in enumerate(read_clip(clip_path)):
            if k % a.stride:
                continue
            stats["frames"] += 1
            if by_session:
                split = run_split
            else:  # middle block of each period, so even a 3-block recording gets a validation part
                split = "val" if (k // a.block) % a.val_every == a.val_every // 2 else "train"
            H, W = main.shape[:2]
            lab = labels.get(seq, [])
            if not lab:
                if rng.random() < a.negatives:
                    src = lo if (a.full == "lores" and lo is not None) else main
                    write(src, [], split, f"r{ri}_{seq:07d}_neg")
                    stats["neg"] += 1
                continue
            if a.full != "none":
                if a.full == "lores" and lo is not None:
                    sx, sy = lo.shape[1] / W, lo.shape[0] / H
                    scaled = [(c, cx * sx, cy * sy, w * sx, h * sy, s) for c, cx, cy, w, h, s in lab]
                    write(lo, yolo_lines(scaled, 0, 0, lo.shape[1], lo.shape[0]), split, f"r{ri}_{seq:07d}_full")
                else:
                    write(main, yolo_lines(lab, 0, 0, W, H), split, f"r{ri}_{seq:07d}_full")
            for li, (cls, cx, cy, w, h, _src) in enumerate(lab):
                if cls != PHONE:
                    continue
                for j in range(a.roi_crops):
                    x0, y0, side = roi_square(cx, cy, w, h, a.k_s, a.s_min, a.s_max, W, H, rng, a.jitter)
                    crop = main[y0:y0 + side, x0:x0 + side]
                    write(crop, yolo_lines(lab, x0, y0, side, side), split, f"r{ri}_{seq:07d}_roi{li}{j}")
                    if a.calib and split == "train":
                        calib_pool.append(crop)
            if a.calib and lo is not None and split == "train":
                calib_pool.append(lo)

    for ni, clip_path in enumerate(a.neg_run):
        for k, (seq, _t, main, lo) in enumerate(read_clip(clip_path)):
            if k % a.stride:
                continue
            stats["frames"] += 1
            if lo is not None:
                write(lo, [], "train", f"n{ni}_{seq:07d}_full")
            H, W = main.shape[:2]
            for j in range(a.roi_crops):   # ROI-sized views of the clutter, as the tracker would see it
                side = int(rng.integers(a.s_min, min(a.s_max, W, H) + 1)) & ~1
                x0, y0 = int(rng.integers(0, W - side + 1)), int(rng.integers(0, H - side + 1))
                write(main[y0:y0 + side, x0:x0 + side], [], "train", f"n{ni}_{seq:07d}_roi{j}")
            stats["neg"] += 1 + a.roi_crops

    with open(os.path.join(a.out, "data.yaml"), "w") as f:
        f.write(f"path: {os.path.abspath(a.out)}\ntrain: images/train\nval: images/val\n"
                "names:\n  0: phone\n  1: person\n")
    if a.calib and calib_pool:
        cdir = os.path.join(a.out, "calib")
        os.makedirs(cdir, exist_ok=True)
        idx = rng.choice(len(calib_pool), size=min(a.calib, len(calib_pool)), replace=False)
        for n, i in enumerate(idx):
            # JPEG stores true colours, so any loader (PIL in hailomz, cv2) reads them correctly
            cv2.imwrite(os.path.join(cdir, f"calib_{n:05d}.jpg"), letterbox(calib_pool[i]), [cv2.IMWRITE_JPEG_QUALITY, 95])
        stats["calib"] = len(idx)
        if len(idx) < 500:
            print(f"warning: {len(idx)} calibration images < 500 (design Sec 3.11)", file=sys.stderr)
    print("  ".join(f"{k} {v}" for k, v in stats.items()))
    if stats["boxes"] == 0:
        print("warning: no labels exported (lower --det-thr, add --ncc-fill, or record with TRUTH)", file=sys.stderr)


if __name__ == "__main__":
    main()
