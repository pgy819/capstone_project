#!/usr/bin/env python3
"""Reader for RCWS binary run logs (.rlog), mirrors rpi/src/record/binlog.hpp.

    from rcws_log import read_log
    log = read_log("run.rlog")      # dict of numpy structured arrays + config dict
    log["meas"]["u"], log["est"]["psi"], ...

CLI:  python3 rcws_log.py run.rlog            -> summary
"""
import struct
import sys

import numpy as np

FRAME = np.dtype([("seq", "<u4"), ("t_sof", "<i8"), ("t_in", "<i8"), ("t_out", "<i8"), ("boot_off", "<i8"),
                  ("exposure_us", "<i4"), ("gain", "<f4"), ("lens", "<f4"), ("state", "u1"), ("in_flight", "u1"),
                  ("dropped", "<u2")])
DETHEAD = np.dtype([("seq", "<u4"), ("consumed_seq", "<u4"), ("t_sof", "<i8"), ("t_submit", "<i8"), ("t_done", "<i8"),
                    ("crop", "<f4", 4), ("kind", "u1"), ("n", "u1"), ("pad", "<u2")])
DET = np.dtype([("box", "<f4", 4), ("score", "<f4"), ("cls", "u1"), ("pad", "u1", 3)])
MEAS = np.dtype([("t_cap", "<i8"), ("seq", "<u4"), ("u", "<f4"), ("v", "<f4"), ("sigma", "<f4"), ("conf", "<f4"),
                 ("w", "<f4"), ("h", "<f4"), ("d2", "<f4"), ("src", "u1"), ("accepted", "u1"), ("pad", "u1", 2)])
EST = np.dtype([("t", "<i8"), ("psi", "<f8"), ("psi_d", "<f8"), ("psi_dd", "<f8"), ("theta", "<f8"), ("theta_d", "<f8"),
                ("theta_dd", "<f8"), ("sig_psi", "<f8"), ("sig_theta", "<f8"), ("mu_ca", "<f4"), ("range_m", "<f4")])
STATE = np.dtype([("t", "<i8"), ("from", "u1"), ("to", "u1"), ("reason", "u1"), ("pad", "u1")])
NCC = np.dtype([("t", "<i8"), ("seq", "<u4"), ("u", "<f4"), ("v", "<f4"), ("rho", "<f4"), ("second", "<f4"),
                ("radius", "<f4"), ("tmpl", "<i2"), ("ok", "u1"), ("pad", "u1")])
TRUTH = np.dtype([("t", "<i8"), ("seq", "<u4"), ("u", "<f4"), ("v", "<f4"), ("w", "<f4"), ("h", "<f4"),
                  ("visible", "u1"), ("pad", "u1", 3)])

TYPES = {1: ("frame", FRAME), 3: ("meas", MEAS), 4: ("est", EST), 5: ("state", STATE), 6: ("ncc", NCC), 8: ("truth", TRUTH)}
STATES = ["SEARCH", "ACQUIRE", "TRACK", "COAST", "REACQ"]
REASONS = ["NONE", "DET_ACQUIRE", "SPRT_ACCEPT", "SPRT_REJECT", "ACQ_TIMEOUT", "NO_MEAS", "NCC_ONLY", "MEAS_BACK",
           "COAST_TIMEOUT", "REACQ_DET", "REACQ_TIMEOUT"]
KINDS = ["Global", "Tile", "Roi"]
CLASSES = {0: "phone", 1: "person", 255: "other"}




def read_log(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:7] != b"RCWSLOG":
        raise ValueError("not an RCWS log")
    ver, n = struct.unpack_from("<II", data, 8)
    cfg_text = data[16:16 + n].decode(errors="replace")
    cfg = dict(line.split("=", 1) for line in cfg_text.splitlines() if "=" in line)
    pos = 16 + n
    chunks = {k: [] for k, _ in TYPES.values()}
    dets, notes = [], []
    end = len(data)
    while pos + 4 <= end:
        ty, ln = struct.unpack_from("<HH", data, pos)
        pos += 4
        payload = data[pos:pos + ln]
        pos += ln
        if len(payload) < ln:
            break
        if ty in TYPES:
            key, dt = TYPES[ty]
            if ln == dt.itemsize:
                chunks[key].append(payload)
        elif ty == 2:
            h = np.frombuffer(payload[:DETHEAD.itemsize], DETHEAD)[0]
            boxes = np.frombuffer(payload[DETHEAD.itemsize:DETHEAD.itemsize + h["n"] * DET.itemsize], DET)
            dets.append((h, boxes))
        elif ty == 7:
            notes.append(payload.decode(errors="replace"))
    out = {k: (np.frombuffer(b"".join(v), dt) if v else np.zeros(0, dt)) for (k, dt), v in
           zip(TYPES.values(), chunks.values())}
    out["det"] = dets
    out["notes"] = notes
    out["config"] = cfg
    out["version"] = ver
    return out


def summary(log):
    fr = log["frame"]
    print(f"frames {len(fr)}  config keys {len(log['config'])}")
    if len(fr) > 1:
        dt = np.diff(fr["t_sof"]) * 1e-6
        proc = (fr["t_out"] - fr["t_in"]) * 1e-6
        print(f"frame interval ms: median {np.median(dt):.3f}  max {dt.max():.3f}   dropped total {int(fr['dropped'].sum())}")
        print(f"processing ms: p50 {np.percentile(proc, 50):.2f}  p99 {np.percentile(proc, 99):.2f}  max {proc.max():.2f}")
        occ = np.bincount(fr["state"], minlength=5) / len(fr)
        print("state occupancy: " + "  ".join(f"{STATES[i]} {100 * occ[i]:.1f}%" for i in range(5)))
    for s in log["state"]:
        print(f"  t={s['t'] * 1e-9:9.3f}  {STATES[s['from']]:>7} -> {STATES[s['to']]:<7} ({REASONS[s['reason']]})")
    if log["det"]:
        lat = np.array([(h["t_done"] - h["t_submit"]) * 1e-6 for h, _ in log["det"]])
        print(f"detector batches {len(lat)}  latency ms p50 {np.percentile(lat, 50):.2f} p99 {np.percentile(lat, 99):.2f}")
    m = log["meas"]
    if len(m):
        for src, name in ((0, "DET"), (1, "NCC")):
            mm = m[m["src"] == src]
            if len(mm):
                print(f"{name} measurements {len(mm)}  accepted {100 * mm['accepted'].mean():.1f}%  "
                      f"median sigma {np.median(mm['sigma']):.2f} px")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    summary(read_log(sys.argv[1]))
