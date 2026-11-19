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
                ("radius", "<f4"), ("tmpl", "<i2"), ("ok", "u1"), ("scale", "u1")])
TRUTH = np.dtype([("t", "<i8"), ("seq", "<u4"), ("u", "<f4"), ("v", "<f4"), ("w", "<f4"), ("h", "<f4"),
                  ("visible", "u1"), ("pad", "u1", 3)])

# v4 (scenarios A-J): multi-target board, aim output gate, image quality, extra synthetic objects
MOT = np.dtype([("t", "<i8"), ("seq", "<u4"), ("id", "<i4"), ("st", "u1"), ("flags", "u1"), ("pad", "<u2"),
                ("u", "<f4"), ("v", "<f4"), ("w", "<f4"), ("h", "<f4"), ("vu", "<f4"), ("vv", "<f4"),
                ("prio", "<f4"), ("score", "<f4")])
AIM = np.dtype([("t", "<i8"), ("seq", "<u4"), ("mode", "u1"), ("pad", "u1"), ("reasons", "<u2"), ("target_id", "<i4"),
                ("psi", "<f4"), ("theta", "<f4"), ("psi_d", "<f4"), ("theta_d", "<f4"), ("hint_dx", "<f4"), ("hint_dy", "<f4")])
QUAL = np.dtype([("t", "<i8"), ("seq", "<u4"), ("mean_luma", "<f4"), ("sat_frac", "<f4"), ("sharp", "<f4"),
                 ("sharp_base", "<f4"), ("raw", "u1"), ("flags", "u1"), ("pad", "u1", 2)])
TRUTH2 = np.dtype([("t", "<i8"), ("seq", "<u4"), ("obj", "<i4"), ("u", "<f4"), ("v", "<f4"), ("w", "<f4"), ("h", "<f4"),
                   ("visible", "u1"), ("pad", "u1", 3)])

# packet v2 link to the STM32 (U1): AIM frames sent, STATUS frames received
LINK_TX = np.dtype([("t", "<i8"), ("seq", "<u4"), ("lseq", "u1"), ("mode", "u1"), ("flags", "u1"), ("pad", "u1"),
                    ("err_pan", "<i2"), ("err_tilt", "<i2"), ("rate_pan", "<i2"), ("rate_tilt", "<i2")])
LINK_RX = np.dtype([("t", "<i8"), ("seq_echo", "u1"), ("guard", "u1"), ("flags", "u1"), ("pad", "u1"),
                    ("pan_goal", "<u2"), ("tilt_goal", "<u2"), ("rx_ok", "<u2"), ("rx_err", "<u2")])

TYPES = {1: ("frame", FRAME), 3: ("meas", MEAS), 4: ("est", EST), 5: ("state", STATE), 6: ("ncc", NCC), 8: ("truth", TRUTH),
         9: ("mot", MOT), 10: ("aim", AIM), 11: ("qual", QUAL), 12: ("truth2", TRUTH2), 13: ("link_tx", LINK_TX),
         14: ("link_rx", LINK_RX)}
GUARD_STATES = ["RUN", "HOLD", "TORQUE_OFF", "RAMP", "ESTOP"]
MOT_STATES = ["TENT", "CONF", "OVERLAP", "OCCL", "EXIT", "AMBIG", "LOST"]
AIM_MODES = ["HOLD", "FOLLOW", "PREDICT"]
AIM_REASONS = ["NO_TARGET", "ACQUIRING", "COAST_LONG", "REACQ", "QUALITY", "AMBIGUOUS", "OVERLAP", "EXITING", "NO_ESTIMATE"]
QUAL_FLAGS = ["DARK", "GLARE", "BLUR"]
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
    summary_v4(log)


def summary_v4(log):
    """Multi-target board, aim gate and image quality (v4)."""
    a = log["aim"]
    if len(a):
        occ = np.bincount(a["mode"], minlength=3) / len(a)
        print("aim output: " + "  ".join(f"{AIM_MODES[i]} {100 * occ[i]:.1f}%" for i in range(3)))
        held = a[a["mode"] == 0]
        if len(held):
            parts = []
            for b, name in enumerate(AIM_REASONS):
                n = int(np.count_nonzero(held["reasons"] & (1 << b)))
                if n:
                    parts.append(f"{name} {100 * n / len(held):.0f}%")
            print("  HOLD reasons (share of HOLD frames): " + ", ".join(parts))
        ids = sorted(set(int(i) for i in a["target_id"] if i >= 0))
        print(f"  followed track ids: {ids}")
    m = log["mot"]
    if len(m):
        ids = np.unique(m["id"])
        print(f"track board: {len(ids)} track ids, max {int(np.max(np.bincount(np.unique(m['t'], return_inverse=True)[1])))} at once")
        last = {}
        for r in m:
            i = int(r["id"])
            st = MOT_STATES[r["st"]]
            if last.get(i) != st:
                print(f"  t={r['t'] * 1e-9:9.3f}  track {i:3d} -> {st:<8} at ({r['u']:6.0f},{r['v']:5.0f})  prio {r['prio']:.2f}"
                      + ("  [lock]" if r["flags"] & 1 else ""))
                last[i] = st
    q = log["qual"]
    if len(q):
        parts = [f"{n} {100 * np.count_nonzero(q['flags'] & (1 << b)) / len(q):.1f}%" for b, n in enumerate(QUAL_FLAGS)]
        print(f"image quality ({len(q)} evaluations): mean luma median {np.median(q['mean_luma']):.0f}, flagged " + ", ".join(parts))
    summary_link(log)


def summary_link(log):
    """Packet v2 link: frames sent, STM32 guard states and link error counters."""
    tx, rx = log["link_tx"], log["link_rx"]
    if not len(tx) and not len(rx):
        return
    print(f"STM32 link: {len(tx)} AIM frames sent, {len(rx)} STATUS frames received")
    if len(rx):
        g = np.bincount(np.minimum(rx["guard"], 4), minlength=5) / len(rx)
        print("  STM32 guard: " + "  ".join(f"{GUARD_STATES[i]} {100 * g[i]:.1f}%" for i in range(5) if g[i] > 0))
        ok = (int(rx["rx_ok"][-1]) - int(rx["rx_ok"][0])) % 65536
        err = (int(rx["rx_err"][-1]) - int(rx["rx_err"][0])) % 65536
        print(f"  STM32 received {ok} valid AIM frames, {err} rejected during the run")
        if np.any(rx["flags"] & 5):
            print("  !! emergency stop seen (B1 or software)")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    summary(read_log(sys.argv[1]))
