#!/usr/bin/env python3
"""Synthetic STS3215 sysid captures, byte-for-byte in the firmware's record format.

Lets the whole chain (record parser -> CSV -> matlab/servo_sysid.m -> loop_design.m) be checked
before the servo is on the bench: the MATLAB script should recover the model written here.

    python sysid_synth.py --out-dir synth        # writes pan_step.csv, pan_prbs.csv, pan_chirp.csv

Truth model (edit below): 2nd order wn = 2*pi*6 Hz, zeta 0.8, 4 ms dead time, speed limit
300 deg/s, 1-count quantisation, 0.35 ms bus reply latency, 0.2 % missing replies.
"""
import argparse
import math
import os
import random
import struct


import sysid_capture as cap

FS = 500
TS = 1.0 / FS
C2D = 360.0 / 4096.0
PREROLL = 250
STEP_AMP = [25, 50, 100, 200, 400]
STEP_PHASE = 300
PRBS_AMP, PRBS_BIT, PRBS_PERIODS = 60, 10, 2
CHIRP_AMP, CHIRP_F0, CHIRP_F1, CHIRP_TICKS = 60.0, 0.2, 12.0, 15000

TRUTH = dict(wn=2 * math.pi * 6.0, zeta=0.8, dead=0.004, vmax_dps=300.0, reply=0.00035, loss=0.002)


def prbs_bit(b):
    s = 0x1FF
    for _ in range(b % 511):
        fb = ((s >> 8) ^ (s >> 4)) & 1
        s = ((s << 1) | fb) & 0x1FF
    return s & 1


def profile(prof, k):
    """Python mirror of SysId_Profile() in stm32_fw/src/sysid.c. Returns (offset_counts, done)."""
    if k < PREROLL:
        return 0, False
    t = k - PREROLL
    if prof == "step":
        per = 4 * STEP_PHASE
        a = t // per
        if a >= len(STEP_AMP):
            return 0, True
        ph = (t % per) // STEP_PHASE
        return (STEP_AMP[a] if ph == 0 else -STEP_AMP[a] if ph == 2 else 0), False
    if prof == "prbs":
        b = t // PRBS_BIT
        if b >= 511 * PRBS_PERIODS:
            return 0, True
        return (PRBS_AMP if prbs_bit(b) else -PRBS_AMP), False
    if prof == "chirp":
        if t >= CHIRP_TICKS:
            return 0, True
        T = CHIRP_TICKS / FS
        tt = t / FS
        r = math.log(CHIRP_F1 / CHIRP_F0)
        ph = 2 * math.pi * CHIRP_F0 * T / r * (math.exp(tt * r / T) - 1)
        return int(round(CHIRP_AMP * math.sin(ph))), False
    raise ValueError(prof)


def simulate(prof, seed=0):
    """Fine-step simulation (20 kHz) of the truth model driven by the 500 Hz goal."""
    rng = random.Random(seed)
    sub = 40
    dt = TS / sub
    wn, z = TRUTH["wn"], TRUTH["zeta"]
    vmax = TRUTH["vmax_dps"] / C2D
    dead_sub = int(round(TRUTH["dead"] / dt))
    pos, vel = 2048.0, 0.0
    goal_hist = [2048.0] * (dead_sub + 1)
    recs, k, t0 = [], 0, 1_000_000
    meas = None
    while True:
        off, done = profile(prof, k)
        goal = 2048 + off
        # servo answers the READ of tick k at t_k + reply latency, sampling the position then
        reply_at = int(TRUTH["reply"] / dt)
        sample = None
        for i in range(sub):
            goal_hist.append(goal)
            g = goal_hist[-1 - dead_sub]
            acc = wn * wn * (g - pos) - 2 * z * wn * vel
            vel = max(-vmax, min(vmax, vel + acc * dt))
            pos += vel * dt
            if i == reply_at:
                sample = (pos, vel)
        flags = 0
        if meas is not None:
            flags |= cap.F_FRESH
        if done:
            flags |= cap.F_DONE
        y, v, tm = meas if meas is not None else (2048, 0, t0)
        t_tick = t0 + int(round(k * TS * 1e6))
        body = struct.pack("<HIIhhhhBBB", k & 0xFFFF, t_tick, tm, goal, y, v, 0, flags,
                           ({"step": 1, "prbs": 2, "chirp": 3}[prof] << 4) | 1, 0)
        rec = cap.SYNC + body + bytes([sum(body) & 0xFF])
        recs.append(rec)
        if rng.random() < TRUTH["loss"]:
            meas = None
        else:
            meas = (int(round(sample[0])), int(round(sample[1])), t_tick + int(TRUTH["reply"] * 1e6))
        k += 1
        if done:
            break
    return b"".join(recs)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default="synth")
    ap.add_argument("--axis", default="pan")
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    for i, prof in enumerate(("step", "prbs", "chirp")):
        stream = b"# start " + prof.encode() + b"\r\n" + simulate(prof, seed=i)
        recs, text, rest = cap.parse_stream(stream)
        rows = cap.to_rows(recs)
        path = os.path.join(a.out_dir, f"{a.axis}_{prof}.csv")
        cap.write_csv(path, rows)
        print(f"{path}: {cap.report(rows)}")
    print("truth:", {k: round(v, 4) for k, v in TRUTH.items()},
          f"(wn = {TRUTH['wn'] / 2 / math.pi:.1f} Hz)")


if __name__ == "__main__":
    main()
