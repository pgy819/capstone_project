#!/usr/bin/env python3
"""Capture an STS3215 identification run from the STM32 'sysid' firmware into CSV.

    python sysid_capture.py --port COM5 --axis pan --profile prbs [--max-speed] [--out pan_prbs.csv]

The firmware (stm32_fw, env 'sysid') streams 24-byte records at 500 Hz over the ST-LINK VCP
(921600 bps). Layout: see stm32_fw/include/sysid.h. The run ends when a record carries the
'profile finished' flag. CSV columns:

    t_s         tick time, s from the first record (TIM5)
    u           goal position written in this tick, counts (absolute, 2048 = centre)
    y           present position, counts (latest status packet)
    v           present speed, counts/s
    load        present load, 0.1 %
    t_meas_s    time the status packet was received, s (same origin as t_s)
    fresh       1 if y/v/load were updated since the previous record
    rx_err, overrun, max_speed, abort   flags (abort: firmware stall watch or B1 ESTOP)

The firmware refuses to run while it is LOCKED (a servo not answering, or the front pose not taught
with "Set home" in the tracking firmware); the tool then stops at once with that message.

Needs pyserial (pip install pyserial).
"""
import argparse
import csv
import struct
import sys
import time

REC_LEN = 24
SYNC = b"\xA5\x5A"
FMT = "<2sHIIhhhhBBBB"
PROFILES = {"step": b"S", "prbs": b"R", "chirp": b"C"}
AXES = {"pan": b"p", "tilt": b"t"}
F_FRESH, F_RXERR, F_DONE, F_OVERRUN, F_MAXSPD, F_ABORT = 0x01, 0x02, 0x04, 0x08, 0x10, 0x20
STOP_WORDS = ("refused", "LOCKED", "ESTOP")


def parse_stream(buf):
    """Split a byte buffer into (records, text_lines, remainder). Resyncs on bad checksums."""
    recs, text = [], []
    i = 0
    n = len(buf)
    while i < n:
        # firmware text lines are "# ...\r\n" and short; a '#' byte inside binary data is not
        if buf[i:i + 2] == b"# ":
            j = buf.find(b"\n", i, i + 120)
            if j >= 0:
                text.append(buf[i:j].decode("ascii", "replace").strip())
                i = j + 1
                continue
            if n - i < 120:
                break
        if buf[i:i + 2] == SYNC:
            if n - i < REC_LEN:
                break
            r = buf[i:i + REC_LEN]
            if (sum(r[2:23]) & 0xFF) == r[23]:
                f = struct.unpack(FMT, r)
                recs.append({"seq": f[1], "t_tick": f[2], "t_meas": f[3], "u": f[4], "y": f[5],
                             "v": f[6], "load": f[7], "flags": f[8],
                             "profile": f[9] >> 4, "axis": f[9] & 0x0F})
                i += REC_LEN
                continue
        i += 1
    return recs, text, buf[i:]


def to_rows(recs):
    """Unwrap the 32-bit microsecond timer and convert to CSV rows."""
    if not recs:
        return []
    t0 = recs[0]["t_tick"]
    rows = []
    for r in recs:
        f = r["flags"]
        rows.append({
            "t_s": ((r["t_tick"] - t0) & 0xFFFFFFFF) * 1e-6,
            "u": r["u"], "y": r["y"], "v": r["v"], "load": r["load"],
            "t_meas_s": (((r["t_meas"] - t0) + 2**31) % 2**32 - 2**31) * 1e-6,
            "fresh": int(bool(f & F_FRESH)), "rx_err": int(bool(f & F_RXERR)),
            "overrun": int(bool(f & F_OVERRUN)), "max_speed": int(bool(f & F_MAXSPD)),
            "abort": int(bool(f & F_ABORT)),
            "seq": r["seq"],
        })
    return rows


def write_csv(path, rows):
    keys = ["t_s", "u", "y", "v", "load", "t_meas_s", "fresh", "rx_err", "overrun", "max_speed", "abort", "seq"]
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def report(rows):
    n = len(rows)
    if n == 0:
        return "no records"
    seq = [r["seq"] for r in rows]
    gaps = sum(((b - a) & 0xFFFF) - 1 for a, b in zip(seq, seq[1:]))
    return (f"{n} records, {rows[-1]['t_s']:.1f} s, seq gaps {gaps}, "
            f"fresh {sum(r['fresh'] for r in rows) / n:.1%}, rx_err {sum(r['rx_err'] for r in rows)}, "
            f"overrun {sum(r['overrun'] for r in rows)}"
            + ("  ** ABORTED (stall or ESTOP): do not use this run **" if any(r["abort"] for r in rows) else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="ST-LINK VCP, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--axis", choices=AXES, default="pan")
    ap.add_argument("--profile", choices=PROFILES, default="prbs")
    ap.add_argument("--max-speed", action="store_true", help="goal speed 0 (servo max) instead of the tracking default")
    ap.add_argument("--out", help="CSV path (default <axis>_<profile>.csv)")
    ap.add_argument("--timeout", type=float, default=60.0)
    a = ap.parse_args()

    import serial  # pyserial

    out = a.out or f"{a.axis}_{a.profile}{'_max' if a.max_speed else ''}.csv"
    with serial.Serial(a.port, 921600, timeout=0.05) as ser:
        ser.reset_input_buffer()
        ser.write(b"X")
        time.sleep(1.0)
        ser.reset_input_buffer()
        ser.write(AXES[a.axis])
        time.sleep(0.05)
        # 'M' toggles; the firmware boots in default-speed mode
        if a.max_speed:
            ser.write(b"M")
            time.sleep(0.05)
        ser.write(PROFILES[a.profile])
        buf, recs = b"", []
        t_end = time.time() + a.timeout
        done = False
        refused = False
        while time.time() < t_end and not done and not refused:
            buf += ser.read(4096)
            new, text, buf = parse_stream(buf)
            for line in text:
                print(line)
                refused |= any(w in line for w in STOP_WORDS)
            recs.extend(new)
            done = any(r["flags"] & F_DONE for r in new)
            if new and len(recs) % 2500 < len(new):
                print(f"  {len(recs)} records", flush=True)
        if a.max_speed:
            ser.write(b"M")       # back to default for the next run
        if refused and not recs:
            print("firmware refused the run (see the '#' line above); nothing saved", file=sys.stderr)
            sys.exit(2)
        if not done:
            ser.write(b"X")
            print("timeout before the profile finished", file=sys.stderr)

    rows = to_rows(recs)
    write_csv(out, rows)
    print(report(rows))
    print("saved", out)


if __name__ == "__main__":
    main()
