#!/usr/bin/env python3
"""Self-test for tools/sysid_capture.py (record parser) and tools/sysid_synth.py.

    python3 tools/tests/test_sysid.py        (standard library only)

Checks: record layout and checksum, resync after garbage and torn records, '#' text lines,
32-bit timer unwrap, profile lengths, and that the synthetic captures are complete.
"""
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import sysid_capture as cap  # noqa: E402
import sysid_synth as syn  # noqa: E402

FAILS = 0


def check(cond, msg):
    global FAILS
    print(("ok   " if cond else "FAIL ") + msg)
    if not cond:
        FAILS += 1


def record(seq, t_tick, t_meas, u, y, v=0, load=0, flags=cap.F_FRESH, prof_axis=0x21):
    body = struct.pack("<HIIhhhhBBB", seq, t_tick, t_meas, u, y, v, load, flags, prof_axis, 0)
    return cap.SYNC + body + bytes([sum(body) & 0xFF])


def main():
    r0 = record(1, 100, 90, 2100, 2090, -5, 12)
    check(len(r0) == cap.REC_LEN, "record is 24 bytes")

    # garbage, text line, torn record, two good records
    stream = b"\x00\x13\xA5" + b"# start prbs\r\n" + r0[:10] + r0 + record(2, 2100, 2050, 2110, 2095)
    recs, text, rest = cap.parse_stream(stream)
    check(len(recs) == 2 and recs[0]["u"] == 2100 and recs[1]["seq"] == 2, "resync to 2 good records")
    check(text == ["# start prbs"], "text line extracted")
    check(rest == b"", "nothing left over")

    bad = bytearray(r0)
    bad[14] ^= 0x40
    recs, _, _ = cap.parse_stream(bytes(bad) + r0)
    check(len(recs) == 1 and recs[0]["y"] == 2090, "bad checksum skipped")

    recs, _, rest = cap.parse_stream(r0 + r0[:7])
    check(len(recs) == 1 and rest == r0[:7], "partial record kept for the next read")

    # 32-bit timer wrap between records
    a = record(10, 0xFFFFFF00, 0xFFFFFEF0, 2048, 2048)
    b = record(11, 0x00000100, 0xFFFFFF80, 2048, 2048)
    rows = cap.to_rows(cap.parse_stream(a + b)[0])
    check(abs(rows[1]["t_s"] - 512e-6) < 1e-9, "tick timer unwrap")
    check(abs(rows[1]["t_meas_s"] - 0x80 * 1e-6) < 1e-9, "measurement timer unwrap (before tick 0 origin)")

    # profiles: lengths in ticks (incl. 0.5 s pre-roll)
    def length(prof):
        k = 0
        while not syn.profile(prof, k)[1]:
            k += 1
        return k + 1
    check(length("step") == 250 + 5 * 4 * 300 + 1, "step series length")
    check(length("prbs") == 250 + 511 * 2 * 10 + 1, "prbs length")
    check(length("chirp") == 250 + 15000 + 1, "chirp length")
    bits = [syn.prbs_bit(i) for i in range(511)]
    check(sum(bits) == 256, "PRBS-9 has 256 ones per period")
    check(max(abs(syn.profile("chirp", k)[0]) for k in range(250, 15250)) == 60, "chirp amplitude 60 counts")

    # synthetic capture end to end
    recs, _, _ = cap.parse_stream(syn.simulate("prbs", seed=3))
    rows = cap.to_rows(recs)
    seq = [r["seq"] for r in rows]
    check(len(rows) == length("prbs") and all((b - a) & 0xFFFF == 1 for a, b in zip(seq, seq[1:])),
          "synthetic PRBS capture complete, no sequence gaps")
    check(0.99 < sum(r["fresh"] for r in rows) / len(rows) <= 1.0, "about 0.2 % missing replies")

    print(f"\n{FAILS} failed")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
