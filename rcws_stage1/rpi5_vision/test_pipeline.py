#!/usr/bin/env python3
"""
test_pipeline.py — Stage 1 automated integration verification
=============================================================

Run:  python3 rpi5_vision/test_pipeline.py      (exit code 0 == all pass)

Covers
  1. Kalman: spec matrices, first-detection sync, dt clamp, circular motion,
     sharp manoeuvre, 3-frame coasting, 6-frame lost reset, conditional lead,
     deadzone / clamp.
  2. 8-byte LE packet serialization + checksum contract.
  3. Byte-stream FSM parsing (Python reference model of protocol.c): sticky 0xFF
     (FF FF FE ...), garbage, corrupted checksum, byte slip, 64-byte DMA ring wrap.
  4. TrackerCore: "cell phone" / confidence filter, nearest lock-on, bbox mapping.
  5. STM32 PD loop reference model: first-frame derivative-kick guard, watchdog hold.
  6. End-to-end: synthetic phone -> TrackerCore -> UART bytes (+noise) -> FSM -> PD.
  7. Optional C cross-verification (gcc/clang + ctypes) of protocol.c / feetech_sts.c.
"""

from __future__ import annotations

import ctypes
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import unittest
from typing import List, Optional, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FW_DIR = os.path.join(ROOT, "stm32_fw")
sys.path.insert(0, HERE)

from kalman_filter import (  # noqa: E402
    KalmanTracker2D, STATUS_COASTING, STATUS_DETECTED, STATUS_LOST,
)
from uart_tx import build_packet, compute_checksum, parse_packet  # noqa: E402
from tracker_node import (  # noqa: E402
    Detection, TrackerCore, bbox_to_error, virtual_to_frame,
)

FPS = 30.0
FRAME_DT = 1.0 / FPS


# =========================================================================== #
# Helpers
# =========================================================================== #
class FakeClock:
    """Deterministic monotonic clock for KalmanTracker2D."""

    def __init__(self, t0: float = 1000.0) -> None:
        self.t = t0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def make_tracker() -> Tuple[KalmanTracker2D, FakeClock]:
    clk = FakeClock()
    return KalmanTracker2D(clock=clk), clk


def step(kf: KalmanTracker2D, clk: FakeClock, meas, dt: float = FRAME_DT):
    clk.advance(dt)
    return kf.update(meas)


class ReferenceFsmParser:
    """Bit-exact Python model of stm32_fw/src/protocol.c Protocol_ParseByte()."""
    WAIT_FF, WAIT_FE, PAYLOAD = 0, 1, 2

    def __init__(self) -> None:
        self.state = self.WAIT_FF
        self.payload: List[int] = []
        self.ok = 0
        self.err = 0

    def feed(self, b: int) -> Optional[Tuple[int, int, int]]:
        if self.state == self.WAIT_FF:
            if b == 0xFF:
                self.state = self.WAIT_FE
        elif self.state == self.WAIT_FE:
            if b == 0xFE:
                self.state = self.PAYLOAD
                self.payload = []
            elif b == 0xFF:
                self.state = self.WAIT_FE          # sticky 0xFF
            else:
                self.state = self.WAIT_FF
        else:
            self.payload.append(b)
            if len(self.payload) == 6:
                p = self.payload
                self.state = self.WAIT_FF
                if (p[0] + p[1] + p[2] + p[3] + p[4]) & 0xFF == p[5]:
                    ex = int.from_bytes(bytes(p[1:3]), "little", signed=True)
                    ey = int.from_bytes(bytes(p[3:5]), "little", signed=True)
                    if p[0] <= 2 and -960 <= ex <= 960 and -540 <= ey <= 540:
                        self.ok += 1
                        return p[0], ex, ey
                self.err += 1
        return None

    def feed_all(self, data: bytes) -> List[Tuple[int, int, int]]:
        out = []
        for b in data:
            r = self.feed(b)
            if r is not None:
                out.append(r)
        return out


class DmaRingModel:
    """Model of the 64-byte circular DMA buffer + Protocol_Poll() head/tail logic."""
    SIZE = 64

    def __init__(self, parser) -> None:
        self.buf = bytearray(self.SIZE)
        self.write_pos = 0
        self.tail = 0
        self.parser = parser

    def dma_receive(self, chunk: bytes) -> None:
        for b in chunk:
            self.buf[self.write_pos] = b
            self.write_pos = (self.write_pos + 1) % self.SIZE

    def ndtr(self) -> int:
        """DMA NDTR register: counts down 64..1 then reloads to 64."""
        return self.SIZE - self.write_pos

    def poll(self) -> List[Tuple[int, int, int]]:
        head = (self.SIZE - self.ndtr()) & 0x3F
        out = []
        while self.tail != head:
            r = self.parser.feed(self.buf[self.tail])
            if r is not None:
                out.append(r)
            self.tail = (self.tail + 1) & 0x3F
        return out


class ServoLoopModel:
    """Python model of main.c Servo_ControlStep() (500 Hz)."""
    KP, KD = 0.0025, 0.015
    DIR_PAN, DIR_TILT = 1.0, -1.0
    WD_LIMIT = 250

    def __init__(self) -> None:
        self.pan = 2048.0
        self.tilt = 2048.0
        self.dx = 0.0
        self.dy = 0.0
        self.prev_x = 0
        self.prev_y = 0
        self.first = True
        self.watchdog = self.WD_LIMIT + 1
        self.pkt = (0, 0, 0)
        self.new_flag = False
        self.last_raw_dex = None

    def on_packet(self, pkt: Tuple[int, int, int]) -> None:
        self.pkt = pkt
        self.new_flag = True
        self.watchdog = 0

    def tick(self) -> Tuple[int, int]:
        status, ex, ey = self.pkt
        new_flag, self.new_flag = self.new_flag, False
        self.watchdog += 1
        self.last_raw_dex = None
        if self.watchdog > self.WD_LIMIT or status not in (1, 2):
            self.dx = self.dy = 0.0
            self.first = True
        else:
            if new_flag:
                if self.first:
                    rdx = rdy = 0.0
                    self.first = False
                else:
                    rdx, rdy = float(ex - self.prev_x), float(ey - self.prev_y)
                self.last_raw_dex = rdx
                self.prev_x, self.prev_y = ex, ey
                self.dx = 0.6 * self.dx + 0.4 * rdx
                self.dy = 0.6 * self.dy + 0.4 * rdy
            else:
                self.dx *= 0.85
                self.dy *= 0.85
            self.pan += self.DIR_PAN * (self.KP * ex + self.KD * self.dx)
            self.tilt += self.DIR_TILT * (self.KP * ey + self.KD * self.dy)
        self.pan = min(max(self.pan, 512.0), 3584.0)
        self.tilt = min(max(self.tilt, 1536.0), 2560.0)
        return int(self.pan + 0.5), int(self.tilt + 0.5)


def py_sync_write(pan: int, pspd: int, tilt: int, tspd: int) -> bytes:
    """Python reference of STS_BuildSyncWritePosSpeed()."""
    pan, tilt = min(pan, 4095), min(tilt, 4095)
    b = [0xFF, 0xFF, 0xFE, 0x12, 0x83, 0x2A, 0x06,
         0x01, pan & 0xFF, pan >> 8, 0, 0, pspd & 0xFF, pspd >> 8,
         0x02, tilt & 0xFF, tilt >> 8, 0, 0, tspd & 0xFF, tspd >> 8]
    b.append(~sum(b[2:21]) & 0xFF)
    return bytes(b)


def phone_detection(ex: float, ey: float, conf: float = 0.9,
                    label: str = "cell phone", w: float = 0.06, h: float = 0.12) -> Detection:
    """Create a normalized Hailo-style bbox centred at virtual error (ex, ey)."""
    cx = (ex + 960.0) / 1920.0
    cy = (ey + 540.0) / 1080.0
    return Detection(label, conf, cx - w / 2.0, cy - h / 2.0, w, h)


# =========================================================================== #
# 1. Kalman filter
# =========================================================================== #
class TestKalmanFilter(unittest.TestCase):

    def test_spec_matrices(self):
        kf, _ = make_tracker()
        for m in (kf.P0, kf.Q, kf.R, kf.H):
            self.assertEqual(m.dtype, np.float32)
        np.testing.assert_array_equal(np.diag(kf.P0), [100, 100, 1000, 1000])
        np.testing.assert_array_equal(np.diag(kf.Q), [2, 2, 50, 50])
        np.testing.assert_array_equal(np.diag(kf.R), [25, 25])
        np.testing.assert_array_equal(kf.H, [[1, 0, 0, 0], [0, 1, 0, 0]])

    def test_first_detection_no_velocity_spike(self):
        kf, clk = make_tracker()
        out = step(kf, clk, (500, -300))
        self.assertEqual(out.status, STATUS_DETECTED)
        self.assertTrue(kf.is_tracking)
        self.assertEqual((out.est_vx, out.est_vy), (0.0, 0.0))
        self.assertEqual((out.pred_ex, out.pred_ey), (500, -300))
        np.testing.assert_array_equal(kf.P, kf.P0)
        # second identical measurement: velocity must stay tiny (no spike from (0,0))
        out = step(kf, clk, (500, -300))
        self.assertLess(abs(out.est_vx), 1.0)
        self.assertLess(abs(out.est_vy), 1.0)

    def test_dynamic_dt_clamping(self):
        kf, clk = make_tracker()
        self.assertAlmostEqual(step(kf, clk, (0, 0), dt=0.001).dt, 0.005, places=6)
        self.assertAlmostEqual(step(kf, clk, (0, 0), dt=1.000).dt, 0.100, places=6)
        self.assertAlmostEqual(step(kf, clk, (0, 0), dt=0.033).dt, 0.033, places=6)
        self.assertAlmostEqual(float(kf.F[0, 2]), 0.033, places=6)

    def _run_circle(self, radius: float, freq_hz: float, frames: int = 300):
        """Phone on a circle at 30 fps with sigma = 3 px detector noise."""
        kf, clk = make_tracker()
        rng = np.random.default_rng(1)
        w = 2 * math.pi * freq_hz
        pos_err, vel_err = [], []
        for k in range(frames):
            t = k * FRAME_DT
            tx, ty = radius * math.cos(w * t), radius * math.sin(w * t)
            meas = (int(tx + rng.normal(0, 3)), int(ty + rng.normal(0, 3)))
            out = step(kf, clk, meas)
            self.assertEqual(out.status, STATUS_DETECTED)
            self.assertTrue(np.all(np.isfinite(kf.X)))
            if k >= 30:  # after convergence
                pos_err.append(math.hypot(out.est_ex - tx, out.est_ey - ty))
                vtx, vty = -radius * w * math.sin(w * t), radius * w * math.cos(w * t)
                vel_err.append(math.hypot(out.est_vx - vtx, out.est_vy - vty))
        rms = lambda a: math.sqrt(sum(e * e for e in a) / len(a))  # noqa: E731
        return rms(pos_err), max(pos_err), rms(vel_err)

    def test_circular_motion_tracking(self):
        """Hand-held phone circle: r = 200 px, 0.25 Hz (314 px/s)."""
        pos_rms, pos_max, vel_rms = self._run_circle(200.0, 0.25)
        self.assertLess(pos_rms, 12.0, f"position RMS {pos_rms:.1f}px")
        self.assertLess(pos_max, 16.0, f"position max {pos_max:.1f}px")
        self.assertLess(vel_rms, 180.0, f"velocity RMS {vel_rms:.1f}px/s")

    def test_circular_motion_aggressive_bounded(self):
        """Aggressive circle: r = 300 px, 0.5 Hz (942 px/s, ~2960 px/s^2).

        With the spec-fixed Q (velocity noise 50, not dt-scaled) a CV model lags on
        high centripetal acceleration (~47 px RMS). Verify it stays bounded/stable.
        """
        pos_rms, pos_max, _ = self._run_circle(300.0, 0.5)
        self.assertLess(pos_rms, 55.0, f"position RMS {pos_rms:.1f}px")
        self.assertLess(pos_max, 60.0, f"position max {pos_max:.1f}px")

    def test_sharp_maneuver(self):
        """+800 px/s then instant reversal to -800 px/s: velocity must flip, stay bounded."""
        kf, clk = make_tracker()
        x = -400.0
        flip_frame = None
        peak_err = 0.0
        for k in range(90):
            v = 800.0 if k < 30 else -800.0
            x += v * FRAME_DT
            out = step(kf, clk, (int(x), 0))
            self.assertTrue(np.all(np.isfinite(kf.X)))
            self.assertTrue(-960 <= out.pred_ex <= 960 and -540 <= out.pred_ey <= 540)
            peak_err = max(peak_err, abs(out.est_ex - x))
            if k >= 30 and flip_frame is None and out.est_vx < 0:
                flip_frame = k - 30
            if k >= 45:
                self.assertLess(abs(out.est_ex - x), 30.0)
                self.assertLess(abs(out.est_vx + 800.0), 250.0)
        self.assertIsNotNone(flip_frame)
        # Measured with spec P0/Q/R: flip after 7 frames (~233 ms), peak lag ~75 px.
        self.assertLessEqual(flip_frame, 8, f"velocity sign flipped after {flip_frame} frames")
        self.assertLess(peak_err, 90.0, f"peak position error {peak_err:.1f}px")

    def test_coasting_3_missed_frames(self):
        kf, clk = make_tracker()
        x = 0.0
        for _ in range(30):
            x += 600 * FRAME_DT
            step(kf, clk, (int(x), 50))
        last_pred = kf.pred_ex
        for i in range(3):
            out = step(kf, clk, None)
            self.assertEqual(out.status, STATUS_COASTING)
            self.assertTrue(kf.is_tracking)
            self.assertEqual(kf.coast_counter, i + 1)
            self.assertGreater(out.pred_ex, last_pred)  # keeps extrapolating
            last_pred = out.pred_ex
        x += 4 * 600 * FRAME_DT
        out = step(kf, clk, (int(x), 50))
        self.assertEqual(out.status, STATUS_DETECTED)
        self.assertEqual(kf.coast_counter, 0)
        self.assertGreater(out.est_vx, 300.0)          # not re-initialised

    def test_lost_reset_after_6_missed_frames(self):
        kf, clk = make_tracker()
        for k in range(20):
            step(kf, clk, (100 + 10 * k, -80))
        for i in range(5):
            self.assertEqual(step(kf, clk, None).status, STATUS_COASTING, f"miss {i + 1}")
        out = step(kf, clk, None)  # 6th miss
        self.assertEqual(out.status, STATUS_LOST)
        self.assertEqual((out.pred_ex, out.pred_ey), (0, 0))
        self.assertFalse(kf.is_tracking)
        np.testing.assert_array_equal(kf.X, np.zeros((4, 1), np.float32))
        np.testing.assert_array_equal(kf.P, kf.P0)
        self.assertEqual(step(kf, clk, None).status, STATUS_LOST)
        # re-acquire: direct sync, zero velocity
        out = step(kf, clk, (-250, 120))
        self.assertEqual(out.status, STATUS_DETECTED)
        self.assertEqual((out.pred_ex, out.pred_ey, out.est_vx), (-250, 120, 0.0))

    def test_conditional_lead(self):
        cp = KalmanTracker2D.compute_prediction
        # diverging (e*v > 0) -> 45 ms
        self.assertEqual(cp(100.0, -100.0, 1000.0, -1000.0), (145, -145))
        # converging (e*v < 0) -> 15 ms
        self.assertEqual(cp(100.0, -100.0, -1000.0, 1000.0), (85, -85))

    def test_deadzone_and_clamp(self):
        cp = KalmanTracker2D.compute_prediction
        self.assertEqual(cp(5.0, -5.4, 0.0, 0.0), (0, 0))
        self.assertEqual(cp(6.0, -6.0, 0.0, 0.0), (6, -6))
        self.assertEqual(cp(950.0, -530.0, 2000.0, -2000.0), (960, -540))
        kf, clk = make_tracker()
        self.assertEqual(step(kf, clk, (3, -4)).pred_ex, 0)


# =========================================================================== #
# 2. Packet serialization
# =========================================================================== #
class TestPacket(unittest.TestCase):

    def test_layout_little_endian(self):
        pkt = build_packet(0x01, 300, -200)
        self.assertEqual(len(pkt), 8)
        self.assertEqual(pkt[:3], b"\xFF\xFE\x01")
        self.assertEqual(pkt[3:5], (300).to_bytes(2, "little", signed=True))
        self.assertEqual(pkt[5:7], (-200).to_bytes(2, "little", signed=True))
        self.assertEqual(pkt[7], (0x01 + 0x2C + 0x01 + 0x38 + 0xFF) & 0xFF)

    def test_checksum_contract(self):
        for st, ex, ey in [(0, 0, 0), (2, -960, 540), (1, -1, -1), (1, 960, -540)]:
            ux, uy = ex & 0xFFFF, ey & 0xFFFF
            ref = (st + (ux & 0xFF) + (ux >> 8) + (uy & 0xFF) + (uy >> 8)) & 0xFF
            self.assertEqual(compute_checksum(st, ex, ey), ref)
            self.assertEqual(build_packet(st, ex, ey)[7], ref)

    def test_clamping(self):
        self.assertEqual(parse_packet(build_packet(1, 5000, -5000)), (1, 960, -540))

    def test_round_trip_random(self):
        rng = random.Random(7)
        for _ in range(2000):
            v = (rng.choice([0, 1, 2]), rng.randint(-960, 960), rng.randint(-540, 540))
            self.assertEqual(parse_packet(build_packet(*v)), v)
        bad = bytearray(build_packet(1, 10, 10))
        bad[7] ^= 0x01
        self.assertIsNone(parse_packet(bytes(bad)))


# =========================================================================== #
# 3. Byte-stream FSM parsing
# =========================================================================== #
class FsmCases:
    """Shared FSM scenarios; subclasses provide make_parser()."""

    def make_parser(self):
        raise NotImplementedError

    def test_sticky_ff(self):
        p = self.make_parser()
        pkt = build_packet(0x01, 123, -45)
        stream = b"\xFF\xFF" + pkt[1:]                       # FF FF FE ...
        self.assertEqual(p.feed_all(stream), [(1, 123, -45)])
        stream = b"\xFF\xFF\xFF\xFF" + pkt[1:]              # many FF
        self.assertEqual(p.feed_all(stream), [(1, 123, -45)])

    def test_payload_containing_ff(self):
        p = self.make_parser()
        pkts = [(1, -1, -1), (2, -2, 255), (1, 255, -257)]   # 0xFF bytes in payload
        stream = b"".join(build_packet(*v) for v in pkts)
        self.assertEqual(p.feed_all(stream), pkts)

    def test_garbage_and_corruption(self):
        p = self.make_parser()
        good1, good2 = build_packet(1, 10, 20), build_packet(2, -30, 40)
        bad = bytearray(build_packet(1, 50, 60))
        bad[4] ^= 0x10
        stream = b"\x00\x13\xFE\xFF\x00" + good1 + bytes(bad) + b"\xFE\xFE" + good2
        self.assertEqual(p.feed_all(stream), [(1, 10, 20), (2, -30, 40)])

    def test_byte_slip_recovery(self):
        p = self.make_parser()
        pkts = [(1, i * 7 - 300, 200 - i * 5) for i in range(12)]
        raw = [build_packet(*v) for v in pkts]
        stream = raw[0] + raw[1][:5] + b"".join(raw[2:])     # truncated packet #1
        got = p.feed_all(stream)
        self.assertTrue(set(got) <= set(pkts), "parser produced a non-sent packet")
        self.assertEqual(got[0], pkts[0])
        self.assertEqual(got[-9:], pkts[-9:], "must resynchronise within 2 packets")

    def test_random_noise_stream(self):
        p = self.make_parser()
        rng = random.Random(42)
        sent, stream = [], bytearray()
        for _ in range(500):
            if rng.random() < 0.3:
                stream += bytes(rng.choice([0xFF, 0xFE, rng.randint(0, 255)])
                                for _ in range(rng.randint(1, 5)))
            v = (rng.choice([0, 1, 2]), rng.randint(-960, 960), rng.randint(-540, 540))
            sent.append(v)
            stream += build_packet(*v)
        got = p.feed_all(bytes(stream))
        self.assertTrue(set(got) <= set(sent))
        self.assertGreaterEqual(len(got), int(0.9 * len(sent)))


class TestReferenceFsm(FsmCases, unittest.TestCase):

    def make_parser(self):
        return ReferenceFsmParser()

    def test_dma_ring_buffer_wrap(self):
        """64-byte circular DMA buffer with irregular arrival / polling."""
        parser = ReferenceFsmParser()
        ring = DmaRingModel(parser)
        rng = random.Random(3)
        sent = [(rng.choice([1, 2]), rng.randint(-960, 960), rng.randint(-540, 540))
                for _ in range(300)]
        stream = b"\xFF" + b"".join(build_packet(*v) for v in sent)
        got, i = [], 0
        while i < len(stream):
            n = rng.randint(1, 40)                           # < 64: no overrun
            ring.dma_receive(stream[i:i + n])
            i += n
            got += ring.poll()
        self.assertEqual(got, sent)


# =========================================================================== #
# 4. TrackerCore (target filter + lock-on)
# =========================================================================== #
class TestTrackerCore(unittest.TestCase):

    def test_bbox_conversion(self):
        self.assertEqual(bbox_to_error(0.45, 0.45, 0.1, 0.1), (0, 0))
        self.assertEqual(bbox_to_error(0.0, 0.0, 0.0, 0.0), (-960, -540))
        self.assertEqual(bbox_to_error(0.7, 0.2, 0.1, 0.2), (int(0.75 * 1920) - 960, int(0.3 * 1080) - 540))
        self.assertEqual(virtual_to_frame(0, 0, 1280, 720), (640, 360))

    def test_label_and_confidence_filter(self):
        kf, clk = make_tracker()
        core = TrackerCore(kf)
        dets = [phone_detection(100, 100, label="person"),
                phone_detection(-200, 50, conf=0.44),
                phone_detection(300, -100, conf=0.45)]
        clk.advance(FRAME_DT)
        res = core.process(dets)
        self.assertEqual(res.status, STATUS_DETECTED)
        self.assertEqual(res.raw, (300, -100))
        clk.advance(FRAME_DT)
        self.assertIsNone(core.process(dets[:2]).raw)

    def test_lock_on_nearest_to_prediction(self):
        kf, clk = make_tracker()
        core = TrackerCore(kf)
        # Lost -> reference (0, 0): the box nearest the centre wins.
        clk.advance(FRAME_DT)
        res = core.process([phone_detection(400, 300), phone_detection(-80, 40)])
        self.assertEqual(res.raw, (-80, 40))
        # Track it moving right; a distractor near the centre must be ignored.
        x = -80
        for _ in range(15):
            x += 15
            clk.advance(FRAME_DT)
            res = core.process([phone_detection(x, 40), phone_detection(-500, -400),
                                phone_detection(600, 300)])
            self.assertEqual(res.raw, bbox_to_error(*_bbox(phone_detection(x, 40))))
        self.assertEqual(res.status, STATUS_DETECTED)


def _bbox(d: Detection):
    return d.xmin, d.ymin, d.width, d.height


# =========================================================================== #
# 5. STM32 PD loop model
# =========================================================================== #
class TestServoLoopModel(unittest.TestCase):

    def test_boot_timeout_holds_center(self):
        s = ServoLoopModel()
        for _ in range(100):
            self.assertEqual(s.tick(), (2048, 2048))

    def test_first_frame_no_derivative_kick(self):
        s = ServoLoopModel()
        s.on_packet((1, 400, 0))
        s.tick()
        self.assertEqual(s.last_raw_dex, 0.0)
        self.assertAlmostEqual(s.dx, 0.0)
        s.on_packet((1, 410, 0))
        s.tick()
        self.assertEqual(s.last_raw_dex, 10.0)

    def test_watchdog_and_lost_hold(self):
        s = ServoLoopModel()
        s.on_packet((1, 200, 100))
        for _ in range(250):
            s.tick()
        held = s.tick()                                    # 251st tick -> timeout
        self.assertTrue(s.first)
        self.assertEqual(s.dx, 0.0)
        for _ in range(50):
            self.assertEqual(s.tick(), held)
        s.on_packet((0, 0, 0))                             # lost -> hold
        self.assertEqual(s.tick(), held)

    def test_direction_and_limits(self):
        s = ServoLoopModel()
        s.on_packet((1, 500, 300))
        for _ in range(10):
            s.tick()
        self.assertGreater(s.pan, 2048)                    # DIR_PAN  = +1
        self.assertLess(s.tilt, 2048)                      # DIR_TILT = -1
        for _ in range(20000):
            s.watchdog = 0
            s.tick()
        self.assertEqual((s.pan, s.tilt), (3584.0, 1536.0))


# =========================================================================== #
# 6. End-to-end pipeline
# =========================================================================== #
class TestEndToEnd(unittest.TestCase):

    def test_full_pipeline(self):
        kf, clk = make_tracker()
        core = TrackerCore(kf)
        parser = ReferenceFsmParser()
        ring = DmaRingModel(parser)
        servo = ServoLoopModel()
        rng = random.Random(11)

        sent, received, statuses = [], [], []
        for k in range(150):
            t = k * FRAME_DT
            ex, ey = 250 * math.cos(math.pi * t), 150 * math.sin(math.pi * t)
            dets = [] if 60 <= k < 63 or 100 <= k < 107 else [phone_detection(ex, ey)]
            if k % 10 == 0:
                dets.append(phone_detection(-700, 400, label="person"))
            clk.advance(FRAME_DT)
            res = core.process(dets)
            statuses.append(res.status)
            sent.append(parse_packet(res.packet))
            noise = b"\xFF" * rng.randint(0, 3)              # sticky 0xFF line noise
            ring.dma_receive(noise + res.packet)
            for pkt in ring.poll():
                received.append(pkt)
                servo.on_packet(pkt)
            for _ in range(int(500 / FPS)):                  # ~16 servo ticks per frame
                servo.tick()

        self.assertEqual(received, sent)
        self.assertEqual(statuses[60:63], [STATUS_COASTING] * 3)
        self.assertEqual(statuses[100:105], [STATUS_COASTING] * 5)
        self.assertEqual(statuses[105:107], [STATUS_LOST] * 2)
        self.assertEqual(statuses[107], STATUS_DETECTED)
        self.assertTrue(512 <= servo.pan <= 3584 and 1536 <= servo.tilt <= 2560)


# =========================================================================== #
# 7. Optional C cross-verification (gcc / clang + ctypes)
# =========================================================================== #
def _find_c_compiler() -> Optional[str]:
    for cc in (os.environ.get("CC"), "gcc", "clang", "cc"):
        if cc and shutil.which(cc):
            return cc
    return None


_CLIB = None
_CLIB_ERR = "no host C compiler (gcc/clang) found"


def _load_c_lib():
    global _CLIB, _CLIB_ERR
    if _CLIB is not None:
        return _CLIB
    cc = _find_c_compiler()
    if cc is None:
        return None
    out_dir = tempfile.mkdtemp(prefix="rcws_c_")
    ext = ".dll" if sys.platform == "win32" else (".dylib" if sys.platform == "darwin" else ".so")
    lib_path = os.path.join(out_dir, "rcws_host" + ext)
    cmd = [cc, "-shared", "-O2", "-Wall", "-std=c11", "-DRCWS_HOST_TEST",
           "-I", os.path.join(FW_DIR, "include"),
           os.path.join(FW_DIR, "src", "protocol.c"),
           os.path.join(FW_DIR, "src", "feetech_sts.c"), "-o", lib_path]
    if sys.platform != "win32":
        cmd.insert(2, "-fPIC")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        _CLIB_ERR = "C compile failed:\n" + proc.stderr
        raise RuntimeError(_CLIB_ERR)
    lib = ctypes.CDLL(lib_path)
    lib.Protocol_Reset.restype = None
    lib.Protocol_ParseByte.argtypes = [ctypes.c_uint8]
    lib.Protocol_ParseByte.restype = None
    lib.Protocol_HostTakePacket.argtypes = [ctypes.POINTER(ctypes.c_uint8),
                                            ctypes.POINTER(ctypes.c_int16),
                                            ctypes.POINTER(ctypes.c_int16)]
    lib.Protocol_HostTakePacket.restype = ctypes.c_uint8
    lib.STS_BuildSyncWritePosSpeed.argtypes = [ctypes.POINTER(ctypes.c_uint8)] + [ctypes.c_uint16] * 4
    lib.STS_BuildSyncWritePosSpeed.restype = ctypes.c_uint8
    lib.STS_BuildWriteByte.argtypes = [ctypes.POINTER(ctypes.c_uint8)] + [ctypes.c_uint8] * 3
    lib.STS_BuildWriteByte.restype = ctypes.c_uint8
    _CLIB = lib
    return lib


class CParserAdapter:
    def __init__(self, lib) -> None:
        self.lib = lib
        lib.Protocol_Reset()
        st, ex, ey = ctypes.c_uint8(), ctypes.c_int16(), ctypes.c_int16()
        while lib.Protocol_HostTakePacket(ctypes.byref(st), ctypes.byref(ex), ctypes.byref(ey)):
            pass
        self._st, self._ex, self._ey = st, ex, ey

    def feed_all(self, data: bytes):
        out = []
        for b in data:
            self.lib.Protocol_ParseByte(b)
            if self.lib.Protocol_HostTakePacket(ctypes.byref(self._st), ctypes.byref(self._ex),
                                                ctypes.byref(self._ey)):
                out.append((self._st.value, self._ex.value, self._ey.value))
        return out


class TestCCrossVerification(FsmCases, unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.lib = _load_c_lib()
        if cls.lib is None:
            raise unittest.SkipTest(_CLIB_ERR)

    def make_parser(self):
        return CParserAdapter(self.lib)

    def test_c_matches_python_reference_fuzz(self):
        rng = random.Random(99)
        data = bytes(rng.choice([0xFF, 0xFE, 0x00, 0x01, 0x02, rng.randint(0, 255)])
                     for _ in range(20000))
        data += b"".join(build_packet(rng.choice([0, 1, 2]), rng.randint(-960, 960),
                                      rng.randint(-540, 540)) for _ in range(500))
        self.assertEqual(CParserAdapter(self.lib).feed_all(data),
                         ReferenceFsmParser().feed_all(data))

    def test_sync_write_22_bytes(self):
        buf = (ctypes.c_uint8 * 22)()
        for pan, tilt in [(2048, 2048), (512, 2560), (3584, 1536), (5000, 0)]:
            n = self.lib.STS_BuildSyncWritePosSpeed(buf, pan, 3400, tilt, 3400)
            self.assertEqual(n, 22)
            self.assertEqual(bytes(buf), py_sync_write(pan, 3400, tilt, 3400))

    def test_write_byte_packet(self):
        buf = (ctypes.c_uint8 * 8)()
        self.assertEqual(self.lib.STS_BuildWriteByte(buf, 1, 0x05, 2), 8)
        exp = [0xFF, 0xFF, 0x01, 0x04, 0x03, 0x05, 0x02]
        exp.append(~sum(exp[2:]) & 0xFF)
        self.assertEqual(bytes(buf), bytes(exp))


class TestSyncWriteReference(unittest.TestCase):
    """Python-side check of the 22-byte SYNC WRITE layout (always runs)."""

    def test_layout(self):
        pkt = py_sync_write(2048, 3400, 1800, 3400)
        self.assertEqual(len(pkt), 22)
        self.assertEqual(pkt[3], 0x12)
        self.assertEqual(pkt[3] + 4, 22)
        self.assertEqual(pkt[:8], bytes([0xFF, 0xFF, 0xFE, 0x12, 0x83, 0x2A, 0x06, 0x01]))
        self.assertEqual(pkt[14], 0x02)
        self.assertEqual(int.from_bytes(pkt[8:10], "little"), 2048)
        self.assertEqual(int.from_bytes(pkt[12:14], "little"), 3400)
        self.assertEqual(int.from_bytes(pkt[15:17], "little"), 1800)
        self.assertEqual((sum(pkt[2:21]) + pkt[21]) & 0xFF, 0xFF)


if __name__ == "__main__":
    result = unittest.main(verbosity=2, exit=False).result
    sys.exit(0 if result.wasSuccessful() else 1)
