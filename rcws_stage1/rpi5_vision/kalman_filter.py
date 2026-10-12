"""
kalman_filter.py — 2D Constant-Velocity Kalman Filter + Eye-in-Hand Lead Compensation
=====================================================================================

Stage 1 (RCWS) — Raspberry Pi 5 vision side.

State vector      X = [Ex, Ey, Vx, Vy]^T   (px, px, px/s, px/s)
Measurement       Z = [raw_ex, raw_ey]^T   (px, relative to virtual origin (960, 540))

Behaviour (see docs/STAGE1_SPEC.md, Module A §2):
  * P0 / Q / R explicitly initialised as np.float32 diagonal matrices.
  * dt is measured every frame and clamped to [0.005, 0.100] s; F is rebuilt from dt.
  * First detection after reset/lost -> state is synchronised directly to the
    measurement with zero velocity (no velocity spike from (0, 0)).
  * Up to 5 consecutive missed frames -> predict-only "coasting" (status 0x02).
  * 6th miss (or miss while not tracking) -> full reset, status 0x00, output (0, 0).
  * Conditional lead: 45 ms when the target diverges from centre (e * v > 0),
    15 ms when it converges, to avoid eye-in-hand oscillation.
  * ±6 px deadzone, then clamp to [-960, 960] x [-540, 540].
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

# --------------------------------------------------------------------------- #
# Constants (Immutable data contract with STM32 — see GEMINI.md §2)
# --------------------------------------------------------------------------- #
STATUS_LOST = 0x00
STATUS_DETECTED = 0x01
STATUS_COASTING = 0x02

DT_MIN_S = 0.005
DT_MAX_S = 0.100

MAX_COAST_FRAMES = 5

LEAD_DIVERGING_S = 0.045   # target moving away from centre
LEAD_CONVERGING_S = 0.015  # target moving towards centre (or stationary)

DEADZONE_PX = 6
LIMIT_X_PX = 960
LIMIT_Y_PX = 540


@dataclass(frozen=True)
class KalmanOutput:
    """Result of one KalmanTracker2D.update() call."""
    status: int
    pred_ex: int
    pred_ey: int
    est_ex: float
    est_ey: float
    est_vx: float
    est_vy: float
    dt: float


class KalmanTracker2D:
    """2D constant-velocity Kalman tracker with coasting and lead compensation."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock

        # Measurement matrix H (2x4)
        self.H = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]], dtype=np.float32)

        # Covariance & noise matrices (explicit float32, values fixed by spec)
        self.P0 = np.diag([100.0, 100.0, 1000.0, 1000.0]).astype(np.float32)
        self.Q = np.diag([2.0, 2.0, 50.0, 50.0]).astype(np.float32)
        self.R = np.diag([25.0, 25.0]).astype(np.float32)

        self._I4 = np.eye(4, dtype=np.float32)
        self.F = np.eye(4, dtype=np.float32)

        self.X = np.zeros((4, 1), dtype=np.float32)
        self.P = self.P0.copy()

        self.is_tracking = False
        self.coast_counter = 0
        self.last_time = self._clock()

        # Last published outputs (used by the tracker node for target locking)
        self.status = STATUS_LOST
        self.pred_ex = 0
        self.pred_ey = 0
        self.last_dt = DT_MIN_S

    # ------------------------------------------------------------------ #
    # Core Kalman steps
    # ------------------------------------------------------------------ #
    def measure_dt(self, now: Optional[float] = None) -> float:
        """Measure elapsed time since the previous frame and clamp it."""
        if now is None:
            now = self._clock()
        dt = float(np.clip(now - self.last_time, DT_MIN_S, DT_MAX_S))
        self.last_time = now
        self.last_dt = dt
        return dt

    def _update_F(self, dt: float) -> None:
        self.F = np.array([[1.0, 0.0, dt, 0.0],
                           [0.0, 1.0, 0.0, dt],
                           [0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0]], dtype=np.float32)

    def predict(self, dt: float) -> None:
        """Time update: X = F X, P = F P F^T + Q."""
        self._update_F(dt)
        self.X = (self.F @ self.X).astype(np.float32)
        self.P = (self.F @ self.P @ self.F.T + self.Q).astype(np.float32)

    def correct(self, z: Sequence[float]) -> None:
        """Measurement update (Joseph form for numerical stability)."""
        z_vec = np.asarray(z, dtype=np.float32).reshape(2, 1)
        y = z_vec - self.H @ self.X
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.X = (self.X + K @ y).astype(np.float32)
        ikh = self._I4 - K @ self.H
        P = ikh @ self.P @ ikh.T + K @ self.R @ K.T
        self.P = (0.5 * (P + P.T)).astype(np.float32)  # keep symmetric

    def reset(self) -> None:
        """Return to the lost state: X = 0, P = P0, not tracking."""
        self.X = np.zeros((4, 1), dtype=np.float32)
        self.P = self.P0.copy()
        self.is_tracking = False

    # ------------------------------------------------------------------ #
    # Lead compensation + deadzone + clamping
    # ------------------------------------------------------------------ #
    @staticmethod
    def compute_prediction(est_ex: float, est_ey: float,
                           est_vx: float, est_vy: float) -> Tuple[int, int]:
        """Eye-in-hand conditional lead, deadzone and clamp (spec §2)."""
        lead_x = LEAD_DIVERGING_S if (est_ex * est_vx > 0) else LEAD_CONVERGING_S
        lead_y = LEAD_DIVERGING_S if (est_ey * est_vy > 0) else LEAD_CONVERGING_S

        pred_ex = int(round(est_ex + est_vx * lead_x))
        pred_ey = int(round(est_ey + est_vy * lead_y))

        if abs(pred_ex) < DEADZONE_PX:
            pred_ex = 0
        if abs(pred_ey) < DEADZONE_PX:
            pred_ey = 0

        pred_ex = max(-LIMIT_X_PX, min(LIMIT_X_PX, pred_ex))
        pred_ey = max(-LIMIT_Y_PX, min(LIMIT_Y_PX, pred_ey))
        return pred_ex, pred_ey

    # ------------------------------------------------------------------ #
    # Per-frame entry point
    # ------------------------------------------------------------------ #
    def update(self, measurement: Optional[Tuple[float, float]],
               now: Optional[float] = None) -> KalmanOutput:
        """
        Process one camera frame.

        measurement: (raw_ex, raw_ey) if a "cell phone" was detected, else None.
        now:         optional timestamp override (seconds, monotonic).
        """
        dt = self.measure_dt(now)

        if measurement is not None:
            raw_ex, raw_ey = float(measurement[0]), float(measurement[1])
            if not self.is_tracking:
                # First detection after reset/lost: sync state, zero velocity.
                self.X = np.array([[raw_ex], [raw_ey], [0.0], [0.0]], dtype=np.float32)
                self.P = self.P0.copy()
                self.is_tracking = True
            else:
                self.predict(dt)
                self.correct((raw_ex, raw_ey))
            self.coast_counter = 0
            status = STATUS_DETECTED
        else:
            # Bounded increment (value above MAX_COAST_FRAMES means "lost").
            self.coast_counter = min(self.coast_counter + 1, MAX_COAST_FRAMES + 1)
            if self.is_tracking and self.coast_counter <= MAX_COAST_FRAMES:
                self.predict(dt)
                status = STATUS_COASTING
            else:
                self.reset()
                status = STATUS_LOST

        est_ex, est_ey, est_vx, est_vy = (float(v) for v in self.X.ravel())

        if status == STATUS_LOST:
            pred_ex, pred_ey = 0, 0
        else:
            pred_ex, pred_ey = self.compute_prediction(est_ex, est_ey, est_vx, est_vy)

        self.status, self.pred_ex, self.pred_ey = status, pred_ex, pred_ey
        return KalmanOutput(status, pred_ex, pred_ey, est_ex, est_ey, est_vx, est_vy, dt)
