"""
tracker_node.py — Hailo-8 "cell phone" tracker -> Kalman -> UART (Stage 1 main node)
====================================================================================

Built on the `hailo-rpi5-examples` GStreamer detection pipeline callback model:
    GStreamerDetectionApp(app_callback, user_data).run()

Per frame:
  1. Read Hailo detections, keep only label == "cell phone" with confidence >= 0.45.
  2. Convert normalized bbox centre to virtual FHD error (origin = (960, 540)).
  3. Lock on to the box closest to the previous predicted position
     (or (0, 0) if the target was lost).
  4. Run the 2D Kalman tracker (coasting / lead / deadzone).
  5. Send the 8-byte packet to the STM32 over /dev/ttyAMA0.
  6. Draw overlay: origin crosshair (green), detection centre (blue),
     Kalman future prediction (red).

Run on the Pi 5 (inside the hailo-rpi5-examples venv: `source setup_env.sh`):
    python3 rpi5_vision/tracker_node.py --input rpi
Environment variables:
    RCWS_UART_PORT   (default /dev/ttyAMA0)
    RCWS_UART_DRYRUN=1   -> do not open the serial port
    RCWS_NO_OVERLAY=1    -> do not force --use-frame / overlay
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Iterable, List, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from kalman_filter import (  # noqa: E402
    KalmanTracker2D, KalmanOutput, STATUS_COASTING, STATUS_DETECTED, STATUS_LOST,
)
from uart_tx import UartTransmitter, build_packet  # noqa: E402

# --------------------------------------------------------------------------- #
# Spec constants (Module A §1)
# --------------------------------------------------------------------------- #
TARGET_LABEL = "cell phone"
CONF_THRESHOLD = 0.45
VIRTUAL_W = 1920
VIRTUAL_H = 1080
VIRTUAL_CX = 960
VIRTUAL_CY = 540

# BGR colours for OpenCV
COLOR_ORIGIN = (0, 255, 0)     # green
COLOR_DETECT = (255, 0, 0)     # blue
COLOR_PREDICT = (0, 0, 255)    # red
COLOR_TEXT = (255, 255, 255)

STATUS_NAMES = {STATUS_LOST: "LOST", STATUS_DETECTED: "DETECTED", STATUS_COASTING: "COASTING"}


# --------------------------------------------------------------------------- #
# Hardware-independent core (unit-tested in test_pipeline.py)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    xmin: float     # normalized [0, 1]
    ymin: float
    width: float
    height: float


@dataclass(frozen=True)
class FrameResult:
    status: int
    pred_ex: int
    pred_ey: int
    raw: Optional[Tuple[int, int]]
    packet: bytes
    kalman: KalmanOutput


def bbox_to_error(xmin: float, ymin: float, width: float, height: float) -> Tuple[int, int]:
    """Normalized Hailo bbox -> raw pixel error in the virtual 1920x1080 frame."""
    raw_ex = int((xmin + width / 2.0) * float(VIRTUAL_W)) - VIRTUAL_CX
    raw_ey = int((ymin + height / 2.0) * float(VIRTUAL_H)) - VIRTUAL_CY
    return raw_ex, raw_ey


class TrackerCore:
    """Target filter + lock-on + Kalman + packet generation (no GStreamer/Hailo)."""

    def __init__(self, kalman: Optional[KalmanTracker2D] = None,
                 transmitter: Optional[UartTransmitter] = None) -> None:
        self.kf = kalman if kalman is not None else KalmanTracker2D()
        self.tx = transmitter
        self.last_result: Optional[FrameResult] = None

    def lock_reference(self) -> Tuple[int, int]:
        """Previous predicted position, or (0, 0) if previously lost."""
        if self.kf.status == STATUS_LOST or not self.kf.is_tracking:
            return 0, 0
        return self.kf.pred_ex, self.kf.pred_ey

    def select_target(self, detections: Iterable[Detection]) -> Optional[Tuple[int, int]]:
        ref_x, ref_y = self.lock_reference()
        best: Optional[Tuple[int, int]] = None
        best_d2 = math.inf
        for det in detections:
            if det.label != TARGET_LABEL or det.confidence < CONF_THRESHOLD:
                continue
            ex, ey = bbox_to_error(det.xmin, det.ymin, det.width, det.height)
            d2 = float(ex - ref_x) ** 2 + float(ey - ref_y) ** 2
            if d2 < best_d2:
                best_d2, best = d2, (ex, ey)
        return best

    def process(self, detections: Iterable[Detection],
                now: Optional[float] = None) -> FrameResult:
        raw = self.select_target(detections)
        out = self.kf.update(raw, now=now)
        if self.tx is not None:
            pkt = self.tx.send(out.status, out.pred_ex, out.pred_ey)
        else:
            pkt = build_packet(out.status, out.pred_ex, out.pred_ey)
        result = FrameResult(out.status, out.pred_ex, out.pred_ey, raw, pkt, out)
        self.last_result = result
        return result


def virtual_to_frame(ex: int, ey: int, width: int, height: int) -> Tuple[int, int]:
    """Virtual error coordinates -> pixel coordinates of the actual frame."""
    px = int(round((ex + VIRTUAL_CX) * width / float(VIRTUAL_W)))
    py = int(round((ey + VIRTUAL_CY) * height / float(VIRTUAL_H)))
    return px, py


def draw_overlay(frame, result: FrameResult, fps: Optional[float] = None):
    """Draw origin crosshair (green), detection (blue), prediction (red)."""
    import cv2  # lazy: not needed for unit tests

    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    arm = max(12, w // 40)

    # Origin crosshair (green)
    cv2.line(frame, (cx - arm, cy), (cx + arm, cy), COLOR_ORIGIN, 2)
    cv2.line(frame, (cx, cy - arm), (cx, cy + arm), COLOR_ORIGIN, 2)
    dz_x = int(round(6 * w / float(VIRTUAL_W)))
    dz_y = int(round(6 * h / float(VIRTUAL_H)))
    cv2.rectangle(frame, (cx - dz_x, cy - dz_y), (cx + dz_x, cy + dz_y), COLOR_ORIGIN, 1)

    det_pt = None
    if result.raw is not None:
        det_pt = virtual_to_frame(result.raw[0], result.raw[1], w, h)
        cv2.circle(frame, det_pt, 7, COLOR_DETECT, 2)

    if result.status != STATUS_LOST:
        pred_pt = virtual_to_frame(result.pred_ex, result.pred_ey, w, h)
        cv2.circle(frame, pred_pt, 5, COLOR_PREDICT, -1)
        if det_pt is not None:
            cv2.line(frame, det_pt, pred_pt, COLOR_PREDICT, 1)

    text = f"{STATUS_NAMES.get(result.status, '?')}  err=({result.pred_ex:+d},{result.pred_ey:+d})"
    if fps is not None:
        text += f"  {fps:4.1f}fps"
    cv2.putText(frame, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_TEXT, 2)
    return frame


# --------------------------------------------------------------------------- #
# Hailo / GStreamer runtime glue
# --------------------------------------------------------------------------- #
def _load_hailo_runtime() -> SimpleNamespace:
    """Import Hailo/GStreamer helpers across hailo-rpi5-examples versions."""
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst  # noqa: F401
    import hailo

    errors = []
    # 1) hailo-apps (newest layout)
    try:
        from hailo_apps.hailo_app_python.core.common.buffer_utils import (
            get_caps_from_pad, get_numpy_from_buffer)
        from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
        from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import (
            GStreamerDetectionApp)
        return SimpleNamespace(Gst=Gst, hailo=hailo, get_caps_from_pad=get_caps_from_pad,
                               get_numpy_from_buffer=get_numpy_from_buffer,
                               app_callback_class=app_callback_class,
                               GStreamerDetectionApp=GStreamerDetectionApp)
    except ImportError as e:
        errors.append(e)
    # 2) hailo_apps_infra
    try:
        from hailo_apps_infra.hailo_rpi_common import (
            get_caps_from_pad, get_numpy_from_buffer, app_callback_class)
        from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp
        return SimpleNamespace(Gst=Gst, hailo=hailo, get_caps_from_pad=get_caps_from_pad,
                               get_numpy_from_buffer=get_numpy_from_buffer,
                               app_callback_class=app_callback_class,
                               GStreamerDetectionApp=GStreamerDetectionApp)
    except ImportError as e:
        errors.append(e)
    # 3) legacy basic_pipelines/ (run from the examples repo)
    try:
        from hailo_rpi_common import (
            get_caps_from_pad, get_numpy_from_buffer, app_callback_class)
        from detection_pipeline import GStreamerDetectionApp
        return SimpleNamespace(Gst=Gst, hailo=hailo, get_caps_from_pad=get_caps_from_pad,
                               get_numpy_from_buffer=get_numpy_from_buffer,
                               app_callback_class=app_callback_class,
                               GStreamerDetectionApp=GStreamerDetectionApp)
    except ImportError as e:
        errors.append(e)
    raise ImportError(f"hailo-rpi5-examples helpers not found: {errors}")


_RT: Optional[SimpleNamespace] = None


def app_callback(pad, info, user_data):
    """GStreamer pad-probe callback (identity element after hailofilter)."""
    Gst = _RT.Gst
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    user_data.increment()

    # --- Hailo detections -> Detection list ---
    roi = _RT.hailo.get_roi_from_buffer(buffer)
    hailo_dets = roi.get_objects_typed(_RT.hailo.HAILO_DETECTION)
    detections: List[Detection] = []
    for d in hailo_dets:
        label = d.get_label()
        if label != TARGET_LABEL:
            continue
        bbox = d.get_bbox()
        detections.append(Detection(label, float(d.get_confidence()),
                                    float(bbox.xmin()), float(bbox.ymin()),
                                    float(bbox.width()), float(bbox.height())))

    # --- Lock-on + Kalman + UART ---
    result = user_data.core.process(detections)

    # --- Overlay ---
    if user_data.use_frame:
        fmt, width, height = _RT.get_caps_from_pad(pad)
        if fmt is not None and width is not None and height is not None:
            import cv2
            frame = _RT.get_numpy_from_buffer(buffer, fmt, width, height)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            draw_overlay(frame, result)
            user_data.set_frame(frame)

    return Gst.PadProbeReturn.OK


def main() -> None:
    global _RT
    _RT = _load_hailo_runtime()

    if os.environ.get("RCWS_NO_OVERLAY") != "1" and "--use-frame" not in sys.argv \
            and "-u" not in sys.argv:
        sys.argv.append("--use-frame")

    port = os.environ.get("RCWS_UART_PORT", "/dev/ttyAMA0")
    dry_run = os.environ.get("RCWS_UART_DRYRUN") == "1"
    tx = UartTransmitter(port=port, baudrate=115200, dry_run=dry_run)
    core = TrackerCore(KalmanTracker2D(), tx)

    class RcwsUserData(_RT.app_callback_class):
        def __init__(self, tracker_core: TrackerCore) -> None:
            super().__init__()
            self.core = tracker_core

    user_data = RcwsUserData(core)
    app = _RT.GStreamerDetectionApp(app_callback, user_data)
    try:
        app.run()
    finally:
        tx.close()


if __name__ == "__main__":
    main()
