// RCWS Stage 1 - common data types shared by all modules.
// All times are CLOCK_MONOTONIC nanoseconds unless noted. Angles are radians.
#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace rcws {

struct RectF {
  float x = 0, y = 0, w = 0, h = 0;  // top-left + size, pixels
  float cx() const { return x + 0.5f * w; }
  float cy() const { return y + 0.5f * h; }
  float area() const { return w * h; }
};

float iou(const RectF& a, const RectF& b);

// One camera frame as seen by the perception pipeline.
// Planes are borrowed from the source and valid until the frame is released.
struct FrameView {
  uint32_t seq = 0;
  int64_t t_sof_ns = 0;       // sensor timestamp converted to CLOCK_MONOTONIC (Sec 5.1)
  int32_t exposure_us = 0;    // actual exposure from metadata
  float analog_gain = 1.f;
  float lens_position = 0.f;
  // Main stream luma (full sensor-mode resolution, e.g. 1536x864)
  const uint8_t* main_y = nullptr;
  int main_w = 0, main_h = 0, main_stride = 0;
  // Main stream chroma (YUV420 planar), may be null for grayscale sources
  const uint8_t* main_u = nullptr;
  const uint8_t* main_v = nullptr;
  int main_uv_stride = 0;
  // Lores stream (YUV420 planar), may be null
  const uint8_t* lo_y = nullptr;
  const uint8_t* lo_u = nullptr;
  const uint8_t* lo_v = nullptr;
  int lo_w = 0, lo_h = 0, lo_stride = 0, lo_uv_stride = 0;
};

enum class ObjClass : uint8_t { Phone = 0, Person = 1, Other = 255 };

// Detector output mapped back to main-stream pixel coordinates.
struct Detection {
  RectF box;
  float score = 0;
  ObjClass cls = ObjClass::Other;
};

enum class DetKind : uint8_t { Global = 0, Tile = 1, Roi = 2 };

struct DetectionBatch {
  uint32_t frame_seq = 0;
  int64_t t_sof_ns = 0;
  DetKind kind = DetKind::Global;
  RectF crop;                   // region of main image that was fed to the detector
  int64_t t_submit_ns = 0;      // when the request was queued
  int64_t t_done_ns = 0;        // when the result was available
  std::vector<Detection> dets;
};

enum class MeasSrc : uint8_t { DET = 0, NCC = 1 };

// Flow 1 output (Sec 3.7). Pixel coords are distorted main-stream pixels;
// the estimator undistorts them through the camera model.
struct VisionMeas {
  int64_t t_cap_ns = 0;
  float u = 0, v = 0;
  float sigma_px = 1.f;
  MeasSrc src = MeasSrc::DET;
  float conf = 0;
  float w = 0, h = 0;
  uint32_t frame_seq = 0;
};

enum class TrackState : uint8_t { SEARCH = 0, ACQUIRE = 1, TRACK = 2, COAST = 3, REACQ = 4 };
const char* to_string(TrackState s);

}  // namespace rcws
