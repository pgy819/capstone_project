// Image quality gate (scenario G: backlight, sudden light change, low light, blur).
// Cheap statistics every few frames: mean luma and saturated fraction of the lores image,
// target sharpness (mean |Laplacian| inside the tracked box) against its own baseline taken
// while tracking was healthy. A flag needs bad_n consecutive bad evaluations to set and
// good_n good ones to clear, so a single dark frame does not stop the gimbal.
#pragma once
#include <cstdint>

#include "common/config.hpp"
#include "common/types.hpp"

namespace rcws {

enum QualFlag : uint8_t { Q_DARK = 1, Q_GLARE = 2, Q_BLUR = 4 };

struct QualitySample {
  float mean_luma = 0, sat_frac = 0, sharp = 0, sharp_base = 0;
  uint8_t raw = 0;     // this evaluation alone
  uint8_t flags = 0;   // after hysteresis
};

class QualityMonitor {
 public:
  explicit QualityMonitor(const QualityCfg& c) : c_(c) {}
  // Returns true when an evaluation ran on this frame (then last() is new).
  bool evaluate(const FrameView& f, const RectF* target, bool tracking_healthy);
  uint8_t flags() const { return c_.enabled ? s_.flags : 0; }
  bool ok() const { return flags() == 0; }
  const QualitySample& last() const { return s_; }
  void reset_baseline() { base_n_ = 0; s_.sharp_base = 0; }

 private:
  QualityCfg c_;
  QualitySample s_;
  uint32_t n_ = 0;
  int bad_[3] = {0, 0, 0}, good_[3] = {0, 0, 0};
  int base_n_ = 0;
};

// Mean absolute 4-neighbour Laplacian of the luma inside `box` (inner 60 %, at most ~4000 samples).
float box_sharpness(const FrameView& f, const RectF& box);

}  // namespace rcws
