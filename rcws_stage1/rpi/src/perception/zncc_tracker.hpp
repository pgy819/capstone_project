// Sub-pixel ZNCC template tracker (Sec 3.6). Works on the main-stream luma plane.
//
// The template always covers the WHOLE target (box x tmpl_scale). Large targets are
// matched at a reduced scale s = 2^L chosen so the template side stays <= tmpl_max;
// a template cut from the middle of a large, uniform phone has no unique peak
// (field data 2026-10-28: 96 px patch inside a 280x420 px phone, peak margin < 0).
#pragma once
#include <opencv2/core.hpp>

#include "common/config.hpp"
#include "common/types.hpp"

namespace rcws {

struct NccResult {
  bool ok = false;
  double u = 0, v = 0;   // template centre, main-stream pixels (sub-pixel)
  float rho = 0;         // best ZNCC peak
  float second = 0;      // best peak outside the main lobe (-2 = none)
  int tmpl = 0;          // template side at the matching scale [px]
  int scale = 1;         // matching scale factor (1, 2, 4, ...)
};

class ZnccTracker {
 public:
  explicit ZnccTracker(const NccCfg& c) : cfg_(c) {}

  bool has_template() const { return !tmpl_.empty(); }
  void clear() { tmpl_.release(); }

  // (Re)create the template from a detection box on frame y.
  bool init(const cv::Mat& y, const RectF& box);
  // Blend the template toward the patch centred at (box centre) (alpha = cfg.update_alpha).
  void refresh(const cv::Mat& y, const RectF& box);
  // Search around pred within +-radius px (main-stream pixels).
  // frame_id identifies the frame so the reduced image is computed once per frame and scale.
  NccResult match(const cv::Mat& y, cv::Point2d pred, double radius, uint32_t frame_id) const;

  int side() const { return tmpl_.cols; }
  int scale() const { return s_; }
  // box centre minus template centre at (re)initialisation (sub-pixel rounding residual)
  cv::Point2d anchor() const { return anchor_; }

 private:
  // Full-resolution square of side side_px with top-left (x0,y0), reduced by s_ (INTER_AREA).
  bool extract(const cv::Mat& y, int x0, int y0, int side_px, cv::Mat* out) const;

  NccCfg cfg_;
  cv::Mat tmpl_;   // CV_32F at the matching scale
  int s_ = 1;      // matching scale factor
  cv::Point2d anchor_;
  mutable cv::Mat small_;          // whole frame reduced by s_ (cache)
  mutable uint32_t small_id_ = ~0u;
  mutable int small_s_ = 0;
};

}  // namespace rcws
