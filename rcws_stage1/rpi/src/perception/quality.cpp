#include "perception/quality.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace rcws {

float box_sharpness(const FrameView& f, const RectF& box) {
  if (!f.main_y) return 0;
  const float mx = 0.2f * box.w, my = 0.2f * box.h;
  const int x0 = std::max(1, int(box.x + mx)), y0 = std::max(1, int(box.y + my));
  const int x1 = std::min(f.main_w - 2, int(box.x + box.w - mx)), y1 = std::min(f.main_h - 2, int(box.y + box.h - my));
  if (x1 - x0 < 4 || y1 - y0 < 4) return 0;
  const int step = std::max(1, int(std::sqrt(double(x1 - x0) * (y1 - y0) / 4000.0)));
  double sum = 0;
  int n = 0;
  for (int y = y0; y <= y1; y += step) {
    const uint8_t* r = f.main_y + size_t(y) * f.main_stride;
    const uint8_t* up = r - f.main_stride;
    const uint8_t* dn = r + f.main_stride;
    for (int x = x0; x <= x1; x += step) {
      sum += std::abs(4 * int(r[x]) - r[x - 1] - r[x + 1] - up[x] - dn[x]);
      ++n;
    }
  }
  return n ? float(sum / n) : 0.f;
}

bool QualityMonitor::evaluate(const FrameView& f, const RectF* target, bool tracking_healthy) {
  if (!c_.enabled) return false;
  if (n_++ % uint32_t(std::max(1, c_.period))) return false;
  // global brightness from the lores luma (or main, subsampled)
  const uint8_t* y = f.lo_y ? f.lo_y : f.main_y;
  const int w = f.lo_y ? f.lo_w : f.main_w, h = f.lo_y ? f.lo_h : f.main_h;
  const int stride = f.lo_y ? f.lo_stride : f.main_stride;
  if (!y) return false;
  const int step = std::max(2, int(std::sqrt(double(w) * h / 15000.0)));
  long sum = 0;
  int n = 0, sat = 0;
  for (int j = 0; j < h; j += step) {
    const uint8_t* r = y + size_t(j) * stride;
    for (int i = 0; i < w; i += step) {
      sum += r[i];
      sat += r[i] >= 250;
      ++n;
    }
  }
  s_.mean_luma = n ? float(sum) / n : 0.f;
  s_.sat_frac = n ? float(sat) / n : 0.f;
  s_.raw = 0;
  if (s_.mean_luma < c_.dark_luma) s_.raw |= Q_DARK;
  if (s_.sat_frac > c_.sat_frac) s_.raw |= Q_GLARE;
  s_.sharp = target ? box_sharpness(f, *target) : 0.f;
  if (target && base_n_ >= 10 && s_.sharp < c_.blur_ratio * s_.sharp_base) s_.raw |= Q_BLUR;
  // baseline only from healthy tracking with a clean picture (a blurred or dark target must not
  // lower its own reference)
  if (target && tracking_healthy && !(s_.raw & (Q_DARK | Q_GLARE)) && !(s_.flags & Q_BLUR) && s_.sharp > 0) {
    const float a = base_n_ < 10 ? 1.f / float(base_n_ + 1) : 0.05f;
    s_.sharp_base = base_n_ ? (1 - a) * s_.sharp_base + a * s_.sharp : s_.sharp;
    ++base_n_;
  }
  for (int b = 0; b < 3; ++b) {
    const uint8_t bit = uint8_t(1u << b);
    if (s_.raw & bit) { bad_[b]++; good_[b] = 0; }
    else { good_[b]++; bad_[b] = 0; }
    if (bad_[b] >= c_.bad_n) s_.flags |= bit;
    if (good_[b] >= c_.good_n) s_.flags &= uint8_t(~bit);
  }
  // no target: blur cannot be judged, do not keep a stale BLUR flag
  if (!target) s_.flags &= uint8_t(~Q_BLUR);
  return true;
}

}  // namespace rcws
