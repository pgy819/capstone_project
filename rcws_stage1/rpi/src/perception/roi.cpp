#include "perception/roi.hpp"

#include <algorithm>
#include <cmath>

namespace rcws {

cv::Rect clamp_shift(cv::Rect r, int img_w, int img_h) {
  r.width = std::min(r.width, img_w);
  r.height = std::min(r.height, img_h);
  r.x = std::clamp(r.x, 0, img_w - r.width);
  r.y = std::clamp(r.y, 0, img_h - r.height);
  return r;
}

cv::Rect track_roi(const RoiCfg& c, cv::Point2d center, float tgt_w, float tgt_h, double sigma_px,
                   int img_w, int img_h) {
  const double s = std::clamp(c.k_s * std::max(tgt_w, tgt_h) + 6.0 * sigma_px, double(c.s_min), double(c.s_max));
  const int side = int(std::lround(s)) & ~1;  // even for YUV420 chroma alignment
  cv::Rect r(int(std::lround(center.x - side / 2.0)) & ~1, int(std::lround(center.y - side / 2.0)) & ~1, side, side);
  return clamp_shift(r, img_w & ~1, img_h & ~1);
}

std::vector<cv::Rect> make_tiles(const RoiCfg& c, int img_w, int img_h) {
  std::vector<cv::Rect> out;
  const int nx = std::max(1, int(std::ceil(double(img_w) / c.tile_w)));
  const int ny = std::max(1, int(std::ceil(double(img_h) / c.tile_h)));
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      // evenly spread origins so neighbours overlap (Sec 3.5: 192 x 144 px for 1536x864)
      const int x = nx == 1 ? 0 : int(std::lround(double(img_w - c.tile_w) * i / (nx - 1)));
      const int y = ny == 1 ? 0 : int(std::lround(double(img_h - c.tile_h) * j / (ny - 1)));
      out.push_back(clamp_shift(cv::Rect(x & ~1, y & ~1, c.tile_w, c.tile_h), img_w, img_h));
    }
  return out;
}

cv::Rect person_hand_roi(const RectF& p, int img_w, int img_h) {
  // head-to-waist height, 1.5x body width (hands may be outside the torso box)
  const double w = 1.5 * p.w;
  const double h = 0.6 * p.h;
  const double side = std::max(w, h);
  cv::Rect r(int(p.cx() - side / 2) & ~1, int(p.y) & ~1, int(side) & ~1, int(side) & ~1);
  return clamp_shift(r, img_w & ~1, img_h & ~1);
}

}  // namespace rcws
