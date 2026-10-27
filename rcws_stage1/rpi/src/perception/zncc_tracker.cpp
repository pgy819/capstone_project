#include "perception/zncc_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace rcws {
namespace {

// 1-D parabola vertex offset from three samples (Sec 3.6 eq.)
inline double parabola(float a, float b, float c) {
  const double den = double(a) - 2.0 * b + c;
  if (std::abs(den) < 1e-9) return 0.0;
  const double d = 0.5 * (double(a) - c) / den;
  return std::clamp(d, -0.5, 0.5);
}

// Reduced pixel j (scale s, INTER_AREA from integer origin x0) covers full-res
// [x0 + j*s, x0 + (j+1)*s); its centre is x0 + j*s + (s-1)/2.
inline double to_full(int x0, double j, int s) { return x0 + j * s + 0.5 * (s - 1); }

}  // namespace

bool ZnccTracker::extract(const cv::Mat& y, int x0, int y0, int side_px, cv::Mat* out) const {
  if (x0 < 0 || y0 < 0 || x0 + side_px > y.cols || y0 + side_px > y.rows) return false;
  cv::Mat roi = y(cv::Rect(x0, y0, side_px, side_px));
  if (s_ == 1) {
    roi.convertTo(*out, CV_32F);
  } else {
    cv::Mat r;
    cv::resize(roi, r, cv::Size(side_px / s_, side_px / s_), 0, 0, cv::INTER_AREA);
    r.convertTo(*out, CV_32F);
  }
  return true;
}

bool ZnccTracker::init(const cv::Mat& y, const RectF& box) {
  const double full = cfg_.tmpl_scale * std::max(box.w, box.h);
  int s = 1;
  while (full / s > cfg_.tmpl_max && s < 16) s *= 2;
  int side = std::max(cfg_.tmpl_min, int(std::lround(full / s))) | 1;  // odd at matching scale
  const int side_px = side * s;
  const int x0 = int(std::lround(box.cx() - 0.5 * side_px));
  const int y0 = int(std::lround(box.cy() - 0.5 * side_px));
  const int saved = s_;
  s_ = s;
  cv::Mat t;
  if (!extract(y, x0, y0, side_px, &t)) { s_ = saved; return false; }
  cv::Scalar mean, sd;
  cv::meanStdDev(t, mean, sd);
  if (sd[0] < 3.0) { s_ = saved; return false; }  // featureless: ZNCC undefined
  tmpl_ = t;
  const double c = 0.5 * (side_px - 1);           // template centre offset at full resolution
  anchor_ = {box.cx() - (x0 + c), box.cy() - (y0 + c)};
  return true;
}

void ZnccTracker::refresh(const cv::Mat& y, const RectF& box) {
  if (tmpl_.empty()) { init(y, box); return; }
  const int side_px = tmpl_.cols * s_;
  const float cx = box.cx(), cy = box.cy();
  if (cx - side_px < 0 || cy - side_px < 0 || cx + side_px >= y.cols || cy + side_px >= y.rows) return;
  // sub-pixel crop centred exactly on the measured template position, then reduce
  cv::Mat full, t;
  cv::getRectSubPix(y, cv::Size(side_px, side_px), cv::Point2f(cx, cy), full, CV_32F);
  if (s_ == 1) t = full;
  else cv::resize(full, t, tmpl_.size(), 0, 0, cv::INTER_AREA);
  cv::addWeighted(tmpl_, 1.0 - cfg_.update_alpha, t, cfg_.update_alpha, 0.0, tmpl_);
}

NccResult ZnccTracker::match(const cv::Mat& y, cv::Point2d pred, double radius) const {
  NccResult r;
  if (tmpl_.empty()) return r;
  const int side = tmpl_.cols;
  const int side_px = side * s_;
  // search radius at matching scale (cost ~ (2R)^2 * side^2 is bounded by tmpl_max/search_max)
  const int rad = std::clamp(int(std::ceil(radius / s_)), std::max(2, cfg_.search_min / s_), cfg_.search_max);
  const int win_px = side_px + 2 * rad * s_;
  int x0 = int(std::lround(pred.x - 0.5 * side_px)) - rad * s_;
  int y0 = int(std::lround(pred.y - 0.5 * side_px)) - rad * s_;
  // clip the window to the image on a scale grid
  const int cx0 = std::max(0, x0), cy0 = std::max(0, y0);
  const int x1 = std::min(y.cols, x0 + win_px), y1 = std::min(y.rows, y0 + win_px);
  const int w = ((x1 - cx0) / s_) * s_, h = ((y1 - cy0) / s_) * s_;
  if (w < side_px + 2 * s_ || h < side_px + 2 * s_) return r;
  cv::Mat roi = y(cv::Rect(cx0, cy0, w, h)), search;
  if (s_ == 1) roi.convertTo(search, CV_32F);
  else {
    cv::Mat t;
    cv::resize(roi, t, cv::Size(w / s_, h / s_), 0, 0, cv::INTER_AREA);
    t.convertTo(search, CV_32F);
  }
  cv::Mat res;
  cv::matchTemplate(search, tmpl_, res, cv::TM_CCOEFF_NORMED);
  double vmax;
  cv::Point pmax;
  cv::minMaxLoc(res, nullptr, &vmax, nullptr, &pmax);
  double dx = 0, dy = 0;
  if (pmax.x > 0 && pmax.x < res.cols - 1)
    dx = parabola(res.at<float>(pmax.y, pmax.x - 1), res.at<float>(pmax.y, pmax.x), res.at<float>(pmax.y, pmax.x + 1));
  if (pmax.y > 0 && pmax.y < res.rows - 1)
    dy = parabola(res.at<float>(pmax.y - 1, pmax.x), res.at<float>(pmax.y, pmax.x), res.at<float>(pmax.y + 1, pmax.x));

  // second-best peak outside the main lobe (look-alike / repeated texture)
  const int lobe = std::clamp(side / 6, 2, 8);
  double v2 = -2;
  if (res.cols > 2 * lobe + 1 || res.rows > 2 * lobe + 1) {
    cv::Mat masked = res.clone();
    cv::rectangle(masked, cv::Rect(pmax.x - lobe, pmax.y - lobe, 2 * lobe + 1, 2 * lobe + 1), cv::Scalar(-2), cv::FILLED);
    cv::minMaxLoc(masked, nullptr, &v2);
  }
  // template centre at matching scale = top-left + (side-1)/2
  const double jc = pmax.x + dx + 0.5 * (side - 1), ic = pmax.y + dy + 0.5 * (side - 1);
  r.u = to_full(cx0, jc, s_);
  r.v = to_full(cy0, ic, s_);
  r.rho = float(vmax);
  r.second = float(v2);
  r.tmpl = side;
  r.scale = s_;
  const bool margin_ok = r.second <= -1.5f || (r.rho - r.second) >= cfg_.peak_margin;
  r.ok = r.rho >= cfg_.rho_min && margin_ok;
  return r;
}

}  // namespace rcws
