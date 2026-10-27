#include "sim/synthetic.hpp"

#include <opencv2/imgproc.hpp>

#include "common/clock.hpp"

namespace rcws {

SyntheticSource::SyntheticSource(const CameraCfg& cam, SynthTraj traj, int n_frames, uint32_t seed)
    : cam_(cam), traj_(std::move(traj)), n_(n_frames), rng_(seed) {
  // cluttered static background: blurred noise + random rectangles
  bg_.create(cam.main_h, cam.main_w, CV_8UC1);
  cv::randu(bg_, 60, 160);
  cv::GaussianBlur(bg_, bg_, cv::Size(0, 0), 6);
  std::uniform_int_distribution<int> rx(0, cam.main_w - 1), ry(0, cam.main_h - 1), rs(10, 120), rv(30, 220);
  for (int i = 0; i < 60; ++i)
    cv::rectangle(bg_, cv::Rect(rx(rng_), ry(rng_), rs(rng_), rs(rng_)), cv::Scalar(rv(rng_)), cv::FILLED);
  cv::GaussianBlur(bg_, bg_, cv::Size(0, 0), 1.2);
  // phone texture: dark body, bright screen with icons
  tex_.create(int(traj_.tgt_h), int(traj_.tgt_w), CV_8UC1);
  tex_.setTo(25);
  if (traj_.plain) {
    // like the field run: almost uniform dark back, rounded slightly lighter edge, small camera bump
    tex_.setTo(40);
    cv::rectangle(tex_, cv::Rect(0, 0, tex_.cols, tex_.rows), cv::Scalar(70), std::max(2, tex_.cols / 40));
    cv::rectangle(tex_, cv::Rect(tex_.cols / 10, tex_.rows / 20, tex_.cols / 3, tex_.rows / 6), cv::Scalar(15), cv::FILLED);
  } else {
  const int m = std::max(2, int(traj_.tgt_w * 0.08));
  cv::Rect scr(m, 2 * m, tex_.cols - 2 * m, tex_.rows - 4 * m);
  tex_(scr).setTo(200);
  std::uniform_int_distribution<int> ix(0, std::max(1, scr.width - 6)), iy(0, std::max(1, scr.height - 6));
  for (int i = 0; i < 10; ++i)
    cv::circle(tex_, cv::Point(scr.x + ix(rng_), scr.y + iy(rng_)), std::max(1, m), cv::Scalar(90), cv::FILLED);
  }
  u_.create(cam.main_h / 2, cam.main_w / 2, CV_8UC1); u_.setTo(128);
  v_.create(cam.main_h / 2, cam.main_w / 2, CV_8UC1); v_.setTo(128);
  lo_u_.create(cam.lores_h / 2, cam.lores_w / 2, CV_8UC1); lo_u_.setTo(128);
  lo_v_.create(cam.lores_h / 2, cam.lores_w / 2, CV_8UC1); lo_v_.setTo(128);
}

bool SyntheticSource::grab(Frame* f, int) {
  if (seq_ >= uint32_t(n_)) return false;
  const uint32_t seq = seq_++;
  const double t = t_of(seq);
  y_ = bg_.clone();
  if (traj_.decoy) {
    cv::Mat d = 255 - tex_;  // inverted look-alike
    cv::Rect r(cam_.main_w / 5, cam_.main_h / 2, d.cols, d.rows);
    d.copyTo(y_(r));
  }
  // Render the target where it is at the capture time the pipeline will assign to its row
  // (t_sof + sign*exp/2 + row*t_line, Sec 5.1), so synthetic truth and timing model agree.
  const int32_t exp_us = last_ctrl_.exposure_us ? last_ctrl_.exposure_us : cam_.exposure_track_us;
  auto t_cap = [&](double row) {
    return t + cam_.timestamp_sign * exp_us * 0.5e-6 + row * double(cam_.sensor_h) / cam_.main_h * cam_.line_time_ns * 1e-9;
  };
  cv::Point2d c = traj_.center(t_cap(cam_.main_h / 2.0));
  c = traj_.center(t_cap(c.y));
  const double t_target = t_cap(c.y);
  const bool occluded = t >= traj_.occl_t0 && t < traj_.occl_t1;
  RectF box{float(c.x - traj_.tgt_w / 2), float(c.y - traj_.tgt_h / 2), float(traj_.tgt_w), float(traj_.tgt_h)};
  if (!occluded) {
    // sub-pixel placement via affine warp so ZNCC sub-pixel accuracy is testable
    cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, box.x, 0, 1, box.y);
    cv::Mat mask(tex_.size(), CV_8UC1, cv::Scalar(255));
    cv::Mat warped, wmask;
    cv::warpAffine(tex_, warped, M, y_.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    cv::warpAffine(mask, wmask, M, y_.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    // alpha blend with the fractional mask
    cv::Mat yf, wf, af;
    y_.convertTo(yf, CV_32F); warped.convertTo(wf, CV_32F); wmask.convertTo(af, CV_32F, 1.0 / 255);
    yf = yf.mul(1 - af) + wf.mul(af);
    yf.convertTo(y_, CV_8U);
  } else {
    cv::rectangle(y_, cv::Rect(int(box.x) - 10, int(box.y) - 10, int(box.w) + 20, int(box.h) + 20), cv::Scalar(140), cv::FILLED);
  }
  cv::Mat noise(y_.size(), CV_16S);
  cv::randn(noise, 0, traj_.noise_sigma);
  cv::Mat y16;
  y_.convertTo(y16, CV_16S);
  y16 += noise;
  y16.convertTo(y_, CV_8U);
  cv::resize(y_, lo_y_, cv::Size(cam_.lores_w, cam_.lores_h), 0, 0, cv::INTER_AREA);

  truth_[seq] = {box, !occluded, 1000000000LL + int64_t(t_target * 1e9)};
  if (truth_.size() > 64) truth_.erase(truth_.begin());

  FrameView& v = f->view;
  v = FrameView{};
  v.seq = seq;
  v.t_sof_ns = 1000000000LL + int64_t(t * 1e9);
  v.exposure_us = exp_us;
  v.analog_gain = last_ctrl_.analog_gain;
  v.lens_position = cam_.lens_position;
  v.main_y = y_.data; v.main_w = y_.cols; v.main_h = y_.rows; v.main_stride = int(y_.step);
  v.main_u = u_.data; v.main_v = v_.data; v.main_uv_stride = int(u_.step);
  v.lo_y = lo_y_.data; v.lo_w = lo_y_.cols; v.lo_h = lo_y_.rows; v.lo_stride = int(lo_y_.step);
  v.lo_u = lo_u_.data; v.lo_v = lo_v_.data; v.lo_uv_stride = int(lo_u_.step);
  f->handle = nullptr;
  f->dropped = 0;
  return true;
}

std::optional<SyntheticSource::Truth> SyntheticSource::truth(uint32_t seq) const {
  auto it = truth_.find(seq);
  if (it == truth_.end()) return std::nullopt;
  return it->second;
}

OracleDetector::OracleDetector(const SyntheticSource& src, cv::Size input, OracleCfg c, uint32_t seed)
    : src_(src), in_(input), c_(c), rng_(seed) {}

bool OracleDetector::submit(DetRequest&& r) {
  if (int(pending_.size()) >= max_in_flight) return false;
  DetectionBatch b;
  b.frame_seq = r.frame_seq;
  b.t_sof_ns = r.t_sof_ns;
  b.kind = r.kind;
  b.crop = r.map.crop;
  b.t_submit_ns = r.t_submit_ns;
  std::uniform_real_distribution<double> U(0, 1);
  std::normal_distribution<double> N(0, c_.box_sigma_px);
  const auto t = src_.truth(r.frame_seq);
  const float k = r.map.scale / r.map.src_to_main;  // main px -> model input px
  if (t && t->visible) {
    const RectF& g = t->box;
    const bool inside = g.cx() >= b.crop.x && g.cy() >= b.crop.y && g.cx() <= b.crop.x + b.crop.w &&
                        g.cy() <= b.crop.y + b.crop.h;
    const bool big_enough = std::min(g.w, g.h) * k >= c_.min_side_in;
    if (inside && big_enough && U(rng_) > c_.p_miss) {
      Detection d;
      const float x0 = g.x + float(N(rng_)), y0 = g.y + float(N(rng_));
      const float x1 = g.x + g.w + float(N(rng_)), y1 = g.y + g.h + float(N(rng_));
      d.box = {x0, y0, x1 - x0, y1 - y0};
      d.score = float(0.55 + 0.4 * U(rng_));
      d.cls = ObjClass::Phone;
      b.dets.push_back(d);
    }
  }
  if (U(rng_) < c_.p_false) {
    Detection d;
    d.box = {b.crop.x + float(U(rng_)) * b.crop.w * 0.8f, b.crop.y + float(U(rng_)) * b.crop.h * 0.8f, 30, 60};
    d.score = float(0.3 + 0.4 * U(rng_));
    d.cls = ObjClass::Phone;
    b.dets.push_back(d);
  }
  pending_.push_back({r.frame_seq + uint32_t(c_.latency_frames), std::move(b)});
  return true;
}

std::optional<DetectionBatch> OracleDetector::poll() {
  // the pipeline polls at the start of frame N, i.e. after frame N-1 was fully processed
  if (pending_.empty() || pending_.front().first > src_.current_seq() - 1) return std::nullopt;
  DetectionBatch b = std::move(pending_.front().second);
  pending_.pop_front();
  b.t_done_ns = now_ns();
  return b;
}

}  // namespace rcws
