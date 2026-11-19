#include "sim/synthetic.hpp"

#include <opencv2/imgproc.hpp>

#include "common/clock.hpp"

namespace rcws {
namespace {

cv::Mat phone_texture(int w, int h, bool plain, std::mt19937& rng) {
  cv::Mat tex(h, w, CV_8UC1, cv::Scalar(25));
  if (plain) {
    tex.setTo(40);
    cv::rectangle(tex, cv::Rect(0, 0, tex.cols, tex.rows), cv::Scalar(70), std::max(2, tex.cols / 40));
    cv::rectangle(tex, cv::Rect(tex.cols / 10, tex.rows / 20, tex.cols / 3, tex.rows / 6), cv::Scalar(15), cv::FILLED);
    return tex;
  }
  const int m = std::max(2, int(w * 0.08));
  cv::Rect scr(m, 2 * m, tex.cols - 2 * m, tex.rows - 4 * m);
  tex(scr).setTo(200);
  std::uniform_int_distribution<int> ix(0, std::max(1, scr.width - 6)), iy(0, std::max(1, scr.height - 6));
  for (int i = 0; i < 10; ++i)
    cv::circle(tex, cv::Point(scr.x + ix(rng), scr.y + iy(rng)), std::max(1, m), cv::Scalar(90), cv::FILLED);
  return tex;
}

}  // namespace

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
  for (const auto& o : traj_.extra) extra_tex_.push_back(phone_texture(int(o.w), int(o.h), false, rng_));
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
  const bool colour = traj_.u != 128 || traj_.v != 128 || !traj_.extra.empty();
  if (colour) { u_.setTo(128); v_.setTo(128); }
  // sub-pixel placement via affine warp so ZNCC sub-pixel accuracy is testable
  auto draw = [&](const cv::Mat& tex, const RectF& b, uint8_t cu, uint8_t cv_) {
    cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, b.x, 0, 1, b.y);
    cv::Mat mask(tex.size(), CV_8UC1, cv::Scalar(255));
    cv::Mat warped, wmask;
    cv::warpAffine(tex, warped, M, y_.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    cv::warpAffine(mask, wmask, M, y_.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    // alpha blend with the fractional mask
    cv::Mat yf, wf, af;
    y_.convertTo(yf, CV_32F); warped.convertTo(wf, CV_32F); wmask.convertTo(af, CV_32F, 1.0 / 255);
    yf = yf.mul(1 - af) + wf.mul(af);
    yf.convertTo(y_, CV_8U);
    if (cu != 128 || cv_ != 128) {   // coloured case: chroma over the whole box (half resolution)
      cv::Rect r(int(b.x / 2), int(b.y / 2), int(b.w / 2), int(b.h / 2));
      r &= cv::Rect(0, 0, u_.cols, u_.rows);
      if (r.area() > 0) { u_(r).setTo(cu); v_(r).setTo(cv_); }
    }
  };
  if (!occluded) draw(tex_, box, traj_.u, traj_.v);
  else
    cv::rectangle(y_, cv::Rect(int(box.x) - 10, int(box.y) - 10, int(box.w) + 20, int(box.h) + 20), cv::Scalar(140), cv::FILLED);
  std::vector<Truth> all;
  const int64_t t_ns = 1000000000LL + int64_t(t_target * 1e9);
  all.push_back({box, !occluded, t_ns});
  for (size_t i = 0; i < traj_.extra.size(); ++i) {
    const SynthObj& o = traj_.extra[i];
    const cv::Point2d oc = o.center(t_target);
    const RectF ob{float(oc.x - o.w / 2), float(oc.y - o.h / 2), float(o.w), float(o.h)};
    const bool hidden = t >= o.occl_t0 && t < o.occl_t1;
    if (!hidden) draw(extra_tex_[i], ob, o.u, o.v);
    all.push_back({ob, !hidden, t_ns});
  }
  // pillar in front of everything; objects mostly behind it or mostly outside the image are invisible
  auto inside_frac = [&](const RectF& b, const cv::Rect& r) {
    const float x0 = std::max(b.x, float(r.x)), y0 = std::max(b.y, float(r.y));
    const float x1 = std::min(b.x + b.w, float(r.x + r.width)), y1 = std::min(b.y + b.h, float(r.y + r.height));
    return std::max(0.f, x1 - x0) * std::max(0.f, y1 - y0) / std::max(1.f, b.area());
  };
  if (traj_.occluder.area() > 0) {
    cv::rectangle(y_, traj_.occluder, cv::Scalar(150), cv::FILLED);
    cv::Rect r(traj_.occluder.x / 2, traj_.occluder.y / 2, traj_.occluder.width / 2, traj_.occluder.height / 2);
    r &= cv::Rect(0, 0, u_.cols, u_.rows);
    if (colour && r.area() > 0) { u_(r).setTo(128); v_(r).setTo(128); }
  }
  for (auto& tr : all) {
    if (traj_.occluder.area() > 0 && inside_frac(tr.box, traj_.occluder) > 0.5f) tr.visible = false;
    if (inside_frac(tr.box, cv::Rect(0, 0, cam_.main_w, cam_.main_h)) < 0.5f) tr.visible = false;
  }
  cv::Mat noise(y_.size(), CV_16S);
  cv::randn(noise, 0, traj_.noise_sigma);
  cv::Mat y16;
  y_.convertTo(y16, CV_16S);
  y16 += noise;
  y16.convertTo(y_, CV_8U);
  if (t >= traj_.dark_t0 && t < traj_.dark_t1) y_.convertTo(y_, CV_8U, traj_.dark_gain);
  cv::resize(y_, lo_y_, cv::Size(cam_.lores_w, cam_.lores_h), 0, 0, cv::INTER_AREA);
  if (colour) {
    cv::resize(u_, lo_u_, lo_u_.size(), 0, 0, cv::INTER_AREA);
    cv::resize(v_, lo_v_, lo_v_.size(), 0, 0, cv::INTER_AREA);
  }

  truth_[seq] = all.front();
  if (truth_.size() > 64) truth_.erase(truth_.begin());
  truth_all_[seq] = all;
  if (truth_all_.size() > 64) truth_all_.erase(truth_all_.begin());

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

std::vector<SyntheticSource::Truth> SyntheticSource::truth_all(uint32_t seq) const {
  auto it = truth_all_.find(seq);
  return it == truth_all_.end() ? std::vector<Truth>{} : it->second;
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
  const float k = r.map.scale / r.map.src_to_main;  // main px -> model input px
  const double p_miss = src_.dark(r.frame_seq) ? 0.9 : c_.p_miss;   // low light: the detector fails
  std::vector<Detection> found;
  for (const auto& g : src_.truth_all(r.frame_seq)) {
    if (!g.visible) continue;
    const RectF& gb = g.box;
    const bool inside = gb.cx() >= b.crop.x && gb.cy() >= b.crop.y && gb.cx() <= b.crop.x + b.crop.w &&
                        gb.cy() <= b.crop.y + b.crop.h;
    const bool big_enough = std::min(gb.w, gb.h) * k >= c_.min_side_in;
    if (!inside || !big_enough || U(rng_) <= p_miss) continue;
    Detection d;
    std::normal_distribution<double> Nr(0, c_.box_sigma_rel * std::max(gb.w, gb.h) + 1e-9);
    const float x0 = gb.x + float(N(rng_) + Nr(rng_)), y0 = gb.y + float(N(rng_) + Nr(rng_));
    const float x1 = gb.x + gb.w + float(N(rng_) + Nr(rng_)), y1 = gb.y + gb.h + float(N(rng_) + Nr(rng_));
    d.box = {x0, y0, x1 - x0, y1 - y0};
    d.score = float(0.55 + 0.4 * U(rng_));
    d.cls = ObjClass::Phone;
    found.push_back(d);
  }
  // two phones overlapping on screen: YOLO often returns one box around both (scenario B)
  for (size_t i = 0; i < found.size(); ++i)
    for (size_t j = i + 1; j < found.size(); ++j) {
      if (iou(found[i].box, found[j].box) < 0.25f || U(rng_) > src_.traj().merge_p) continue;
      const float x0 = std::min(found[i].box.x, found[j].box.x), y0 = std::min(found[i].box.y, found[j].box.y);
      const float x1 = std::max(found[i].box.x + found[i].box.w, found[j].box.x + found[j].box.w);
      const float y1 = std::max(found[i].box.y + found[i].box.h, found[j].box.y + found[j].box.h);
      found[i].box = {x0, y0, x1 - x0, y1 - y0};
      found[i].score = std::min(found[i].score, found[j].score);
      found.erase(found.begin() + long(j));
      --j;
    }
  for (auto& d : found) b.dets.push_back(d);
  const double p_false = src_.traj().p_false >= 0 ? src_.traj().p_false : c_.p_false;
  if (U(rng_) < p_false) {
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

namespace rcws {

const char* scenario_names() { return "sine parallel cross merge pillar longocc exit dark clutter"; }

// Scenario set of the v4 test plan (scenario sheet in README sec. 7). Image 1536x864, phone ~36x74 px.
SynthTraj make_scenario(const std::string& name, const CameraCfg& cam, int* frames, bool* ok) {
  const double cx = cam.main_w / 2.0, cy = cam.main_h / 2.0, W = cam.main_w;
  SynthTraj tr;
  *ok = true;
  int sec = 6;
  auto sine = [cx, cy](double t) { return cv::Point2d(cx + 300 * std::sin(0.8 * t), cy + 120 * std::sin(1.3 * t)); };
  // case colours (U, V): red-ish, blue-ish, green-ish
  constexpr uint8_t RU = 100, RV = 180, BU = 180, BV = 100, GU = 90, GV = 90;
  if (name == "sine") {                      // single moving target (v3 default) with a short occlusion
    tr.center = sine;
    tr.occl_t0 = 6.0; tr.occl_t1 = 6.3;
    sec = 10;
  } else if (name == "parallel") {           // two phones moving side by side: two IDs, no switch
    tr.center = [cx, cy](double t) { return cv::Point2d(cx - 150 + 200 * std::sin(0.9 * t), cy - 130); };
    tr.u = RU; tr.v = RV;
    tr.extra.push_back({[cx, cy](double t) { return cv::Point2d(cx - 150 + 200 * std::sin(0.9 * t), cy + 130); }, 36, 74, BU, BV});
  } else if (name == "cross") {              // two phones crossing (different colours): IDs must survive
    tr.center = [cx, cy](double t) { return cv::Point2d(cx - 420 + 150 * t, cy + 14); };
    tr.u = RU; tr.v = RV;
    tr.extra.push_back({[cx, cy](double t) { return cv::Point2d(cx + 420 - 150 * t, cy - 14); }, 36, 74, BU, BV});
  } else if (name == "merge") {              // same-looking phones overlap completely, then split
    tr.center = [cx, cy](double t) { return cv::Point2d(cx - 300 + 100 * t, cy); };
    tr.extra.push_back({[cx, cy](double t) { return cv::Point2d(cx + 300 - 100 * t, cy); }, 36, 74, 128, 128});
    tr.merge_p = 0.8;
  } else if (name == "pillar") {             // short occlusion behind a pillar, a second phone elsewhere
    tr.center = [cx, cy](double t) { return cv::Point2d(cx - 500 + 160 * t, cy + 40 * std::sin(1.1 * t)); };
    tr.u = RU; tr.v = RV;
    tr.occluder = cv::Rect(int(cx) - 20, 0, 40, cam.main_h);
    tr.extra.push_back({[cx, cy](double t) { return cv::Point2d(cx + 380, cy - 220 + 30 * std::sin(0.7 * t)); }, 36, 74, GU, GV});
  } else if (name == "longocc") {            // long occlusion (1.5 s): LOST, then re-identified by colour
    tr.center = [cx, cy](double t) { return cv::Point2d(cx + 80 * std::sin(0.6 * t), cy + 40 * std::sin(0.9 * t)); };
    tr.u = RU; tr.v = RV;
    tr.occl_t0 = 2.0; tr.occl_t1 = 3.5;
    tr.extra.push_back({[cx, cy](double t) { return cv::Point2d(cx - 450, cy + 200 + 20 * std::sin(t)); }, 36, 74, BU, BV});
  } else if (name == "exit") {               // leaves through the right edge, comes back 1.2 s later
    tr.center = [cx, cy, W](double t) {
      if (t < 3.5) return cv::Point2d(cx + 250 * t, cy - 60);
      if (t < 4.7) return cv::Point2d(W + 200, cy - 60);
      return cv::Point2d(W + 30 - 250 * (t - 4.7), cy - 60);
    };
    tr.u = RU; tr.v = RV;
    tr.extra.push_back({[cx, cy](double t) { return cv::Point2d(cx - 420, cy + 160 + 25 * std::sin(0.8 * t)); }, 36, 74, BU, BV});
    sec = 8;
  } else if (name == "dark") {               // light goes off for 1 s
    tr.center = sine;
    tr.dark_t0 = 2.0; tr.dark_t1 = 3.0;
  } else if (name == "clutter") {            // look-alike decoy and frequent false detections
    tr.center = sine;
    tr.decoy = true;
    tr.p_false = 0.08;
  } else {
    *ok = false;
  }
  if (frames && *frames <= 0) *frames = sec * cam.fps;
  return tr;
}

}  // namespace rcws
