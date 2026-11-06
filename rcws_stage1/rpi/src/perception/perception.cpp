#include "perception/perception.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

#include "common/clock.hpp"
#include "perception/roi.hpp"

namespace rcws {
namespace {

constexpr size_t kHistFrames = 12;     // ~100 ms at 120 fps: covers detector latency
constexpr size_t kNccHist = 32;
constexpr double kNoMeasCoast = 0.10;  // s without any accepted measurement -> COAST
constexpr double kAcquireTimeout = 1.0;

inline double s_of(int64_t ns) { return ns * 1e-9; }

// Crop an I420 image region (planes + strides) to RGB. r is aligned to even coordinates.
cv::Mat i420_crop_to_rgb(const uint8_t* py, const uint8_t* pu_src, const uint8_t* pv_src, int w, int h, int ys,
                         int uvs, cv::Rect r) {
  cv::Mat rgb;
  const cv::Mat y(h, w, CV_8UC1, const_cast<uint8_t*>(py), ys);
  if (!pu_src || !pv_src) {
    cv::cvtColor(y(r), rgb, cv::COLOR_GRAY2RGB);
    return rgb;
  }
  r.x &= ~1; r.y &= ~1; r.width &= ~1; r.height &= ~1;
  cv::Mat i420(r.height * 3 / 2, r.width, CV_8UC1);
  y(r).copyTo(i420(cv::Rect(0, 0, r.width, r.height)));
  const cv::Rect rc(r.x / 2, r.y / 2, r.width / 2, r.height / 2);
  uint8_t* pu = i420.ptr(r.height);
  uint8_t* pv = pu + (r.width / 2) * (r.height / 2);
  for (int j = 0; j < rc.height; ++j) {
    std::memcpy(pu + j * rc.width, pu_src + size_t(rc.y + j) * uvs + rc.x, rc.width);
    std::memcpy(pv + j * rc.width, pv_src + size_t(rc.y + j) * uvs + rc.x, rc.width);
  }
  cv::cvtColor(i420, rgb, cv::COLOR_YUV2RGB_I420);
  return rgb;
}

cv::Mat yuv420_crop_to_rgb(const FrameView& f, cv::Rect r) {
  return i420_crop_to_rgb(f.main_y, f.main_u, f.main_v, f.main_w, f.main_h, f.main_stride, f.main_uv_stride, r);
}

cv::Mat lores_to_rgb(const FrameView& f) {
  cv::Mat rgb;
  if (!f.lo_y) return rgb;
  const cv::Mat y(f.lo_h, f.lo_w, CV_8UC1, const_cast<uint8_t*>(f.lo_y), f.lo_stride);
  if (!f.lo_u || !f.lo_v) { cv::cvtColor(y, rgb, cv::COLOR_GRAY2RGB); return rgb; }
  cv::Mat i420(f.lo_h * 3 / 2, f.lo_w, CV_8UC1);
  y.copyTo(i420(cv::Rect(0, 0, f.lo_w, f.lo_h)));
  uint8_t* pu = i420.ptr(f.lo_h);
  uint8_t* pv = pu + (f.lo_w / 2) * (f.lo_h / 2);
  for (int j = 0; j < f.lo_h / 2; ++j) {
    std::memcpy(pu + j * (f.lo_w / 2), f.lo_u + j * f.lo_uv_stride, f.lo_w / 2);
    std::memcpy(pv + j * (f.lo_w / 2), f.lo_v + j * f.lo_uv_stride, f.lo_w / 2);
  }
  cv::cvtColor(i420, rgb, cv::COLOR_YUV2RGB_I420);
  return rgb;
}

}  // namespace

Perception::Perception(const Config& cfg, const CameraModel& cam, IDetector* det, BinLog* log, GimbalAt gimbal)
    : cfg_(cfg), cam_(cam), det_(det), log_(log), est_(cfg, cam, std::move(gimbal)), ncc_(cfg.ncc) {
  gain_ = cfg.cam.gain_init;
}

// ---------------------------------------------------------------- time (Sec 5.1)
int64_t Perception::t_cap_of(int64_t t_sof, int32_t exp_us, double row) const {
  const double row_sensor = row * double(cfg_.cam.sensor_h) / double(cfg_.cam.main_h);
  return t_sof + int64_t(cfg_.cam.timestamp_sign * exp_us * 500.0) + int64_t(row_sensor * cfg_.cam.line_time_ns);
}
int64_t Perception::t_cap_of(const FrameView& f, double row) const { return t_cap_of(f.t_sof_ns, f.exposure_us, row); }

// ---------------------------------------------------------------- history
void Perception::push_hist(const FrameView& f) {
  HistFrame h;
  if (!hist_.empty() && hist_.size() >= kHistFrames) {
    h.y = std::move(hist_.front().y);  // reuse buffer
    hist_.pop_front();
  }
  h.seq = f.seq;
  h.t_sof = f.t_sof_ns;
  h.exp_us = f.exposure_us;
  const cv::Mat y(f.main_h, f.main_w, CV_8UC1, const_cast<uint8_t*>(f.main_y), f.main_stride);
  y.copyTo(h.y);
  hist_.push_back(std::move(h));
}

const Perception::HistFrame* Perception::hist(uint32_t seq) const {
  for (auto it = hist_.rbegin(); it != hist_.rend(); ++it)
    if (it->seq == seq) return &*it;
  return nullptr;
}

// ---------------------------------------------------------------- state
void Perception::set_state(TrackState s, StateReason why, int64_t t) {
  if (s == state_) return;
  RecState r{t, uint8_t(state_), uint8_t(s), uint8_t(why), 0};
  if (log_) log_->write(RecType::STATE, r);
  state_ = s;
  t_state_ = t;
}

void Perception::reset_track() {
  est_.reset();
  ncc_.clear();
  ncc_hist_.clear();
  ncc_off_ = {0, 0};
  off_n_ = 0;
  strong_rejects_ = 0;
  sprt_L_ = 0;
  coast_ncc_only_ = false;
}

std::optional<RectF> Perception::target_box() const {
  if (state_ == TrackState::SEARCH) return std::nullopt;
  return tgt_box_;
}

VisionMeas Perception::det_meas(const Detection& d, const DetectionBatch& b) const {
  VisionMeas m;
  m.src = MeasSrc::DET;
  m.u = d.box.cx();
  m.v = d.box.cy();
  const HistFrame* h = hist(b.frame_seq);
  const int32_t exp = h ? h->exp_us : cfg_.cam.exposure_track_us;
  m.t_cap_ns = t_cap_of(b.t_sof_ns, exp, m.v);
  m.sigma_px = float(std::sqrt(cfg_.est.sigma_det_px0 * cfg_.est.sigma_det_px0 + cfg_.est.sigma_det_c * d.box.area()));
  m.conf = d.score;
  m.w = d.box.w;
  m.h = d.box.h;
  m.frame_seq = b.frame_seq;
  return m;
}

void Perception::log_meas(const VisionMeas& m, bool accepted, double d2) {
  if (!log_) return;
  RecMeas r{};
  r.t_cap = m.t_cap_ns; r.seq = m.frame_seq; r.u = m.u; r.v = m.v; r.sigma = m.sigma_px;
  r.conf = m.conf; r.w = m.w; r.h = m.h; r.d2 = float(d2);
  r.src = uint8_t(m.src); r.accepted = accepted;
  log_->write(RecType::MEAS, r);
}

void Perception::log_est(int64_t t) {
  if (!log_) return;
  const EstState s = est_.predict(t);
  if (!s.valid) return;
  RecEst r{t, s.psi, s.psi_d, s.psi_dd, s.theta, s.theta_d, s.theta_dd, s.sig_psi, s.sig_theta,
           float(s.mu_ca), float(s.range_m)};
  log_->write(RecType::EST, r);
}

// ---------------------------------------------------------------- detections
void Perception::handle_batch(const DetectionBatch& b, uint32_t now_seq, int64_t now) {
  stats_.det_batches++;
  if (log_) log_->write_det(b, now_seq);
  last_batch_ = b;
  const auto& tc = cfg_.det;

  // collect persons for context search (Sec 3.13.1)
  if (state_ == TrackState::SEARCH || state_ == TrackState::REACQ) {
    for (const auto& d : b.dets)
      if (d.cls == ObjClass::Person && d.score >= tc.thr_acquire && person_rois_.size() < 4)
        person_rois_.push_back(person_hand_roi(d.box, cfg_.cam.main_w, cfg_.cam.main_h));
  }

  // best phone candidate: in SEARCH by score; otherwise best gated (score x likelihood)
  const Detection* best = nullptr;
  const Detection* strong = nullptr;  // best confident phone that failed the gate
  VisionMeas best_m;
  double best_d2 = 0, best_key = -1;
  for (const auto& d : b.dets) {
    if (d.cls != ObjClass::Phone) continue;
    const VisionMeas m = det_meas(d, b);
    if (state_ == TrackState::SEARCH) {
      if (d.score >= tc.thr_acquire && d.score > best_key) { best = &d; best_m = m; best_key = d.score; }
      continue;
    }
    const GateResult g = est_.gate(m);
    if (!g.pass) {
      log_meas(m, false, g.d2);
      stats_.meas_gated_out++;
      if (d.score >= tc.thr_acquire && (!strong || d.score > strong->score)) strong = &d;
      continue;
    }
    const double key = d.score * std::exp(-0.5 * g.d2);
    if (key > best_key) { best = &d; best_m = m; best_d2 = g.d2; best_key = key; }
  }

  const double hit = std::log(cfg_.trk.sprt_pd / cfg_.trk.sprt_pfa);
  const double miss = std::log((1 - cfg_.trk.sprt_pd) / (1 - cfg_.trk.sprt_pfa));
  const double A = std::log((1 - cfg_.trk.sprt_beta) / cfg_.trk.sprt_alpha);
  const double B = std::log(cfg_.trk.sprt_beta / (1 - cfg_.trk.sprt_alpha));

  switch (state_) {
    case TrackState::SEARCH:
      if (best) {
        reset_track();
        est_.update(best_m);
        est_.update_range(best_m);
        log_meas(best_m, true, 0);
        tgt_box_ = best->box;
        t_last_meas_ = best_m.t_cap_ns;
        t_last_confirm_ = now;
        sprt_L_ = hit;
        set_state(TrackState::ACQUIRE, StateReason::DET_ACQUIRE, now);
      }
      break;

    case TrackState::ACQUIRE: {
      // only batches whose crop covered the prediction carry evidence
      cv::Point2d p;
      double sp = 0;
      const bool covered = est_.predict_pixel(b.t_sof_ns, &p, &sp) && p.x >= b.crop.x && p.y >= b.crop.y &&
                           p.x <= b.crop.x + b.crop.w && p.y <= b.crop.y + b.crop.h;
      if (!covered) break;
      const bool ok = best && best->score >= tc.thr_track;
      // a gated but weak detection (COCO 'cell phone' often scores 0.2-0.3) is neither evidence
      // for nor against: only a missing detection counts as a miss
      sprt_L_ = cfg_.trk.sprt_forget * sprt_L_ + (ok ? hit : (best ? 0.0 : miss));
      if (ok && best_m.t_cap_ns >= t_last_meas_) {
        est_.update(best_m);
        est_.update_range(best_m);
        log_meas(best_m, true, best_d2);
        tgt_box_ = best->box;
        t_last_meas_ = best_m.t_cap_ns;
        t_last_confirm_ = now;
      }
      if (sprt_L_ >= A) {
        // template is optional: a featureless target is tracked by detections alone
        const HistFrame* h = hist(b.frame_seq);
        if (h && ncc_.init(h->y, tgt_box_)) { ncc_off_ = ncc_.anchor(); off_n_ = 0; }
        else ncc_.clear();
        set_state(TrackState::TRACK, StateReason::SPRT_ACCEPT, now);
        t_last_confirm_ = now;
      } else if (sprt_L_ <= B) {
        reset_track();
        set_state(TrackState::SEARCH, StateReason::SPRT_REJECT, now);
      }
      break;
    }

    case TrackState::TRACK:
    case TrackState::COAST:
    case TrackState::REACQ: {
      if (best) strong_rejects_ = 0;
      else if (strong && ++strong_rejects_ >= cfg_.trk.strong_reject_reinit) {
        // The detector keeps seeing a confident phone where the filter does not expect it:
        // the filter is wrong, not the detector. Re-anchor the track on the detection.
        const VisionMeas m = det_meas(*strong, b);
        est_.reset();
        est_.update(m);
        est_.update_range(m);
        log_meas(m, true, 0);
        tgt_box_ = strong->box;
        t_last_meas_ = m.t_cap_ns;
        t_last_confirm_ = now;
        const HistFrame* h = hist(b.frame_seq);
        if (h && ncc_.init(h->y, strong->box)) { ncc_off_ = ncc_.anchor(); off_n_ = 0; }
        else ncc_.clear();
        ncc_hist_.clear();
        strong_rejects_ = 0;
        stats_.template_reinit++;
        if (state_ != TrackState::TRACK) set_state(TrackState::TRACK, StateReason::REACQ_DET, now);
        break;
      }
      if (!best || best->score < tc.thr_track) break;
      const HistFrame* h = hist(b.frame_seq);
      // NCC result for the same frame, if any (drift check, Sec 3.6)
      const NccHist* nh = nullptr;
      for (const auto& n : ncc_hist_) if (n.seq == b.frame_seq) nh = &n;
      const float side = std::max(best->box.w, best->box.h);
      bool reinit = !ncc_.has_template() || state_ == TrackState::REACQ;
      if (nh) {
        const double dist = std::hypot(nh->u - best->box.cx(), nh->v - best->box.cy());
        if (dist > cfg_.ncc.drift_frac * side) reinit = true;
        else {
          // The template centre inherits the first detection's box error. Averaging
          // DET - NCC over many confirmations removes that anchor bias: gain 1/n until it
          // reaches a floor, so the residual shrinks like sigma_det / sqrt(n).
          ++off_n_;
          const double g = std::max(1.0 / off_n_, 0.03);
          ncc_off_.x += g * (best->box.cx() - nh->u);
          ncc_off_.y += g * (best->box.cy() - nh->v);
        }
        if (!reinit && h && best->score >= cfg_.ncc.update_score) {
          const RectF nb{float(nh->u - tgt_box_.w / 2), float(nh->v - tgt_box_.h / 2), tgt_box_.w, tgt_box_.h};
          // blend the patch at the NCC-measured template position (not at the noisy DET box),
          // so refreshing never shifts the template content
          const double ru = nh->u - ncc_off_.x, rv = nh->v - ncc_off_.y;
          const RectF at{float(ru - tgt_box_.w / 2), float(rv - tgt_box_.h / 2), tgt_box_.w, tgt_box_.h};
          if (iou(nb, best->box) >= cfg_.ncc.update_iou) ncc_.refresh(h->y, at);
        }
      }
      if (reinit) {
        if (h && ncc_.init(h->y, best->box)) { stats_.template_reinit++; ncc_off_ = ncc_.anchor(); off_n_ = 0; }
        else if (!h) stats_.stale_det++;
      }
      // Position update from DET only when NCC did not measure that frame and ordering holds
      if (!nh && best_m.t_cap_ns >= t_last_meas_) {
        est_.update(best_m);
        log_meas(best_m, true, best_d2);
        t_last_meas_ = best_m.t_cap_ns;
      }
      est_.update_range(best_m);
      tgt_box_ = best->box;
      t_last_confirm_ = now;
      if (state_ != TrackState::TRACK)
        set_state(TrackState::TRACK, state_ == TrackState::REACQ ? StateReason::REACQ_DET : StateReason::MEAS_BACK, now);
      coast_ncc_only_ = false;
      break;
    }
  }
}

// ---------------------------------------------------------------- NCC (every frame)
void Perception::run_ncc(const FrameView& f, int64_t now) {
  if (!ncc_.has_template()) return;
  if (state_ != TrackState::TRACK && !(state_ == TrackState::COAST && !coast_ncc_only_)) return;
  cv::Point2d pred;
  double sp = 0;
  const int64_t t_pred = t_cap_of(f, tgt_box_.cy());
  if (!est_.predict_pixel(t_pred, &pred, &sp)) return;
  const cv::Mat y(f.main_h, f.main_w, CV_8UC1, const_cast<uint8_t*>(f.main_y), f.main_stride);
  const double radius = std::max(3.0 * sp, double(cfg_.ncc.search_min));
  // prediction too uncertain for a local search: leave it to the detector this frame
  if (radius > 1.5 * ncc_.side() * ncc_.scale() + 48) { stats_.ncc_fail++; return; }
  const NccResult r = ncc_.match(y, pred - ncc_off_, radius, f.seq);
  if (log_) {
    RecNcc rec{now, f.seq, float(r.u), float(r.v), r.rho, r.second, float(radius), int16_t(r.tmpl), uint8_t(r.ok),
               uint8_t(r.scale)};
    log_->write(RecType::NCC, rec);
  }
  if (!r.ok) { stats_.ncc_fail++; return; }
  stats_.ncc_ok++;
  const double mu = r.u + ncc_off_.x, mv = r.v + ncc_off_.y;
  ncc_hist_.push_back({f.seq, mu, mv, r.rho});
  if (ncc_hist_.size() > kNccHist) ncc_hist_.pop_front();

  VisionMeas m;
  m.src = MeasSrc::NCC;
  m.u = float(mu);
  m.v = float(mv);
  m.t_cap_ns = t_cap_of(f, mv);
  const double unconfirmed = std::max(0.0, s_of(now - t_last_confirm_));
  // sub-pixel accuracy holds at the matching scale: in full-res pixels it is x scale
  m.sigma_px = float(r.scale * std::max<double>(cfg_.ncc.sigma_floor_px, cfg_.ncc.sigma_n_px / std::sqrt(std::max(r.rho, 0.1f))) +
                     cfg_.ncc.sigma_growth_px_per_s * unconfirmed);
  m.conf = r.rho;
  m.w = tgt_box_.w;
  m.h = tgt_box_.h;
  m.frame_seq = f.seq;
  const GateResult g = est_.gate(m);
  log_meas(m, g.pass && m.t_cap_ns >= t_last_meas_, g.d2);
  if (!g.pass) { stats_.meas_gated_out++; return; }
  if (m.t_cap_ns < t_last_meas_) return;
  est_.update(m);
  t_last_meas_ = m.t_cap_ns;
  tgt_box_.x = m.u - tgt_box_.w / 2;
  tgt_box_.y = m.v - tgt_box_.h / 2;
  if (state_ == TrackState::COAST) set_state(TrackState::TRACK, StateReason::MEAS_BACK, now);
}

// ---------------------------------------------------------------- timers
void Perception::update_timers(int64_t now) {
  const double in_state = s_of(now - t_state_);
  switch (state_) {
    case TrackState::ACQUIRE:
      if (in_state > kAcquireTimeout) { reset_track(); set_state(TrackState::SEARCH, StateReason::ACQ_TIMEOUT, now); }
      break;
    case TrackState::TRACK: {
      // t_last_meas_ is a capture time; compare against wall time with the pipeline latency included
      const double since_meas = s_of(now - t_last_meas_);
      if (s_of(now - t_last_confirm_) > cfg_.ncc.ncc_only_max_s) {
        coast_ncc_only_ = true;
        set_state(TrackState::COAST, StateReason::NCC_ONLY, now);
      } else if (since_meas > kNoMeasCoast + 0.05) {
        set_state(TrackState::COAST, StateReason::NO_MEAS, now);
      }
      break;
    }
    case TrackState::COAST:
      if (in_state > cfg_.trk.coast_s) set_state(TrackState::REACQ, StateReason::COAST_TIMEOUT, now);
      break;
    case TrackState::REACQ:
      if (in_state > cfg_.trk.reacq_s) { reset_track(); set_state(TrackState::SEARCH, StateReason::REACQ_TIMEOUT, now); }
      break;
    default: break;
  }
}

// ---------------------------------------------------------------- detector scheduling
bool Perception::submit_main_crop(const FrameView& f, cv::Rect r, DetKind kind) {
  r = clamp_shift(r, f.main_w & ~1, f.main_h & ~1);
  if (r.width < 16 || r.height < 16) return false;
  const cv::Size in = det_->input_size();
  // v3: plane-domain downscale (detector.preproc_i420). Only when the region is larger than the
  // model input; for upscaling the v2 path converts fewer pixels and is kept.
  auto plane_path = [&](int w, int h, const uint8_t* u, const uint8_t* v) {
    return cfg_.det.preproc_i420 && u && v && (w > in.width || h > in.height);
  };
  auto submit = [&](DetRequest&& req) {
    const bool ok = det_->submit(std::move(req));
    ok ? stats_.det_submitted++ : stats_.det_rejected++;
    return ok;
  };
  // Regions larger than the model input are downscaled anyway; cutting them from lores
  // (2.4x smaller) avoids a full-resolution YUV->RGB conversion. Targets this large keep
  // >= 40 px in lores. Field run 2: 864 px ROIs from main cost ~10 ms per frame on Pi 5.
  // detector.lores_large_roi = false keeps the earlier rule (lores only without detail loss) so the
  // v3 plane-domain downscale of the main crop can be compared on the same clip.
  const bool use_lores = cfg_.det.lores_large_roi ? r.width > in.width
                                                  : double(r.width) * f.lo_w / f.main_w >= 0.9 * in.width;
  if (f.lo_y && use_lores) {
    const double k = double(f.lo_w) / f.main_w;
    cv::Rect lr(int(r.x * k) & ~1, int(r.y * k) & ~1, int(r.width * k) & ~1, int(r.height * k) & ~1);
    lr = clamp_shift(lr, f.lo_w & ~1, f.lo_h & ~1);
    const float to_main = float(f.main_w) / float(f.lo_w);
    if (plane_path(lr.width, lr.height, f.lo_u, f.lo_v)) {
      const I420View v{f.lo_y, f.lo_u, f.lo_v, f.lo_w, f.lo_h, f.lo_stride, f.lo_uv_stride};
      return submit(make_request_i420(v, lr, in, to_main, f.seq, f.t_sof_ns, kind, cfg_.det.linear_mild_downscale));
    }
    cv::Mat rgb = i420_crop_to_rgb(f.lo_y, f.lo_u, f.lo_v, f.lo_w, f.lo_h, f.lo_stride, f.lo_uv_stride, lr);
    DetRequest req = make_request(rgb, cv::Rect(0, 0, rgb.cols, rgb.rows), in, to_main, f.seq, f.t_sof_ns, kind);
    req.map.crop = {lr.x * to_main, lr.y * to_main, lr.width * to_main, lr.height * to_main};
    return submit(std::move(req));
  }
  if (plane_path(r.width, r.height, f.main_u, f.main_v)) {
    const I420View v{f.main_y, f.main_u, f.main_v, f.main_w, f.main_h, f.main_stride, f.main_uv_stride};
    return submit(make_request_i420(v, r, in, 1.f, f.seq, f.t_sof_ns, kind, cfg_.det.linear_mild_downscale));
  }
  cv::Mat rgb = yuv420_crop_to_rgb(f, r);
  DetRequest req = make_request(rgb, cv::Rect(0, 0, rgb.cols, rgb.rows), in, 1.f, f.seq, f.t_sof_ns, kind);
  req.map.crop = {float(r.x), float(r.y), float(r.width), float(r.height)};
  return submit(std::move(req));
}

bool Perception::submit_global(const FrameView& f) {
  if (f.lo_y) {
    // lores (640x360) already fits a 640x640 input: s = 1, so the plane path is not taken here
    cv::Mat rgb = lores_to_rgb(f);
    DetRequest req = make_request(rgb, cv::Rect(0, 0, rgb.cols, rgb.rows), det_->input_size(),
                                  float(f.main_w) / float(f.lo_w), f.seq, f.t_sof_ns, DetKind::Global);
    const bool ok = det_->submit(std::move(req));
    ok ? stats_.det_submitted++ : stats_.det_rejected++;
    return ok;
  }
  return submit_main_crop(f, cv::Rect(0, 0, f.main_w, f.main_h), DetKind::Global);
}

void Perception::schedule(const FrameView& f) {
  if (!det_) return;
  if (tiles_.empty()) tiles_ = make_tiles(cfg_.roi, f.main_w, f.main_h);
  const int free_slots = cfg_.det.max_in_flight - det_->in_flight();
  if (free_slots <= 0) return;
  ++frames_since_det_;

  cv::Point2d pred;
  double sp = 0;
  const bool have_pred = est_.predict_pixel(t_cap_of(f, tgt_box_.cy()), &pred, &sp);

  switch (state_) {
    case TrackState::SEARCH: {
      if (!person_rois_.empty()) {   // context first (Sec 3.13.1)
        submit_main_crop(f, person_rois_.front(), DetKind::Roi);
        person_rois_.pop_front();
      } else if (frame_counter_ % (1 + cfg_.roi.tiles_per_frame) == 0) {
        submit_global(f);
      } else {
        submit_main_crop(f, tiles_[tile_idx_++ % tiles_.size()], DetKind::Tile);
      }
      break;
    }
    case TrackState::ACQUIRE:
      if (have_pred) submit_main_crop(f, track_roi(cfg_.roi, pred, tgt_box_.w, tgt_box_.h, sp, f.main_w, f.main_h), DetKind::Roi);
      break;
    case TrackState::TRACK: {
      // confidence-based scheduling (Sec 3.13.10)
      const bool good = !ncc_hist_.empty() && ncc_hist_.back().seq == f.seq && ncc_hist_.back().rho >= 0.85f;
      const int period = good ? cfg_.roi.track_det_period_hi : cfg_.roi.track_det_period_lo;
      if (frames_since_det_ >= period && have_pred) {
        if (submit_main_crop(f, track_roi(cfg_.roi, pred, tgt_box_.w, tgt_box_.h, sp, f.main_w, f.main_h), DetKind::Roi))
          frames_since_det_ = 0;
      }
      break;
    }
    case TrackState::COAST:
    case TrackState::REACQ: {
      if (!have_pred) { submit_global(f); break; }
      const cv::Rect r = track_roi(cfg_.roi, pred, tgt_box_.w, tgt_box_.h, sp, f.main_w, f.main_h);
      if (r.width >= int(cfg_.roi.s_max) - 2) {  // prediction too uncertain: widen to search
        if (frame_counter_ % 2) submit_global(f);
        else submit_main_crop(f, tiles_[tile_idx_++ % tiles_.size()], DetKind::Tile);
      } else {
        submit_main_crop(f, r, DetKind::Roi);
      }
      break;
    }
  }
}

// ---------------------------------------------------------------- camera control policy
FrameControls Perception::controls(const FrameView& f) {
  FrameControls c;
  const bool searching = state_ == TrackState::SEARCH || state_ == TrackState::REACQ;
  c.exposure_us = searching ? cfg_.cam.exposure_search_us : cfg_.cam.exposure_track_us;
  // ROI-referenced gain (Sec 3.13.5): exposure fixed, gain follows target luma
  if (cfg_.cam.roi_gain_control && state_ == TrackState::TRACK) {
    cv::Rect r(int(tgt_box_.x), int(tgt_box_.y), int(tgt_box_.w), int(tgt_box_.h));
    r &= cv::Rect(0, 0, f.main_w, f.main_h);
    if (r.area() > 16) {
      const cv::Mat y(f.main_h, f.main_w, CV_8UC1, const_cast<uint8_t*>(f.main_y), f.main_stride);
      cv::Mat roi = y(r);
      int hist[256] = {0};
      for (int j = 0; j < roi.rows; ++j) {
        const uint8_t* p = roi.ptr(j);
        for (int i = 0; i < roi.cols; ++i) hist[p[i]]++;
      }
      // trimmed mean: drop the brightest 5 % (saturated phone screen)
      const int keep = int(0.95 * r.area());
      long sum = 0;
      int n = 0;
      for (int v = 0; v < 256 && n < keep; ++v) {
        const int take = std::min(hist[v], keep - n);
        sum += long(take) * v;
        n += take;
      }
      const double mean = n ? double(sum) / n : cfg_.cam.roi_target_luma;
      const double ratio = cfg_.cam.roi_target_luma / std::max(mean, 1.0);
      gain_ = float(std::clamp(gain_ * std::pow(ratio, double(cfg_.cam.gain_kappa)), double(cfg_.cam.gain_min),
                               double(cfg_.cam.gain_max)));
    }
  } else if (searching) {
    gain_ = cfg_.cam.gain_init * float(cfg_.cam.exposure_track_us) / std::max(1, cfg_.cam.exposure_search_us);
    gain_ = std::clamp(gain_, cfg_.cam.gain_min, cfg_.cam.gain_max);
  }
  c.analog_gain = gain_;
  return c;
}

// ---------------------------------------------------------------- main entry
FrameControls Perception::on_frame(const FrameView& f) {
  const int64_t t_in = now_ns();
  // Logic clock = capture time of the frame centre row. All state timing uses it, so a
  // replayed or synthetic sequence takes the same decisions as the live run (Sec 12.4).
  const int64_t now = t_cap_of(f, f.main_h / 2.0);
  stats_.frames++;
  frame_counter_++;
  cur_seq_ = f.seq;
  push_hist(f);
  if (t_state_ == 0) t_state_ = now;

  if (det_)
    while (auto b = det_->poll()) handle_batch(*b, f.seq, now);

  run_ncc(f, now);
  update_timers(now);
  schedule(f);
  const FrameControls c = controls(f);
  log_est(t_cap_of(f, f.main_h / 2.0));

  if (log_) {
    RecFrame r{};
    r.seq = f.seq; r.t_sof = f.t_sof_ns; r.t_in = t_in; r.t_out = now_ns();
    r.boot_minus_mono = 0;  // filled by the source-specific path when available
    r.exposure_us = f.exposure_us; r.gain = f.analog_gain; r.lens = f.lens_position;
    r.state = uint8_t(state_); r.in_flight = uint8_t(det_ ? det_->in_flight() : 0);
    log_->write(RecType::FRAME, r);
  }
  return c;
}

}  // namespace rcws
