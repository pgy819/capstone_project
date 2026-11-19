#include "perception/mot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rcws {
namespace {

constexpr double kInf = 1e9;
inline double s_of(int64_t ns) { return ns * 1e-9; }
inline float clamp01(double v) { return float(std::clamp(v, 0.0, 1.0)); }

// 2x2 CV predict for one axis: p, v and covariance [pp pv; vp vv]
void cv_predict(double& p, double& v, double* P, double dt, double q) {
  p += v * dt;
  const double pp = P[0] + dt * (P[1] + P[2]) + dt * dt * P[3] + q * dt * dt * dt / 3.0;
  const double pv = P[1] + dt * P[3] + q * dt * dt / 2.0;
  const double vv = P[3] + q * dt;
  P[0] = pp; P[1] = pv; P[2] = pv; P[3] = vv;
}

void cv_correct(double& p, double& v, double* P, double z, double R) {
  const double S = P[0] + R;
  const double k0 = P[0] / S, k1 = P[2] / S;
  const double y = z - p;
  p += k0 * y;
  v += k1 * y;
  const double pp = (1 - k0) * P[0], pv = (1 - k0) * P[1], vv = P[3] - k1 * P[1];
  P[0] = pp; P[1] = pv; P[2] = pv; P[3] = vv;
}

}  // namespace

const char* to_string(MotState s) {
  switch (s) {
    case MotState::TENTATIVE: return "TENT";
    case MotState::CONFIRMED: return "CONF";
    case MotState::OVERLAP: return "OVERLAP";
    case MotState::OCCLUDED: return "OCCL";
    case MotState::EXITING: return "EXIT";
    case MotState::AMBIGUOUS: return "AMBIG";
    case MotState::LOST: return "LOST";
  }
  return "?";
}

// ---------------------------------------------------------------- appearance
bool box_histogram(const FrameView& f, const RectF& box, AppHist* out) {
  if (!f.main_y) return false;
  // inner 60 % of the box: the edges carry background
  const float mx = 0.2f * box.w, my = 0.2f * box.h;
  int x0 = int(box.x + mx), y0 = int(box.y + my), x1 = int(box.x + box.w - mx), y1 = int(box.y + box.h - my);
  x0 = std::max(x0, 0); y0 = std::max(y0, 0);
  x1 = std::min(x1, f.main_w - 1); y1 = std::min(y1, f.main_h - 1);
  if (x1 - x0 < 4 || y1 - y0 < 4) return false;
  // at most ~1600 samples whatever the box size (a 400 px box costs the same as a 40 px one)
  const int step = std::max(2, int(std::sqrt(double(x1 - x0) * (y1 - y0) / 1600.0)) & ~1);
  out->fill(0.f);
  int n = 0;
  for (int y = y0; y <= y1; y += step) {
    const uint8_t* py = f.main_y + size_t(y) * f.main_stride;
    const uint8_t* pu = f.main_u ? f.main_u + size_t(y / 2) * f.main_uv_stride : nullptr;
    const uint8_t* pv = f.main_v ? f.main_v + size_t(y / 2) * f.main_uv_stride : nullptr;
    for (int x = x0; x <= x1; x += step) {
      const int u = pu ? pu[x / 2] >> 5 : 4, v = pv ? pv[x / 2] >> 5 : 4;
      (*out)[u * 8 + v] += 1.f;
      (*out)[64 + (py[x] >> 5)] += 1.f;
      ++n;
    }
  }
  if (n == 0) return false;
  for (auto& v : *out) v /= float(n);
  return true;
}

float hist_distance(const AppHist& a, const AppHist& b) {
  double bc_c = 0, bc_l = 0;
  for (int i = 0; i < 64; ++i) bc_c += std::sqrt(double(a[i]) * b[i]);
  for (int i = 64; i < kHistBins; ++i) bc_l += std::sqrt(double(a[i]) * b[i]);
  // chroma separates phones (case colours); luma alone says little (dark backs, bright screens)
  const double bc = 0.7 * bc_c + 0.3 * bc_l;
  return float(std::sqrt(std::max(0.0, 1.0 - bc)));
}

// ---------------------------------------------------------------- Hungarian (min cost)
std::vector<int> hungarian(const std::vector<double>& cost, int rows, int cols, double max_cost) {
  std::vector<int> out(rows, -1);
  if (rows == 0 || cols == 0) return out;
  const int n = std::max(rows, cols);
  auto a = [&](int i, int j) { return (i < rows && j < cols) ? std::min(cost[i * cols + j], max_cost) : max_cost; };
  // e-maxx potentials formulation, 1-based
  std::vector<double> u(n + 1, 0), v(n + 1, 0), minv(n + 1);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);
  std::vector<char> used(n + 1);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::fill(minv.begin(), minv.end(), std::numeric_limits<double>::max());
    std::fill(used.begin(), used.end(), 0);
    do {
      used[j0] = 1;
      const int i0 = p[j0];
      double delta = std::numeric_limits<double>::max();
      int j1 = 0;
      for (int j = 1; j <= n; ++j) {
        if (used[j]) continue;
        const double cur = a(i0 - 1, j - 1) - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] != 0);
    do { const int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
  }
  for (int j = 1; j <= n; ++j) {
    const int i = p[j] - 1, c = j - 1;
    if (i >= 0 && i < rows && c < cols && cost[i * cols + c] < max_cost) out[i] = c;
  }
  return out;
}

// ---------------------------------------------------------------- track board
const MotTrack* TrackBoard::find(int id) const {
  for (const auto& k : tracks_) if (k.id == id) return &k;
  return nullptr;
}

void TrackBoard::set_state(MotTrack& k, MotState s, int64_t t) {
  if (k.st == s) return;
  k.st = s;
  k.t_state = t;
}

void TrackBoard::predict(MotTrack& k, int64_t t) const {
  const double dt = s_of(t - k.t);
  if (dt <= 0) return;
  // a lost track stops moving (its velocity is no longer observed); occluded ones coast
  const bool frozen = k.st == MotState::LOST;
  double vu = frozen ? 0 : k.x[2], vv = frozen ? 0 : k.x[3];
  cv_predict(k.x[0], vu, k.Pu, dt, c_.q_px);
  cv_predict(k.x[1], vv, k.Pv, dt, c_.q_px);
  if (!frozen) { k.x[2] = vu; k.x[3] = vv; }
  k.t = t;
}

void TrackBoard::correct(MotTrack& k, const Detection& d, int64_t t, const AppHist* h) {
  const double side = std::max(d.box.w, d.box.h);
  const double r = c_.r_px0 + c_.r_rel * side;
  cv_correct(k.x[0], k.x[2], k.Pu, d.box.cx(), r * r);
  cv_correct(k.x[1], k.x[3], k.Pv, d.box.cy(), r * r);
  const double dt = s_of(t - k.t_seen);
  const double old_size = std::sqrt(std::max(1.f, k.w * k.h));
  k.w = k.hits ? 0.7f * k.w + 0.3f * d.box.w : d.box.w;
  k.h = k.hits ? 0.7f * k.h + 0.3f * d.box.h : d.box.h;
  if (k.hits && dt > 1e-3 && dt < 0.5) {
    const double rate = std::log(std::sqrt(std::max(1.f, k.w * k.h)) / old_size) / dt;
    k.log_size_rate = float(0.8 * k.log_size_rate + 0.2 * std::clamp(rate, -5.0, 5.0));
  }
  k.score = k.hits ? 0.8f * k.score + 0.2f * d.score : d.score;
  if (h) {
    if (!k.has_hist) { k.hist = *h; k.has_hist = true; }
    else for (int i = 0; i < kHistBins; ++i) k.hist[i] = 0.9f * k.hist[i] + 0.1f * (*h)[i];
  }
  k.t_seen = t;
  k.hits++;
  k.misses = 0;
}

double TrackBoard::cost(const MotTrack& k, const Detection& d, const AppHist* h, bool weak) const {
  const RectF kb = k.box();
  const float ov = iou(kb, d.box);
  const double ratio = std::sqrt(std::max(1.f, d.box.area()) / std::max(1.f, kb.area()));
  if (ratio > c_.size_jump || ratio < 1.0 / c_.size_jump) return kInf;
  const double side = std::max(d.box.w, d.box.h);
  const double r = c_.r_px0 + c_.r_rel * side;
  const double du = d.box.cx() - k.x[0], dv = d.box.cy() - k.x[1];
  const double d2 = du * du / (k.Pu[0] + r * r) + dv * dv / (k.Pv[0] + r * r);
  if (weak) {   // ByteTrack second stage: weak detections only extend a track they clearly overlap
    if (ov < 0.3f || d2 > c_.gate_chi2) return kInf;
    return 1.0 - ov;
  }
  if (d2 > c_.gate_chi2 && ov < 0.1f) return kInf;
  double c = c_.w_iou * (1.0 - ov) + c_.w_dist * std::min(1.0, std::sqrt(d2 / c_.gate_chi2));
  if (h && k.has_hist) c += c_.w_app * hist_distance(k.hist, *h);
  return c;
}

bool TrackBoard::near_edge(const MotTrack& k, float* dx, float* dy) const {
  const float mx = c_.edge_margin * W_, my = c_.edge_margin * H_;
  const RectF b = k.box();
  float ex = 0, ey = 0;
  if (b.x < mx && k.x[2] <= 0) ex = -1;
  if (b.x + b.w > W_ - mx && k.x[2] >= 0) ex = 1;
  if (b.y < my && k.x[3] <= 0) ey = -1;
  if (b.y + b.h > H_ - my && k.x[3] >= 0) ey = 1;
  if (ex == 0 && ey == 0) return false;
  const float n = std::hypot(ex, ey);
  *dx = ex / n;
  *dy = ey / n;
  return true;
}

MotUpdate TrackBoard::update(const DetectionBatch& b, const FrameView* f, float thr_high, float thr_low) {
  MotUpdate out;
  out.det_track.assign(b.dets.size(), -1);
  const int64_t t = b.t_sof_ns;
  // ego motion since the previous batch (gimbal turned -> the scene shifted in the image)
  for (auto& k : tracks_) { k.x[0] += ego_du_; k.x[1] += ego_dv_; }
  ego_du_ = ego_dv_ = 0;
  for (auto& k : tracks_) predict(k, t);

  // colour histograms of the phone detections (from the current frame: 1-2 frames newer than the
  // detection; the inner-box crop tolerates that shift)
  std::vector<int> hi, lo;
  std::vector<AppHist> hist(b.dets.size());
  std::vector<char> has_h(b.dets.size(), 0);
  for (size_t i = 0; i < b.dets.size(); ++i) {
    const auto& d = b.dets[i];
    if (d.cls != ObjClass::Phone) continue;
    if (d.score >= thr_high) {
      hi.push_back(int(i));
      if (f) has_h[i] = box_histogram(*f, d.box, &hist[i]);
    } else if (d.score >= thr_low) {
      lo.push_back(int(i));
    }
  }
  auto observable = [&](const MotTrack& k) {
    return k.x[0] >= b.crop.x && k.x[1] >= b.crop.y && k.x[0] <= b.crop.x + b.crop.w && k.x[1] <= b.crop.y + b.crop.h;
  };

  // stage 1: confident detections vs every live track (LOST ones are re-identified below)
  std::vector<int> live;
  for (size_t j = 0; j < tracks_.size(); ++j)
    if (tracks_[j].st != MotState::LOST) live.push_back(int(j));
  std::vector<char> matched(tracks_.size(), 0);
  std::vector<char> det_used(b.dets.size(), 0);
  auto assign = [&](const std::vector<int>& dets, bool weak) {
    std::vector<int> cand;
    for (int j : live) if (!matched[j]) cand.push_back(j);
    if (dets.empty() || cand.empty()) return;
    std::vector<double> C(dets.size() * cand.size());
    for (size_t r = 0; r < dets.size(); ++r)
      for (size_t c = 0; c < cand.size(); ++c)
        C[r * cand.size() + c] = cost(tracks_[cand[c]], b.dets[dets[r]], has_h[dets[r]] ? &hist[dets[r]] : nullptr, weak);
    const auto a = hungarian(C, int(dets.size()), int(cand.size()), weak ? 1.0 : c_.max_cost);
    for (size_t r = 0; r < dets.size(); ++r) {
      if (a[r] < 0) continue;
      MotTrack& k = tracks_[cand[a[r]]];
      const int di = dets[r];
      correct(k, b.dets[di], t, has_h[di] ? &hist[di] : nullptr);
      matched[cand[a[r]]] = 1;
      det_used[di] = 1;
      out.det_track[di] = k.id;
    }
  };
  assign(hi, false);
  assign(lo, true);

  // unmatched confident detections: re-identify a LOST track, else start a new one
  for (int di : hi) {
    if (det_used[di]) continue;
    const Detection& d = b.dets[di];
    bool dup = false;   // a second box on an already matched track (NMS leftovers)
    for (size_t j = 0; j < tracks_.size(); ++j)
      if (matched[j] && iou(tracks_[j].box(), d.box) > 0.5f) dup = true;
    if (dup) continue;
    int best = -1;
    double best_d = c_.reid_hist;
    for (size_t j = 0; j < tracks_.size(); ++j) {
      MotTrack& k = tracks_[j];
      if (k.st != MotState::LOST || matched[j]) continue;
      const double ratio = std::sqrt(std::max(1.f, d.box.area()) / std::max(1.f, k.w * k.h));
      if (ratio > c_.reid_size || ratio < 1.0 / c_.reid_size) continue;
      bool place_ok;
      if (k.exited) {   // re-entry: through the same side of the frame
        const float bx = k.exit_dx > 0 ? (W_ - d.box.cx()) / W_ : k.exit_dx < 0 ? d.box.cx() / W_ : 0.f;
        const float by = k.exit_dy > 0 ? (H_ - d.box.cy()) / H_ : k.exit_dy < 0 ? d.box.cy() / H_ : 0.f;
        place_ok = std::max(bx, by) < 0.3f;
      } else {
        const double rad = 3.0 * std::max(k.w, k.h) + c_.reid_px_per_s * s_of(t - k.t_seen);
        place_ok = std::hypot(d.box.cx() - k.x[0], d.box.cy() - k.x[1]) < rad;
      }
      if (!place_ok) continue;
      // colour is the only identity evidence once the track is lost: without it, no re-ID
      if (!has_h[di] || !k.has_hist) continue;
      const double hd = hist_distance(k.hist, hist[di]);
      if (hd < best_d) { best_d = hd; best = int(j); }
    }
    if (best >= 0) {
      MotTrack& k = tracks_[best];
      k.x[0] = d.box.cx(); k.x[1] = d.box.cy(); k.x[2] = 0; k.x[3] = 0;
      k.Pu[0] = k.Pv[0] = 25; k.Pu[1] = k.Pu[2] = k.Pv[1] = k.Pv[2] = 0; k.Pu[3] = k.Pv[3] = 500.0 * 500.0;
      k.t = t;
      k.hits = 0;
      correct(k, d, t, &hist[di]);
      k.hits = c_.n_init - 1;          // one more hit confirms the re-identified identity
      k.revived = true;   // `exited` is kept until confirmed: falling back to LOST keeps the exit side
      set_state(k, MotState::TENTATIVE, t);
      matched[best] = 1;
      out.det_track[di] = k.id;
      continue;
    }
    MotTrack k;
    k.id = next_id_++;
    k.x[0] = d.box.cx(); k.x[1] = d.box.cy();
    k.Pu[0] = k.Pv[0] = 25; k.Pu[3] = k.Pv[3] = 500.0 * 500.0;
    k.t = k.t_seen = k.t_born = k.t_state = t;
    correct(k, d, t, has_h[di] ? &hist[di] : nullptr);
    tracks_.push_back(k);
    matched.push_back(1);
    out.det_track[di] = k.id;
  }

  // state of every track after this batch
  for (size_t j = 0; j < tracks_.size(); ++j) {
    MotTrack& k = tracks_[j];
    if (matched[j]) {
      if (k.st == MotState::TENTATIVE && k.hits >= c_.n_init) { k.revived = false; k.exited = false; set_state(k, MotState::CONFIRMED, t); }
      else if (k.st == MotState::OCCLUDED || k.st == MotState::EXITING) set_state(k, MotState::CONFIRMED, t);
      continue;
    }
    if (k.st == MotState::LOST || !observable(k)) continue;
    k.misses++;
    if (k.st == MotState::TENTATIVE) continue;   // removed in tick()
    float dx = 0, dy = 0;
    if (k.st == MotState::EXITING || k.st == MotState::OCCLUDED) continue;
    if (near_edge(k, &dx, &dy)) {
      k.exit_dx = dx; k.exit_dy = dy;
      set_state(k, MotState::EXITING, t);
    } else if (k.st != MotState::AMBIGUOUS) {
      set_state(k, MotState::OCCLUDED, t);
    }
  }
  check_separation(t);
  // unconfirmed tracks that keep missing are dropped; a re-identified one falls back to LOST
  // (it keeps its identity for another re-identification)
  for (auto& k : tracks_)
    if (k.revived && k.st == MotState::TENTATIVE && k.misses >= c_.tentative_max_miss) { k.revived = false; set_state(k, MotState::LOST, t); }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const MotTrack& k) { return k.st == MotState::TENTATIVE && k.misses >= c_.tentative_max_miss; }),
                tracks_.end());
  return out;
}

void TrackBoard::check_separation(int64_t t) {
  // overlap bookkeeping on predicted boxes of established tracks
  for (auto& k : tracks_) {
    if (k.st == MotState::TENTATIVE || k.st == MotState::LOST) continue;
    int partner = -1;
    float best = c_.overlap_iou;
    for (const auto& o : tracks_) {
      if (o.id == k.id || o.st == MotState::TENTATIVE || o.st == MotState::LOST) continue;
      const float ov = iou(k.box(), o.box());
      if (ov > best) { best = ov; partner = o.id; }
    }
    if (partner >= 0) {
      k.partner = partner;
      k.was_overlapped = true;
      if (k.st == MotState::CONFIRMED) set_state(k, MotState::OVERLAP, t);
    } else if (k.was_overlapped && k.st != MotState::OCCLUDED) {
      // separated again: can the two identities be told apart?
      const MotTrack* o = find(k.partner);
      bool ambiguous = false;
      if (o && o->st != MotState::LOST) {
        ambiguous = !k.has_hist || !o->has_hist || hist_distance(k.hist, o->hist) < c_.amb_hist_margin;
      }
      k.was_overlapped = false;
      if (ambiguous) {
        if (k.st != MotState::AMBIGUOUS) ambiguous_events_++;
        k.t_ambiguous_until = t + int64_t(c_.amb_hold_s * 1e9);
        set_state(k, MotState::AMBIGUOUS, t);
      } else if (k.st == MotState::OVERLAP) {
        set_state(k, MotState::CONFIRMED, t);
      }
    }
  }
}

void TrackBoard::tick(int64_t t) {
  for (auto& k : tracks_) {
    const double in_state = s_of(t - k.t_state);
    const double unseen = s_of(t - k.t_seen);
    switch (k.st) {
      case MotState::OCCLUDED:
        if (in_state > c_.occl_s) set_state(k, MotState::LOST, t);
        break;
      case MotState::EXITING: {
        MotTrack p = k;
        predict(p, t);
        const bool out = p.x[0] < 0 || p.x[1] < 0 || p.x[0] > W_ || p.x[1] > H_;
        if (out || in_state > c_.occl_s) { k.exited = true; set_state(k, MotState::LOST, t); }
        break;
      }
      case MotState::AMBIGUOUS:
        if (t >= k.t_ambiguous_until) set_state(k, k.misses ? MotState::OCCLUDED : MotState::CONFIRMED, t);
        break;
      case MotState::CONFIRMED:
      case MotState::OVERLAP:
        if (unseen > c_.unseen_s) set_state(k, MotState::LOST, t);
        break;
      default: break;
    }
  }
  for (auto& k : tracks_)
    if (k.revived && k.st == MotState::TENTATIVE && s_of(t - k.t_seen) > 0.3) { k.revived = false; set_state(k, MotState::LOST, t); }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const MotTrack& k) {
                                 if (k.st == MotState::LOST) return s_of(t - k.t_state) > c_.lost_keep_s;
                                 if (k.st == MotState::TENTATIVE) return s_of(t - k.t_seen) > 1.0;
                                 return false;
                               }),
                tracks_.end());
}

// ---------------------------------------------------------------- priority (scenario A)
float TrackBoard::priority(const MotTrack& k, int64_t t) const {
  const double half_diag = 0.5 * std::hypot(W_, H_);
  const double centre = 1.0 - std::hypot(k.x[0] - W_ / 2.0, k.x[1] - H_ / 2.0) / half_diag;
  const double size = std::sqrt(std::max(1.f, k.w * k.h)) / c_.size_ref_px;
  const double approach = 0.5 + k.log_size_rate / (2.0 * c_.approach_ref);
  const double age = s_of(t - k.t_born) / 1.0;
  return c_.w_center * clamp01(centre) + c_.w_size * clamp01(size) + c_.w_approach * clamp01(approach) +
         c_.w_conf * clamp01(k.score) + c_.w_age * clamp01(age);
}

void TrackBoard::exclude(int id) {
  if (id < 0) return;
  excluded_.insert(id);
  if (prefer_ == id) prefer_ = -1;
  if (designated_ == id) designated_ = -1;
}

void TrackBoard::prefer(int id) {
  prefer_ = id;
  excluded_.erase(id);
}

void TrackBoard::clear_operator() {
  excluded_.clear();
  prefer_ = -1;
}

int TrackBoard::designate(int locked, int64_t t) {
  auto eligible = [this](const MotTrack& k) { return k.st == MotState::CONFIRMED && !excluded(k.id); };
  for (auto& k : tracks_) k.priority = priority(k, t);
  if (locked >= 0) {   // Perception follows a track: never designate another one
    if (designated_ != locked) { designated_ = locked; t_designated_ = t; }
    challenger_ = -1;
    return locked;
  }
  if (prefer_ >= 0) {  // operator's choice wins while that object is in view
    const MotTrack* p = find(prefer_);
    if (p && p->st != MotState::LOST) {
      if (designated_ != prefer_) { designated_ = prefer_; t_designated_ = t; }
      challenger_ = -1;
      return designated_;
    }
    if (!p) prefer_ = -1;   // deleted: back to automatic choice
  }
  const MotTrack* best = nullptr;
  for (const auto& k : tracks_)
    if (eligible(k) && (!best || k.priority > best->priority)) best = &k;
  const MotTrack* cur = find(designated_);
  if (!cur || !eligible(*cur)) {
    designated_ = best ? best->id : -1;
    t_designated_ = t;
    challenger_ = -1;
    return designated_;
  }
  if (!best || best->id == cur->id || best->priority < cur->priority * (1.f + c_.switch_margin) + 0.02f) {
    challenger_ = -1;
    return designated_;
  }
  if (challenger_ != best->id) { challenger_ = best->id; challenger_n_ = 0; t_challenge_ = t; }
  if (s_of(t - t_challenge_) >= c_.switch_s && s_of(t - t_designated_) >= c_.hold_s) {
    designated_ = best->id;
    t_designated_ = t;
    challenger_ = -1;
  }
  return designated_;
}

}  // namespace rcws
