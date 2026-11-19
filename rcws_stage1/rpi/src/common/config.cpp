#include "common/config.hpp"

#include <fstream>
#include <functional>
#include <sstream>
#include <variant>

namespace rcws {
namespace {

using Ref = std::variant<int*, float*, double*, bool*, std::string*>;

std::map<std::string, Ref> registry(Config& c) {
  std::map<std::string, Ref> m;
  auto& a = c.cam;
  m["camera.main_w"] = &a.main_w; m["camera.main_h"] = &a.main_h;
  m["camera.lores_w"] = &a.lores_w; m["camera.lores_h"] = &a.lores_h;
  m["camera.fps"] = &a.fps;
  m["camera.exposure_search_us"] = &a.exposure_search_us;
  m["camera.exposure_track_us"] = &a.exposure_track_us;
  m["camera.gain_init"] = &a.gain_init; m["camera.gain_min"] = &a.gain_min; m["camera.gain_max"] = &a.gain_max;
  m["camera.roi_gain_control"] = &a.roi_gain_control; m["camera.gain_kappa"] = &a.gain_kappa;
  m["camera.roi_target_luma"] = &a.roi_target_luma;
  m["camera.lens_position"] = &a.lens_position;
  m["camera.force_sensor_mode"] = &a.force_sensor_mode; m["camera.sensor_bit_depth"] = &a.sensor_bit_depth;
  m["camera.sensor_w"] = &a.sensor_w; m["camera.sensor_h"] = &a.sensor_h;
  m["camera.buffer_count"] = &a.buffer_count;
  m["camera.timestamp_sign"] = &a.timestamp_sign; m["camera.line_time_ns"] = &a.line_time_ns;

  auto& k = c.intr;
  m["intrinsics.fx"] = &k.fx; m["intrinsics.fy"] = &k.fy; m["intrinsics.cx"] = &k.cx; m["intrinsics.cy"] = &k.cy;
  const char* dn[5] = {"k1", "k2", "p1", "p2", "k3"};
  for (int i = 0; i < 5; ++i) m[std::string("intrinsics.") + dn[i]] = &k.dist[i];
  m["intrinsics.rcb_roll"] = &k.rcb_rpy[0]; m["intrinsics.rcb_pitch"] = &k.rcb_rpy[1]; m["intrinsics.rcb_yaw"] = &k.rcb_rpy[2];
  m["intrinsics.offset_x"] = &k.offset_m[0]; m["intrinsics.offset_y"] = &k.offset_m[1]; m["intrinsics.offset_z"] = &k.offset_m[2];

  auto& d = c.det;
  m["detector.hef_path"] = &d.hef_path; m["detector.class_phone"] = &d.class_phone; m["detector.class_person"] = &d.class_person;
  m["detector.score_min"] = &d.score_min; m["detector.thr_acquire"] = &d.thr_acquire; m["detector.thr_track"] = &d.thr_track;
  m["detector.max_in_flight"] = &d.max_in_flight; m["detector.preproc_i420"] = &d.preproc_i420;
  m["detector.linear_mild_downscale"] = &d.linear_mild_downscale;
  m["detector.lores_large_roi"] = &d.lores_large_roi;

  auto& r = c.roi;
  m["roi.k_s"] = &r.k_s; m["roi.s_min"] = &r.s_min; m["roi.s_max"] = &r.s_max;
  m["roi.tile_w"] = &r.tile_w; m["roi.tile_h"] = &r.tile_h; m["roi.tiles_per_frame"] = &r.tiles_per_frame;
  m["roi.track_det_period_hi"] = &r.track_det_period_hi; m["roi.track_det_period_lo"] = &r.track_det_period_lo;

  auto& n = c.ncc;
  m["ncc.tmpl_min"] = &n.tmpl_min; m["ncc.tmpl_max"] = &n.tmpl_max; m["ncc.tmpl_scale"] = &n.tmpl_scale;
  m["ncc.search_min"] = &n.search_min; m["ncc.search_max"] = &n.search_max;
  m["ncc.rho_min"] = &n.rho_min; m["ncc.peak_margin"] = &n.peak_margin; m["ncc.update_alpha"] = &n.update_alpha;
  m["ncc.update_iou"] = &n.update_iou; m["ncc.update_score"] = &n.update_score; m["ncc.drift_frac"] = &n.drift_frac;
  m["ncc.ncc_only_max_s"] = &n.ncc_only_max_s; m["ncc.sigma_n_px"] = &n.sigma_n_px; m["ncc.sigma_floor_px"] = &n.sigma_floor_px;
  m["ncc.sigma_growth_px_per_s"] = &n.sigma_growth_px_per_s;

  auto& t = c.trk;
  m["track.coast_s"] = &t.coast_s; m["track.reacq_s"] = &t.reacq_s;
  m["track.sprt_pd"] = &t.sprt_pd; m["track.sprt_pfa"] = &t.sprt_pfa; m["track.sprt_alpha"] = &t.sprt_alpha;
  m["track.sprt_beta"] = &t.sprt_beta; m["track.sprt_forget"] = &t.sprt_forget;
  m["track.gate_chi2"] = &t.gate_chi2; m["track.phone_height_m"] = &t.phone_height_m;
  m["track.strong_reject_reinit"] = &t.strong_reject_reinit;
  m["track.size_jump"] = &t.size_jump;

  auto& o = c.mot;
  m["mot.enabled"] = &o.enabled; m["mot.n_init"] = &o.n_init; m["mot.tentative_max_miss"] = &o.tentative_max_miss;
  m["mot.occl_s"] = &o.occl_s; m["mot.lost_keep_s"] = &o.lost_keep_s; m["mot.unseen_s"] = &o.unseen_s;
  m["mot.q_px"] = &o.q_px; m["mot.r_px0"] = &o.r_px0; m["mot.r_rel"] = &o.r_rel; m["mot.gate_chi2"] = &o.gate_chi2;
  m["mot.w_iou"] = &o.w_iou; m["mot.w_dist"] = &o.w_dist; m["mot.w_app"] = &o.w_app; m["mot.max_cost"] = &o.max_cost;
  m["mot.size_jump"] = &o.size_jump; m["mot.overlap_iou"] = &o.overlap_iou;
  m["mot.amb_hist_margin"] = &o.amb_hist_margin; m["mot.amb_hold_s"] = &o.amb_hold_s; m["mot.edge_margin"] = &o.edge_margin;
  m["mot.reid_hist"] = &o.reid_hist; m["mot.reid_size"] = &o.reid_size; m["mot.reid_px_per_s"] = &o.reid_px_per_s;
  m["mot.w_center"] = &o.w_center; m["mot.w_size"] = &o.w_size; m["mot.w_approach"] = &o.w_approach;
  m["mot.w_conf"] = &o.w_conf; m["mot.w_age"] = &o.w_age; m["mot.size_ref_px"] = &o.size_ref_px;
  m["mot.approach_ref"] = &o.approach_ref; m["mot.switch_margin"] = &o.switch_margin; m["mot.switch_s"] = &o.switch_s;
  m["mot.hold_s"] = &o.hold_s; m["mot.scan_period"] = &o.scan_period;

  auto& q = c.qual;
  m["quality.enabled"] = &q.enabled; m["quality.period"] = &q.period; m["quality.dark_luma"] = &q.dark_luma;
  m["quality.sat_frac"] = &q.sat_frac; m["quality.blur_ratio"] = &q.blur_ratio;
  m["quality.bad_n"] = &q.bad_n; m["quality.good_n"] = &q.good_n;

  m["aim.coast_follow_s"] = &c.aim.coast_follow_s;

  auto& e = c.est;
  m["estimator.q_cv"] = &e.q_cv; m["estimator.q_ca"] = &e.q_ca;
  for (int i = 0; i < 4; ++i) m["estimator.pi" + std::to_string(i)] = &e.pi[i];
  m["estimator.sigma_det_px0"] = &e.sigma_det_px0; m["estimator.sigma_det_c"] = &e.sigma_det_c;
  m["estimator.lead_s"] = &e.lead_s;

  auto& u = c.rt;
  m["runtime.vision_cpu"] = &u.vision_cpu; m["runtime.vision_prio"] = &u.vision_prio; m["runtime.logger_cpu"] = &u.logger_cpu;
  m["runtime.log_path"] = &u.log_path; m["runtime.clip_seconds"] = &u.clip_seconds; m["runtime.clip_path"] = &u.clip_path;
  m["runtime.roi_temporal_avg"] = &u.roi_temporal_avg;
  return m;
}

std::string trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t\r");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r");
  return s.substr(b, e - b + 1);
}

bool assign(const Ref& ref, const std::string& v) {
  try {
    if (auto p = std::get_if<int*>(&ref)) **p = std::stoi(v);
    else if (auto p = std::get_if<float*>(&ref)) **p = std::stof(v);
    else if (auto p = std::get_if<double*>(&ref)) **p = std::stod(v);
    else if (auto p = std::get_if<bool*>(&ref)) **p = (v == "1" || v == "true" || v == "on");
    else if (auto p = std::get_if<std::string*>(&ref)) **p = v;
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

bool Config::load_ini(const std::string& path, std::string* err) {
  std::ifstream f(path);
  if (!f) { if (err) *err = "cannot open " + path; return false; }
  auto reg = registry(*this);
  std::string line, section;
  int ln = 0;
  std::ostringstream problems;
  while (std::getline(f, line)) {
    ++ln;
    line = trim(line.substr(0, line.find_first_of("#;")));
    if (line.empty()) continue;
    if (line.front() == '[' && line.back() == ']') { section = trim(line.substr(1, line.size() - 2)); continue; }
    const auto eq = line.find('=');
    if (eq == std::string::npos) { problems << path << ":" << ln << ": no '='\n"; continue; }
    const std::string key = section + "." + trim(line.substr(0, eq));
    const std::string val = trim(line.substr(eq + 1));
    auto it = reg.find(key);
    if (it == reg.end()) { problems << path << ":" << ln << ": unknown key " << key << "\n"; continue; }
    if (!assign(it->second, val)) problems << path << ":" << ln << ": bad value for " << key << "\n";
  }
  if (err) *err = problems.str();
  return problems.str().empty();
}

std::string Config::dump() const {
  auto reg = registry(const_cast<Config&>(*this));
  std::ostringstream o;
  o.precision(9);
  for (auto& [k, ref] : reg) {
    o << k << "=";
    std::visit([&](auto p) { o << *p; }, ref);
    o << "\n";
  }
  return o.str();
}

}  // namespace rcws
