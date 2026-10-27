// End-to-end Flow 1 on the synthetic scene: acquisition, tracking accuracy,
// occlusion recovery, decoy rejection, and deterministic record/replay.
#include <chrono>
#include <cstdio>
#include <vector>

#include "estimation/camera_model.hpp"
#include "perception/perception.hpp"
#include "record/binlog.hpp"
#include "record/clip.hpp"
#include "sim/synthetic.hpp"
#include "test_util.hpp"

using namespace rcws;

namespace {

struct RunResult {
  double t_first_track = -1;
  double rms_mrad = 0, max_mrad = 0;
  int track_frames = 0, visible_frames = 0;
  double t_recover = -1;  // first TRACK after occlusion end
  std::vector<uint8_t> states;
  bool locked_decoy = false;
  double proc_avg_ms = 0, proc_max_ms = 0;
};

RunResult run(const Config& cfg, const SynthTraj& tr, int frames, const std::string& log_path = "",
              const std::string& clip_path = "", OracleCfg oc = OracleCfg{}) {
  SyntheticSource src(cfg.cam, tr, frames);
  OracleDetector det(src, {640, 640}, oc);
  det.max_in_flight = cfg.det.max_in_flight;
  CameraModel cam(cfg.intr);
  BinLog log;
  if (!log_path.empty()) log.open(log_path, cfg.dump());
  Perception per(cfg, cam, &det, log.is_open() ? &log : nullptr, [](int64_t) { return GimbalPose{}; });
  ClipRecorder clip;
  RunResult res;
  double se = 0;
  int n = 0;
  Frame fr;
  std::string err;
  src.start(&err);
  int k = 0;
  while (src.grab(&fr, 0)) {
    if (k == 0 && !clip_path.empty()) clip.init(fr.view, frames / double(cfg.cam.fps) + 1, cfg.cam.fps);
    const auto tp = std::chrono::steady_clock::now();
    const FrameControls c = per.on_frame(fr.view);
    const double pm = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tp).count();
    res.proc_avg_ms += pm / frames;
    res.proc_max_ms = std::max(res.proc_max_ms, pm);
    if (clip.active()) clip.push(fr.view);
    src.release(fr, c);
    const double t = src.t_of(fr.view.seq);
    res.states.push_back(uint8_t(per.state()));
    const auto truth = src.truth(fr.view.seq);
    if (truth && truth->visible) res.visible_frames++;
    if (per.state() == TrackState::TRACK) {
      if (res.t_first_track < 0) res.t_first_track = t;
      if (tr.occl_t1 > 0 && t >= tr.occl_t1 && res.t_recover < 0) res.t_recover = t - tr.occl_t1;
      if (truth && truth->visible) {
        res.track_frames++;
        const EstState e = per.estimate(truth->t_ns);
        const Los g = cam.pixel_to_los(truth->box.cx(), truth->box.cy(), 0, 0, 0);
        const double err_mrad = 1e3 * std::hypot(e.psi - g.psi, e.theta - g.theta);
        if (err_mrad > 50) res.locked_decoy = true;
        if (t > res.t_first_track + 0.5) {  // after IMM convergence
          se += err_mrad * err_mrad;
          res.max_mrad = std::max(res.max_mrad, err_mrad);
          ++n;
        }
      }
    }
    ++k;
  }
  res.rms_mrad = n ? std::sqrt(se / n) : 1e9;
  log.close();
  if (!clip_path.empty()) clip.save(clip_path);
  return res;
}

SynthTraj sine_traj(const CameraCfg& c) {
  SynthTraj tr;
  const double cx = c.main_w / 2.0, cy = c.main_h / 2.0;
  tr.center = [cx, cy](double t) { return cv::Point2d(cx + 300 * std::sin(0.8 * t), cy + 120 * std::sin(1.3 * t)); };
  return tr;
}

}  // namespace

TEST(pipeline_acquires_and_tracks_moving_target) {
  Config cfg;
  const RunResult r = run(cfg, sine_traj(cfg.cam), 4 * 120);
  std::printf("  first TRACK %.3f s, tracked %d/%d visible frames, RMS %.3f mrad, max %.3f mrad, on_frame avg %.2f max %.2f ms\n",
              r.t_first_track, r.track_frames, r.visible_frames, r.rms_mrad, r.max_mrad, r.proc_avg_ms, r.proc_max_ms);
  CHECK(r.t_first_track >= 0 && r.t_first_track < 0.5);
  CHECK(r.track_frames > 0.85 * r.visible_frames);
  CHECK(r.rms_mrad < 0.5);  // sine target up to ~14 deg/s (0.5 mrad = KPI basic observation goal)
  CHECK(!r.locked_decoy);
}

TEST(pipeline_recovers_after_occlusion) {
  Config cfg;
  SynthTraj tr = sine_traj(cfg.cam);
  tr.occl_t0 = 2.0;
  tr.occl_t1 = 2.35;
  const RunResult r = run(cfg, tr, 4 * 120);
  std::printf("  recovered %.3f s after occlusion end\n", r.t_recover);
  CHECK(r.t_recover >= 0 && r.t_recover < 0.5);
  bool coasted = false;
  for (uint8_t s : r.states) coasted |= s == uint8_t(TrackState::COAST);
  CHECK(coasted);
}

TEST(pipeline_ignores_decoy) {
  Config cfg;
  SynthTraj tr = sine_traj(cfg.cam);
  tr.decoy = true;
  const RunResult r = run(cfg, tr, 3 * 120);
  CHECK(!r.locked_decoy);
}

TEST(record_replay_is_deterministic) {
  Config cfg;
  SynthTraj tr = sine_traj(cfg.cam);
  tr.occl_t0 = 1.5;
  tr.occl_t1 = 1.7;
  const std::string log1 = "/tmp/rcws_test_live.rlog", clip = "/tmp/rcws_test.rclip";
  const RunResult live = run(cfg, tr, 300, log1, clip);

  ReplaySource src(clip);
  std::string err;
  CHECK(src.start(&err));
  ReplayDetector det(log1, {640, 640}, [&] { return src.current_seq(); });
  CHECK(det.ok());
  CameraModel cam(cfg.intr);
  Perception per(cfg, cam, &det, nullptr, [](int64_t) { return GimbalPose{}; });
  std::vector<uint8_t> states;
  Frame fr;
  while (src.grab(&fr, 0)) {
    per.on_frame(fr.view);
    states.push_back(uint8_t(per.state()));
  }
  CHECK(states.size() == live.states.size());
  int diff = 0;
  for (size_t i = 0; i < std::min(states.size(), live.states.size()); ++i) diff += states[i] != live.states[i];
  std::printf("  replayed %zu frames, state mismatches %d\n", states.size(), diff);
  CHECK(diff == 0);
}

// Field regression (2026-10-28 run): phone held ~0.35-1 m from the camera -> 280x420 px box,
// almost uniform back. A 96 px template cut from its middle had no unique ZNCC peak.
TEST(pipeline_tracks_large_plain_phone) {
  Config cfg;
  SynthTraj tr;
  const double cx = cfg.cam.main_w / 2.0, cy = cfg.cam.main_h / 2.0;
  tr.center = [cx, cy](double t) { return cv::Point2d(cx + 150 * std::sin(1.2 * t), cy + 60 * std::sin(1.7 * t)); };
  tr.tgt_w = 280;
  tr.tgt_h = 420;
  tr.plain = true;
  const RunResult r = run(cfg, tr, 3 * 120);
  std::printf("  large plain phone: first TRACK %.3f s, tracked %d/%d, RMS %.3f mrad, on_frame avg %.2f max %.2f ms\n",
              r.t_first_track, r.track_frames, r.visible_frames, r.rms_mrad, r.proc_avg_ms, r.proc_max_ms);
  CHECK(r.t_first_track >= 0 && r.t_first_track < 0.5);
  CHECK(r.track_frames > 0.85 * r.visible_frames);
  CHECK(r.rms_mrad < 2.0);       // 420 px target matched at 1/8 scale
  CHECK(r.proc_avg_ms < 4.0);     // PC budget; Pi 5 ~2-3x
}

// Field regression (2026-10-28 run 2): phone 0.4-1 m away (~210x340 px), jerky hand motion
// (median ~34 deg/s, ~400 deg/s^2). With the first IMM tuning half of the confident detections
// fell outside the gate and the track kept collapsing into COAST.
TEST(pipeline_tracks_handheld_jerky_motion) {
  Config cfg;
  SynthTraj tr;
  const double cx = cfg.cam.main_w / 2.0, cy = cfg.cam.main_h / 2.0;
  tr.center = [cx, cy](double t) {
    return cv::Point2d(cx + 120 * std::sin(2 * M_PI * 1.3 * t) + 40 * std::sin(2 * M_PI * 3.1 * t + 1.0),
                       cy + 80 * std::sin(2 * M_PI * 0.9 * t) + 30 * std::sin(2 * M_PI * 2.7 * t + 0.4));
  };
  tr.tgt_w = 210;
  tr.tgt_h = 340;
  tr.plain = true;
  OracleCfg oc;
  oc.latency_frames = 1;
  oc.box_sigma_rel = 0.015;   // ~5 px edge jitter at this size, like the field log
  const RunResult r = run(cfg, tr, 4 * 120, "", "", oc);
  int coast = 0;
  for (uint8_t s : r.states) coast += s == uint8_t(TrackState::COAST);
  std::printf("  hand-held: first TRACK %.3f s, tracked %d/%d, COAST frames %d, RMS %.3f mrad, on_frame avg %.2f max %.2f ms\n",
              r.t_first_track, r.track_frames, r.visible_frames, coast, r.rms_mrad, r.proc_avg_ms, r.proc_max_ms);
  CHECK(r.track_frames > 0.9 * r.visible_frames);
  CHECK(coast < 0.05 * r.visible_frames);
  CHECK(r.rms_mrad < 5.0);
  CHECK(r.proc_avg_ms < 3.0);
}
