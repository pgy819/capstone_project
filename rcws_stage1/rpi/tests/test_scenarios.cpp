// v4 scenario tests (A-J of the scenario sheet) on the synthetic scene: multi-target identity,
// crossing, merge -> AMBIGUOUS, pillar occlusion, long occlusion re-identification, leaving the
// view without switching target, low light, clutter. Plus unit tests of the track board.
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <vector>

#include "estimation/camera_model.hpp"
#include "perception/mot.hpp"
#include "perception/perception.hpp"
#include "perception/quality.hpp"
#include "sim/synthetic.hpp"
#include "test_util.hpp"

using namespace rcws;

namespace {

struct ScenarioRun {
  int frames = 0;
  std::vector<int> id_switches;          // per truth object, board identity changes
  std::vector<int> matched;              // per truth object, frames it had a board track
  int lock_obj_switches = 0;             // Perception's target jumped between objects
  std::vector<int> lock_obj;             // per frame: object followed (-1 none)
  std::vector<uint8_t> state;            // Perception state per frame
  std::vector<AimOut> aim;
  std::set<int> lock_ids;                // board ids Perception followed
  uint32_t lock_track_states = 0;        // bitmask of MotState seen on the lock track
  uint64_t ambiguous_events = 0;
  ScenarioStats ss;
  std::vector<uint8_t> qflags;
  double t_first_track = -1;
  struct LockTrace {
    int id = -1;
    int st = -1;          // MotState of the lock track, -1 = not on the board
    bool exited = false;
    float exit_dx = 0;
  };
  std::vector<LockTrace> lock_trace;     // per frame (diagnostics on failure)
};

ScenarioRun run_scenario(const std::string& name, Config cfg = Config{}, uint32_t seed = 1) {
  int frames = 0;
  bool ok = false;
  SynthTraj tr = make_scenario(name, cfg.cam, &frames, &ok);
  CHECK(ok);
  SyntheticSource src(cfg.cam, tr, frames, seed);
  OracleDetector det(src, {640, 640}, OracleCfg{}, seed + 1);
  det.max_in_flight = cfg.det.max_in_flight;
  CameraModel cam(cfg.intr);
  Perception per(cfg, cam, &det, nullptr, [](int64_t) { return GimbalPose{}; });
  ScenarioRun r;
  const size_t n_obj = 1 + tr.extra.size();
  r.id_switches.assign(n_obj, 0);
  r.matched.assign(n_obj, 0);
  std::vector<int> last_id(n_obj, -1);
  int last_lock_obj = -1;
  Frame fr;
  std::string err;
  src.start(&err);
  while (src.grab(&fr, 0)) {
    const FrameControls c = per.on_frame(fr.view);
    src.release(fr, c);
    r.frames++;
    const auto truths = src.truth_all(fr.view.seq);
    // board identity per visible object
    for (size_t o = 0; o < truths.size(); ++o) {
      if (!truths[o].visible) continue;
      const RectF& g = truths[o].box;
      // two phones on top of each other: which track is which cannot be scored from geometry
      bool crowded = false;
      for (size_t p = 0; p < truths.size(); ++p)
        if (p != o && truths[p].visible &&
            std::hypot(truths[p].box.cx() - g.cx(), truths[p].box.cy() - g.cy()) < std::max(g.w, g.h))
          crowded = true;
      if (crowded) continue;
      int id = -1;
      double best = std::max(20.f, 0.6f * g.w);
      for (const auto& k : per.board().tracks()) {
        if (k.st == MotState::LOST || k.st == MotState::TENTATIVE) continue;
        const double d = std::hypot(k.x[0] - g.cx(), k.x[1] - g.cy());
        if (d < best) { best = d; id = k.id; }
      }
      if (id < 0) continue;
      r.matched[o]++;
      if (last_id[o] >= 0 && id != last_id[o]) r.id_switches[o]++;
      last_id[o] = id;
    }
    // object Perception follows
    int lo = -1;
    if (per.state() == TrackState::TRACK) {
      if (r.t_first_track < 0) r.t_first_track = src.t_of(fr.view.seq);
      if (auto b = per.target_box()) {
        // nearest visible phone, counted only when clearly nearer than any other (while two phones
        // overlap on screen the followed one cannot be told from geometry alone)
        double best = 40, second = 1e9;
        for (size_t o = 0; o < truths.size(); ++o) {
          if (!truths[o].visible) continue;
          const double d = std::hypot(b->cx() - truths[o].box.cx(), b->cy() - truths[o].box.cy());
          if (d < best) { second = std::min(second, best); best = d; lo = int(o); }
          else second = std::min(second, d);
        }
        if (lo >= 0 && second < 2.0 * best + 20) lo = -1;
      }
    }
    if (lo >= 0) {
      if (last_lock_obj >= 0 && lo != last_lock_obj) r.lock_obj_switches++;
      last_lock_obj = lo;
    }
    r.lock_obj.push_back(lo);
    r.state.push_back(uint8_t(per.state()));
    r.aim.push_back(per.aim());
    r.qflags.push_back(per.quality_flags());
    ScenarioRun::LockTrace lt;
    lt.id = per.lock_id();
    if (per.lock_id() >= 0) {
      r.lock_ids.insert(per.lock_id());
      if (const MotTrack* k = per.board().find(per.lock_id())) {
        r.lock_track_states |= 1u << unsigned(k->st);
        lt.st = int(k->st);
        lt.exited = k->exited;
        lt.exit_dx = k->exit_dx;
      }
    }
    r.lock_trace.push_back(lt);
  }
  r.ambiguous_events = per.board().id_switch_guards();
  r.ss = per.scenario_stats();
  return r;
}

int frame_of(double t, const Config& cfg) { return int(t * cfg.cam.fps); }

void print_run(const char* name, const ScenarioRun& r) {
  std::printf("  %-8s frames %d  id switches", name, r.frames);
  for (size_t o = 0; o < r.id_switches.size(); ++o) std::printf(" [%zu]%d/%d", o, r.id_switches[o], r.matched[o]);
  std::printf("  lock obj switches %d  lock ids %zu  ambiguous %llu  other-skipped %llu size-jump %llu reanchor-blocked %llu\n",
              r.lock_obj_switches, r.lock_ids.size(), (unsigned long long)r.ambiguous_events,
              (unsigned long long)r.ss.other_target_skipped, (unsigned long long)r.ss.size_jump_rejected,
              (unsigned long long)r.ss.reanchor_blocked);
}

}  // namespace

// ---------------------------------------------------------------- board unit tests
TEST(hungarian_small_matrix) {
  // rows: tracks, cols: detections
  const std::vector<double> C = {4, 1, 3,
                                 2, 0, 5,
                                 3, 2, 2};
  const auto a = hungarian(C, 3, 3, 100);
  CHECK(a[0] == 1 && a[1] == 0 && a[2] == 2);   // 1 + 2 + 2 = 5 is the minimum
  const auto b = hungarian({0.5, 9, 9, 0.7}, 2, 2, 1.0);
  CHECK(b[0] == 0 && b[1] == 1);
  const auto c = hungarian({5, 5}, 1, 2, 1.0);   // every pair above max_cost -> unassigned
  CHECK(c[0] == -1);
}

TEST(board_priority_hysteresis) {
  // Two confirmed tracks; their priorities cross back and forth slowly. The designated target may
  // change at most once (hold + switch margin), never flap.
  MotCfg mc;
  TrackBoard tb(mc, 1536, 864);
  int switches = 0, last = -1;
  for (int k = 0; k < 600; ++k) {   // 5 s at 120 Hz
    const double t = k / 120.0;
    DetectionBatch b;
    b.frame_seq = uint32_t(k);
    b.t_sof_ns = 1000000000LL + int64_t(t * 1e9);
    b.crop = {0, 0, 1536, 864};
    // A slides from the centre outward, B the other way, with a small wobble around the crossing
    const double a = 0.5 + 0.45 * std::tanh((t - 2.5) * 0.8) + 0.03 * std::sin(9 * t);
    Detection da, db;
    da.box = {float(768 + 500 * a - 18), 300 - 37, 36, 74}; da.score = 0.8f; da.cls = ObjClass::Phone;
    db.box = {float(768 + 500 * (1 - a) - 18), 560 - 37, 36, 74}; db.score = 0.8f; db.cls = ObjClass::Phone;
    b.dets = {da, db};
    tb.update(b, nullptr, 0.3f, 0.15f);
    tb.tick(b.t_sof_ns);
    const int d = tb.designate(-1, b.t_sof_ns);
    if (d >= 0) { if (last >= 0 && d != last) ++switches; last = d; }
  }
  std::printf("  designated switches %d\n", switches);
  CHECK(last >= 0);
  CHECK(switches <= 1);
}

TEST(board_lock_is_never_redesignated) {
  MotCfg mc;
  TrackBoard tb(mc, 1536, 864);
  DetectionBatch b;
  b.crop = {0, 0, 1536, 864};
  Detection d1, d2;
  d1.box = {100, 100, 36, 74}; d1.score = 0.9f; d1.cls = ObjClass::Phone;
  d2.box = {750, 400, 36, 74}; d2.score = 0.9f; d2.cls = ObjClass::Phone;   // more central
  for (int k = 0; k < 10; ++k) {
    b.t_sof_ns = 1000000000LL + k * 8333333LL;
    b.dets = {d1, d2};
    tb.update(b, nullptr, 0.3f, 0.15f);
  }
  const int lock = tb.tracks()[0].id;   // the off-centre one
  CHECK(tb.designate(lock, b.t_sof_ns) == lock);
  // a board that was never locked chooses freely: the central, otherwise identical phone
  TrackBoard fresh(mc, 1536, 864);
  for (int k = 0; k < 10; ++k) {
    b.t_sof_ns = 1000000000LL + k * 8333333LL;
    b.dets = {d1, d2};
    fresh.update(b, nullptr, 0.3f, 0.15f);
  }
  CHECK(fresh.designate(-1, b.t_sof_ns) == fresh.tracks()[1].id);
}

TEST(board_exit_without_edge_crop) {
  // Pi 5 run of the "exit" scenario: while the phone left through the right edge, no detector crop
  // covered it, so it collected no miss, stayed CONFIRMED and timed out as a plain LOST (no exit
  // side -> no search hint). Two crop patterns: crops that never reach the edge, and none at all.
  for (int pattern = 0; pattern < 2; ++pattern) {
    MotCfg mc;
    TrackBoard tb(mc, 1536, 864);
    DetectionBatch b;
    b.crop = {0, 0, 1536, 864};
    int64_t t = 1000000000LL;
    const double v = 900;   // px/s to the right
    double x = 1200;
    int id = -1;
    for (int k = 0; k < 120; ++k, t += 8333333LL) {   // seen while x < 1450
      x += v / 120;
      b.t_sof_ns = t;
      Detection d;
      d.box = {float(x - 18), 400, 36, 74}; d.score = 0.9f; d.cls = ObjClass::Phone;
      if (x < 1450) b.dets = {d};
      else { b.dets.clear(); b.crop = pattern == 0 ? RectF{0, 0, 900, 864} : RectF{300, 200, 400, 400}; }
      tb.update(b, nullptr, 0.3f, 0.15f);
      tb.tick(t);
      if (id < 0 && !tb.tracks().empty()) id = tb.tracks()[0].id;
    }
    const MotTrack* k = tb.find(id);
    CHECK(k != nullptr);
    if (!k) continue;
    std::printf("  pattern %d: %s exited %d exit_dx %.2f\n", pattern, to_string(k->st), int(k->exited), k->exit_dx);
    CHECK(k->st == MotState::LOST && k->exited && k->exit_dx > 0.5f);
  }
}

TEST(quality_flags_dark_with_hysteresis) {
  QualityCfg qc;
  QualityMonitor q(qc);
  cv::Mat lo(360, 640, CV_8UC1, cv::Scalar(120));
  FrameView f;
  f.lo_y = lo.data; f.lo_w = 640; f.lo_h = 360; f.lo_stride = 640;
  int first_dark = -1, first_clear = -1;
  for (int k = 0; k < 200; ++k) {
    lo.setTo(k >= 50 && k < 120 ? 10 : 120);
    q.evaluate(f, nullptr, false);
    if (first_dark < 0 && (q.flags() & Q_DARK)) first_dark = k;
    if (first_dark >= 0 && first_clear < 0 && k >= 120 && !(q.flags() & Q_DARK)) first_clear = k;
  }
  std::printf("  dark from frame 50 -> flagged %d, light from 120 -> cleared %d\n", first_dark, first_clear);
  CHECK(first_dark >= 50 && first_dark <= 50 + 3 * qc.period + qc.period);
  CHECK(first_clear >= 120 && first_clear <= 120 + 6 * qc.period + qc.period);
}

TEST(aim_gate_rules) {
  AimCfg ac;
  AimInputs in;
  in.est.valid = true;
  in.est.psi = 0.1;
  in.est.psi_d = 0.5;
  in.state = TrackState::TRACK;
  CHECK(decide_aim(ac, in).mode == AimMode::FOLLOW);
  in.quality_flags = Q_DARK;
  AimOut o = decide_aim(ac, in);
  CHECK(o.mode == AimMode::HOLD && (o.reasons & A_QUALITY) && o.psi_d == 0);
  in.quality_flags = 0;
  in.state = TrackState::COAST;
  in.since_meas_s = 0.05;
  CHECK(decide_aim(ac, in).mode == AimMode::PREDICT);
  in.since_meas_s = 0.4;   // never keep turning on the last velocity
  CHECK(decide_aim(ac, in).mode == AimMode::HOLD);
  MotTrack k;
  k.st = MotState::AMBIGUOUS;
  in.state = TrackState::TRACK;
  in.lock = &k;
  o = decide_aim(ac, in);
  CHECK(o.mode == AimMode::HOLD && (o.reasons & A_AMBIGUOUS));
  k.st = MotState::LOST; k.exited = true; k.exit_dx = 1; k.exit_dy = 0;
  in.state = TrackState::REACQ;
  o = decide_aim(ac, in);
  CHECK(o.mode == AimMode::HOLD && o.hint_dx == 1.f);
}

// ---------------------------------------------------------------- end-to-end scenarios
TEST(scenario_parallel_two_ids) {
  const ScenarioRun r = run_scenario("parallel");
  print_run("parallel", r);
  CHECK(r.id_switches[0] == 0 && r.id_switches[1] == 0);
  CHECK(r.matched[0] > r.frames / 2 && r.matched[1] > r.frames / 4);   // the other phone is seen by the scans
  CHECK(r.lock_obj_switches == 0);
  CHECK(r.t_first_track >= 0 && r.t_first_track < 0.6);
}

TEST(scenario_cross_keeps_identity) {
  const ScenarioRun r = run_scenario("cross");
  print_run("cross", r);
  CHECK(r.lock_obj_switches == 0);           // the gimbal target never jumps to the other phone
  CHECK(r.id_switches[0] == 0);
  int followed = -1;
  for (int o : r.lock_obj) if (o >= 0) { followed = o; break; }
  int last = -1;
  for (int o : r.lock_obj) if (o >= 0) last = o;
  CHECK(followed >= 0 && last == followed);  // still on the same phone after the crossing
}

TEST(scenario_merge_flags_ambiguity) {
  const ScenarioRun r = run_scenario("merge");
  print_run("merge", r);
  // same-looking phones overlap completely: identity cannot be guaranteed after the split, so the
  // aim output must have been held for that reason at least once, or the board must have flagged it
  bool held_for_ambiguity = false;
  for (const auto& a : r.aim) held_for_ambiguity |= a.mode == AimMode::HOLD && (a.reasons & A_AMBIGUOUS);
  CHECK(r.ambiguous_events > 0);
  CHECK(held_for_ambiguity);
  CHECK(r.ss.size_jump_rejected + r.ss.other_target_skipped > 0);   // merged boxes / other phone kept out
}

TEST(scenario_pillar_short_occlusion) {
  const ScenarioRun r = run_scenario("pillar");
  print_run("pillar", r);
  CHECK(r.lock_obj_switches == 0);
  CHECK(r.id_switches[0] == 0);
  CHECK(r.lock_track_states & (1u << unsigned(MotState::OCCLUDED)));
  // tracking again after the pillar (target is right of it from ~3.4 s)
  int track_after = 0;
  for (size_t i = size_t(3.6 * 120); i < r.state.size(); ++i) track_after += r.state[i] == uint8_t(TrackState::TRACK);
  CHECK(track_after > 0.8 * (r.state.size() - size_t(3.6 * 120)));
}

TEST(scenario_long_occlusion_reidentified) {
  Config cfg;
  const ScenarioRun r = run_scenario("longocc", cfg);
  print_run("longocc", r);
  CHECK(r.lock_track_states & (1u << unsigned(MotState::LOST)));
  CHECK(r.lock_obj_switches == 0);           // the other phone was never taken
  // back on target soon after it reappears (3.5 s) and with the same identity
  int back = -1;
  for (size_t i = size_t(frame_of(3.5, cfg)); i < r.state.size(); ++i)
    if (r.state[i] == uint8_t(TrackState::TRACK)) { back = int(i); break; }
  std::printf("  back in TRACK %.3f s after reappearing, lock ids used %zu\n", back < 0 ? -1.0 : back / 120.0 - 3.5, r.lock_ids.size());
  CHECK(back >= 0 && back / 120.0 - 3.5 < 0.5);
  CHECK(r.lock_ids.size() == 1);             // re-identified, not a new track
}

TEST(scenario_exit_no_switch_and_return) {
  Config cfg;
  const ScenarioRun r = run_scenario("exit", cfg);
  print_run("exit", r);
  CHECK(r.lock_obj_switches == 0);           // never jumped to the phone left in view
  bool exiting = r.lock_track_states & (1u << unsigned(MotState::EXITING));
  CHECK(exiting);
  // while it is gone the gimbal holds and knows where it went
  bool hint = false, moved_while_gone = false;
  for (size_t i = size_t(frame_of(3.4, cfg)); i < size_t(frame_of(4.7, cfg)); ++i) {
    hint |= r.aim[i].hint_dx > 0.5f;
    moved_while_gone |= r.aim[i].mode == AimMode::FOLLOW;
  }
  CHECK(hint);
  CHECK(!moved_while_gone);
  if (!hint || moved_while_gone) {   // timeline of the lock while it is gone (platform differences)
    std::printf("  timeline 3.0-4.8 s (changes only): t  state  aim  hint_dx  lock  lock_st  exited  exit_dx\n");
    std::string last;
    for (size_t i = size_t(frame_of(3.0, cfg)); i < size_t(frame_of(4.8, cfg)) && i < r.state.size(); ++i) {
      const auto& lt = r.lock_trace[i];
      char b[160];
      std::snprintf(b, sizeof b, "%s %s %.2f %d %s %d %.2f", to_string(TrackState(r.state[i])), to_string(r.aim[i].mode),
                    r.aim[i].hint_dx, lt.id, lt.st < 0 ? "gone" : to_string(MotState(lt.st)), int(lt.exited), lt.exit_dx);
      if (b != last) { std::printf("    %.3f  %s\n", i / double(cfg.cam.fps), b); last = b; }
    }
  }
  int back = -1;
  for (size_t i = size_t(frame_of(4.8, cfg)); i < r.state.size(); ++i)
    if (r.state[i] == uint8_t(TrackState::TRACK) && r.lock_obj[i] == 0) { back = int(i); break; }
  std::printf("  back on the same phone %.3f s after it re-entered\n", back < 0 ? -1.0 : back / 120.0 - 4.8);
  CHECK(back >= 0);
}

TEST(scenario_dark_holds_output) {
  Config cfg;
  const ScenarioRun r = run_scenario("dark", cfg);
  print_run("dark", r);
  int dark_flag = 0, hold_q = 0, follow_in_dark = 0;
  for (size_t i = size_t(frame_of(2.1, cfg)); i < size_t(frame_of(3.0, cfg)); ++i) {
    dark_flag += (r.qflags[i] & Q_DARK) != 0;
    hold_q += r.aim[i].mode == AimMode::HOLD;
    follow_in_dark += r.aim[i].mode == AimMode::FOLLOW;
  }
  const int span = frame_of(3.0, cfg) - frame_of(2.1, cfg);
  std::printf("  dark interval: DARK flag %d/%d frames, HOLD %d, FOLLOW %d\n", dark_flag, span, hold_q, follow_in_dark);
  CHECK(dark_flag > 0.9 * span);
  CHECK(follow_in_dark == 0);
  int track_after = 0;
  for (size_t i = size_t(frame_of(4.0, cfg)); i < r.state.size(); ++i) track_after += r.aim[i].mode == AimMode::FOLLOW;
  CHECK(track_after > 0.8 * (r.state.size() - size_t(frame_of(4.0, cfg))));
}

TEST(scenario_clutter_no_false_lock) {
  const ScenarioRun r = run_scenario("clutter");
  print_run("clutter", r);
  int on_target = 0, tracked = 0;
  for (int o : r.lock_obj) { tracked += o >= 0; on_target += o == 0; }
  CHECK(tracked > 0.8 * r.frames);
  CHECK(on_target == tracked);
}

// Robustness sweep over random seeds (scene clutter, detector noise, misses, false alarms).
// Off by default (slow): RCWS_SWEEP=10 ./rcws_tests
TEST(scenario_seed_sweep) {
  const char* e = std::getenv("RCWS_SWEEP");
  if (!e) return;
  const int n = std::atoi(e);
  const char* only = std::getenv("RCWS_SWEEP_ONLY");   // e.g. RCWS_SWEEP_ONLY=cross: per-seed detail
  for (const char* name : {"parallel", "cross", "merge", "pillar", "longocc", "exit", "dark", "clutter"}) {
    if (only && std::string(only) != name) continue;
    int lock_sw = 0, id_sw0 = 0, worst = 0;
    for (int k = 0; k < n; ++k) {
      const ScenarioRun r = run_scenario(name, Config{}, uint32_t(11 + 7 * k));
      if (only) { std::printf("  seed %u: ", 11 + 7 * k); print_run(name, r); }
      lock_sw += r.lock_obj_switches > 0;
      id_sw0 += r.id_switches[0] > 0;
      worst = std::max(worst, r.lock_obj_switches);
    }
    std::printf("  %-8s seeds %d: runs with a target jump %d, with an ID switch on the target %d (worst jumps %d)\n",
                name, n, lock_sw, id_sw0, worst);
  }
}
