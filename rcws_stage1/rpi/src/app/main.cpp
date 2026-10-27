// rcws_vision - Stage 1 (Step 1) runtime: object recognition + tracking + logging on Pi 5.
// The gimbal is fixed in Step 1; estimates are absolute LOS angles in the camera mount frame.
//
//   rcws_vision --config config/rcws.ini --source camera --detector hailo --log run.rlog
//   rcws_vision --source synthetic --detector oracle --frames 1200          (PC, no hardware)
//   rcws_vision --source replay:run.rclip --detector replay:run.rlog --log replay.rlog
#include <atomic>
#include <unistd.h>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include "camera/frame_source.hpp"
#include "common/clock.hpp"
#include "common/config.hpp"
#include "common/rt.hpp"
#include "detect/detector.hpp"
#include "estimation/camera_model.hpp"
#include "perception/perception.hpp"
#include "record/binlog.hpp"
#include "record/clip.hpp"
#include "sim/synthetic.hpp"
#include "app/preview_server.hpp"
#include <opencv2/imgproc.hpp>
#ifdef RCWS_HAVE_HAILO
#include "detect/hailo_detector.hpp"
#endif

using namespace rcws;

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) {
  if (g_stop.exchange(true)) _exit(130);  // second Ctrl-C: leave immediately
}

struct Args {
  std::string config, source = "camera", detector = "hailo", log;
  int frames = 0;          // 0 = until Ctrl-C / end of stream
  int print_every = 120;
  double clip_seconds = -1;
  int preview_port = 0;    // 0 = off
  bool window = false;     // on-screen popup on the Pi desktop
  int preview_fps = 10;
};

void usage() {
  std::cerr << "usage: rcws_vision [--config f.ini] [--source camera|synthetic|replay:CLIP]\n"
               "                   [--detector hailo|oracle|none|replay:RLOG] [--log out.rlog]\n"
               "                   [--frames N] [--clip-seconds S] [--print-every N]\n"
               "                   [--preview PORT] [--preview-fps N]   (live view: http://<pi>:PORT/)\n"
               "                   [--window]                           (popup window on the Pi desktop)\n";
}

bool parse(int argc, char** argv, Args* a) {
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    auto val = [&](std::string* out) { if (i + 1 >= argc) return false; *out = argv[++i]; return true; };
    std::string v;
    if (k == "--config" && val(&a->config)) continue;
    if (k == "--source" && val(&a->source)) continue;
    if (k == "--detector" && val(&a->detector)) continue;
    if (k == "--log" && val(&a->log)) continue;
    if (k == "--frames" && val(&v)) { a->frames = std::stoi(v); continue; }
    if (k == "--print-every" && val(&v)) { a->print_every = std::stoi(v); continue; }
    if (k == "--clip-seconds" && val(&v)) { a->clip_seconds = std::stod(v); continue; }
    if (k == "--preview" && val(&v)) { a->preview_port = std::stoi(v); continue; }
    if (k == "--preview-fps" && val(&v)) { a->preview_fps = std::stoi(v); continue; }
    if (k == "--window") { a->window = true; continue; }
    return false;
  }
  return true;
}

double deg(double r) { return r * 180.0 / M_PI; }

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse(argc, argv, &args)) { usage(); return 2; }
  Config cfg;
  if (!args.config.empty()) {
    std::string err;
    if (!cfg.load_ini(args.config, &err)) { std::cerr << err; if (err.find("cannot open") != std::string::npos) return 2; }
  }
  if (!args.log.empty()) cfg.rt.log_path = args.log;
  if (args.clip_seconds >= 0) cfg.rt.clip_seconds = args.clip_seconds;
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  // ---- source
  std::unique_ptr<IFrameSource> src;
  SyntheticSource* synth = nullptr;
  ReplaySource* replay = nullptr;
  if (args.source == "camera") {
    src = make_libcamera_source(cfg.cam);
    if (!src) { std::cerr << "built without libcamera\n"; return 1; }
  } else if (args.source == "synthetic") {
    SynthTraj tr;
    const double cx = cfg.cam.main_w / 2.0, cy = cfg.cam.main_h / 2.0;
    tr.center = [cx, cy](double t) { return cv::Point2d(cx + 300 * std::sin(0.8 * t), cy + 120 * std::sin(1.3 * t)); };
    tr.occl_t0 = 6.0; tr.occl_t1 = 6.3;
    auto s = std::make_unique<SyntheticSource>(cfg.cam, tr, args.frames > 0 ? args.frames : 1200);
    synth = s.get();
    src = std::move(s);
  } else if (args.source.rfind("replay:", 0) == 0) {
    auto s = std::make_unique<ReplaySource>(args.source.substr(7));
    replay = s.get();
    src = std::move(s);
  } else { usage(); return 2; }

  std::string err;
  if (!src->start(&err)) { std::cerr << "source: " << err << "\n"; return 1; }

  // ---- detector
  std::unique_ptr<IDetector> det;
  const cv::Size default_in(640, 640);
  if (args.detector == "hailo") {
#ifdef RCWS_HAVE_HAILO
    det = make_hailo_detector(cfg.det, &err);
    if (!det) { std::cerr << "hailo: " << err << "\n"; return 1; }
#else
    std::cerr << "built without HailoRT\n";
    return 1;
#endif
  } else if (args.detector == "oracle") {
    if (!synth) { std::cerr << "oracle detector needs --source synthetic\n"; return 2; }
    auto o = std::make_unique<OracleDetector>(*synth, default_in, OracleCfg{});
    o->max_in_flight = cfg.det.max_in_flight;
    det = std::move(o);
  } else if (args.detector.rfind("replay:", 0) == 0) {
    if (!replay) { std::cerr << "replay detector needs --source replay:CLIP\n"; return 2; }
    auto r = std::make_unique<ReplayDetector>(args.detector.substr(7), default_in, [replay] { return replay->current_seq(); });
    if (!r->ok()) { std::cerr << "cannot read detections from log\n"; return 1; }
    det = std::move(r);
  } else if (args.detector == "none") {
    det = make_null_detector(default_in);
  } else { usage(); return 2; }

  // ---- pipeline
  CameraModel cam(cfg.intr);
  BinLog log;
  if (!cfg.rt.log_path.empty() && !log.open(cfg.rt.log_path, cfg.dump(), cfg.rt.logger_cpu))
    std::cerr << "warning: cannot open log " << cfg.rt.log_path << "\n";
  Perception per(cfg, cam, det.get(), log.is_open() ? &log : nullptr,
                 [](int64_t) { return GimbalPose{0, 0}; });  // Step 1: gimbal fixed
  ClipRecorder clip;
  PreviewServer preview;
  const bool want_view = args.preview_port > 0 || args.window;
  if (want_view) {
    if (args.window && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
      std::cerr << "--window: no desktop session (DISPLAY unset). Over SSH run with: DISPLAY=:0 ./build/rcws_vision ...\n";
      args.window = false;
    }
    if (preview.start(args.preview_port, args.window, cfg.rt.logger_cpu, &err)) {
      if (args.preview_port > 0) std::cerr << "preview: open http://<pi-address>:" << args.preview_port << "/ in a browser\n";
      if (args.window) std::cerr << "preview: popup window 'RCWS Stage 1' on the Pi desktop\n";
    } else {
      std::cerr << "preview disabled: " << err << "\n";
    }
  }
  double last_fps = 0, last_proc = 0;
  int64_t t_prev_frame = 0;
  int64_t t_preview = 0;

  if (args.source == "camera") set_thread_rt(cfg.rt.vision_cpu, cfg.rt.vision_prio, "vision");

  uint64_t n = 0, dropped = 0;
  int64_t t_print = now_ns();
  double proc_max_ms = 0, proc_sum_ms = 0;
  while (!g_stop) {
    Frame fr;
    if (!src->grab(&fr, 200)) {
      if (src->eos()) break;
      continue;
    }
    if (n == 0 && cfg.rt.clip_seconds > 0) {
      if (!clip.init(fr.view, cfg.rt.clip_seconds, cfg.cam.fps)) std::cerr << "clip init failed\n";
    }
    dropped += fr.dropped;
    const int64_t t0 = now_ns();
    const FrameControls c = per.on_frame(fr.view);
    const double ms = (now_ns() - t0) * 1e-6;
    proc_max_ms = std::max(proc_max_ms, ms);
    proc_sum_ms += ms;
    if (clip.active()) clip.push(fr.view);
    {
      const int64_t t = now_ns();
      if (t_prev_frame) last_fps = 0.9 * last_fps + 0.1 * (1e9 / std::max<int64_t>(1, t - t_prev_frame));
      t_prev_frame = t;
      last_proc = 0.9 * last_proc + 0.1 * ms;
      // live preview at a low rate: only a small luma copy happens on this thread
      if (want_view && t - t_preview >= 1000000000LL / std::max(1, args.preview_fps)) {
        t_preview = t;
        cv::Mat gray;
        float scale = 1.f;
        if (fr.view.lo_y) {
          gray = cv::Mat(fr.view.lo_h, fr.view.lo_w, CV_8UC1, const_cast<uint8_t*>(fr.view.lo_y), fr.view.lo_stride);
          scale = float(fr.view.lo_w) / float(fr.view.main_w);
        } else {
          const cv::Mat y(fr.view.main_h, fr.view.main_w, CV_8UC1, const_cast<uint8_t*>(fr.view.main_y), fr.view.main_stride);
          cv::resize(y, gray, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
          scale = 0.5f;
        }
        PreviewOverlay ov;
        ov.state = per.state();
        if (auto b = per.target_box()) { ov.has_target = true; ov.target = *b; }
        const auto& lb = per.last_detections();
        ov.dets = lb.dets;
        ov.det_crop = lb.crop;
        ov.scale = scale;
        ov.fps = last_fps;
        ov.proc_ms = last_proc;
        ov.seq = fr.view.seq;
        const EstState e = per.estimate(fr.view.t_sof_ns);
        if (e.valid) {
          ov.est_valid = true;
          ov.psi_deg = deg(e.psi); ov.theta_deg = deg(e.theta);
          ov.rate_psi = deg(e.psi_d); ov.rate_theta = deg(e.theta_d);
          ov.range_m = e.range_m;
        }
        preview.publish(gray, ov);
      }
    }
    if (synth && log.is_open())
      if (auto tr = synth->truth(fr.view.seq)) {
        RecTruth rt{tr->t_ns, fr.view.seq, tr->box.cx(), tr->box.cy(), tr->box.w, tr->box.h, uint8_t(tr->visible), {0, 0, 0}};
        log.write(RecType::TRUTH, rt);
      }
    src->release(fr, c);
    ++n;

    if (args.print_every > 0 && n % uint64_t(args.print_every) == 0) {
      const int64_t t = now_ns();
      const double fps = args.print_every / ((t - t_print) * 1e-9);
      t_print = t;
      int64_t t_est = fr.view.t_sof_ns;
      if (synth) if (auto tr = synth->truth(fr.view.seq)) t_est = tr->t_ns;
      const EstState e = per.estimate(t_est);
      const auto& st = per.stats();
      std::fprintf(stderr, "#%6llu %-7s fps %6.1f  proc avg %.2f max %.2f ms  drop %llu  det sub %llu done %llu fly %d",
                   (unsigned long long)n, to_string(per.state()), fps, proc_sum_ms / args.print_every, proc_max_ms,
                   (unsigned long long)dropped, (unsigned long long)st.det_submitted,
                   (unsigned long long)st.det_batches, det ? det->in_flight() : 0);
      const auto& lb = per.last_detections();
      float best_phone = 0;
      for (const auto& d : lb.dets) if (d.cls == ObjClass::Phone) best_phone = std::max(best_phone, d.score);
      if (best_phone > 0) std::fprintf(stderr, "  phone %.2f", best_phone);
      if (e.valid)
        std::fprintf(stderr, "  psi %+7.3f th %+7.3f deg  rate %+6.1f %+6.1f deg/s  R %.1f m", deg(e.psi), deg(e.theta),
                     deg(e.psi_d), deg(e.theta_d), e.range_m);
      if (synth) {
        if (auto tr = synth->truth(fr.view.seq); tr && tr->visible) {
          const Los g = cam.pixel_to_los(tr->box.cx(), tr->box.cy(), 0, 0, 0);
          if (e.valid)
            std::fprintf(stderr, "  err %.3f mrad", 1e3 * std::hypot(e.psi - g.psi, e.theta - g.theta));
        }
      }
      std::fprintf(stderr, "\n");
      proc_sum_ms = 0;
      proc_max_ms = 0;
    }
    if (args.frames > 0 && n >= uint64_t(args.frames)) break;
  }
  std::cerr << "stopping...\n";
  preview.stop();
  src->stop();
  if (clip.active()) {
    std::cerr << (clip.save(cfg.rt.clip_path) ? "clip saved: " : "clip save FAILED: ") << cfg.rt.clip_path << "\n";
  }
  log.close();
  const auto& s = per.stats();
  std::fprintf(stderr,
               "frames %llu  ncc ok/fail %llu/%llu  det batches %llu submitted %llu rejected %llu  gated-out %llu  "
               "template reinit %llu  stale det %llu  log drops %llu\n",
               (unsigned long long)s.frames, (unsigned long long)s.ncc_ok, (unsigned long long)s.ncc_fail,
               (unsigned long long)s.det_batches, (unsigned long long)s.det_submitted, (unsigned long long)s.det_rejected,
               (unsigned long long)s.meas_gated_out, (unsigned long long)s.template_reinit,
               (unsigned long long)s.stale_det, (unsigned long long)log.dropped());
  return 0;
}
