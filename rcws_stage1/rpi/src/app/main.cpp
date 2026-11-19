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
#include "app/operator.hpp"
#include "app/web_util.hpp"
#include "link/uart_link.hpp"
#include <opencv2/imgcodecs.hpp>
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
  std::string preview_view = "plain";  // plain | stack
  std::string scenario = "sine";       // synthetic source only (v4 scenario set)
  std::string record;                  // directory: annotated frames at 30 fps of scene time (demo video)
  std::string uart;                    // STM32 link device (packet v2), empty = off
  int uart_baud = LINK_BAUD_DEFAULT;
  double link_lead_ms = 0;             // extra prediction of the sent aim beyond the processing time
  std::string web_root = "web";        // operator web page (index.html) served with --preview
  double jog_dps = 20;                 // web jog rate at full deflection
};

void usage() {
  std::cerr << "usage: rcws_vision [--config f.ini] [--source camera|synthetic|replay:CLIP]\n"
               "                   [--detector hailo|oracle|none|replay:RLOG] [--log out.rlog]\n"
               "                   [--frames N] [--clip-seconds S] [--print-every N]\n"
               "                   [--preview PORT] [--preview-fps N]   (live view: http://<pi>:PORT/)\n"
               "                   [--window]                           (popup window on the Pi desktop)\n"
               "                   [--preview-view plain|stack]         (stack: + detector input + NCC map)\n"
               "                   [--scenario NAME]                    (synthetic: " << scenario_names() << ")\n"
               "                   [--record DIR]                       (annotated frames, 30 fps of scene time:\n"
               "                                                         ffmpeg -framerate 30 -i DIR/f_%05d.jpg demo.mp4)\n"
               "                   [--uart /dev/serial0] [--uart-baud 115200] [--link-lead-ms 0]\n"
               "                                                        (drive the STM32: packet v2 AIM frames)\n"
               "                   [--web-root DIR] [--jog-dps 20]      (operator page at http://<pi>:PORT/)\n";
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
    if (k == "--scenario" && val(&a->scenario)) continue;
    if (k == "--record" && val(&a->record)) continue;
    if (k == "--uart" && val(&a->uart)) continue;
    if (k == "--uart-baud" && val(&v)) { a->uart_baud = std::stoi(v); continue; }
    if (k == "--link-lead-ms" && val(&v)) { a->link_lead_ms = std::stod(v); continue; }
    if (k == "--web-root" && val(&a->web_root)) continue;
    if (k == "--jog-dps" && val(&v)) { a->jog_dps = std::stod(v); continue; }
    if (k == "--preview-view" && val(&a->preview_view)) {
      if (a->preview_view != "plain" && a->preview_view != "stack") return false;
      continue;
    }
    return false;
  }
  return true;
}

double deg(double r) { return r * 180.0 / M_PI; }

// web page target classes (COCO ids of the fixed YOLOv8s HEF)
struct WebClass {
  const char* name;
  int coco;
};
constexpr WebClass kClasses[] = {{"phone", 67}, {"ball", 32}, {"person", 0}};
const char* class_name(int coco) {
  for (const auto& c : kClasses) if (c.coco == coco) return c.name;
  return "other";
}
const char* guard_name(uint8_t g) {
  static const char* k[] = {"RUN", "HOLD", "TORQUE_OFF", "RAMP", "ESTOP"};
  return g < 5 ? k[g] : "?";
}
double goal_deg(uint16_t g) { return (int(g) - 2048) * 360.0 / 4096.0; }

std::string event_json(const char* level, const std::string& text) {
  char t[32];
  std::snprintf(t, sizeof t, "%.3f", now_ns() * 1e-9);
  return std::string("{\"type\":\"event\",\"t\":") + t + ",\"level\":\"" + level + "\",\"text\":\"" +
         web::json_escape(text) + "\"}";
}

// Self-check for the web page (connection check): camera, detector, STM32 link, servo answers.
struct SelfCheck {
  int64_t t_start = 0;     // 0 = idle
  uint64_t frames0 = 0, batches0 = 0;
};

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
    bool ok = false;
    int nf = args.frames;
    SynthTraj tr = make_scenario(args.scenario, cfg.cam, &nf, &ok);
    if (!ok) { std::cerr << "unknown scenario " << args.scenario << " (" << scenario_names() << ")\n"; return 2; }
    if (args.scenario == "sine" && args.frames <= 0) nf = 1200;
    auto s = std::make_unique<SyntheticSource>(cfg.cam, tr, nf);
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
  UartLink link;
  if (!args.uart.empty()) {
    if (!link.open(args.uart, args.uart_baud, &err)) { std::cerr << "uart: " << err << "\n"; return 1; }
    std::cerr << "uart: STM32 link on " << args.uart << " @ " << args.uart_baud << " (packet v2)\n";
  }
  uint8_t link_seq = 0;
  uint32_t link_rx_logged = 0;
  OperatorCfg ocfg;
  ocfg.jog_dps = args.jog_dps;
  Operator op(ocfg);
  int target_coco = det ? det->target_class() : 67;
  int last_capture_lock = -1;
  TrackState prev_state = TrackState::SEARCH;
  int prev_guard = -1;
  bool prev_link_fresh = false;
  SelfCheck check;
  check.t_start = now_ns();
  double det_ms = 0;
  uint32_t last_det_seq = 0;
  PreviewServer preview;
  const bool want_view = args.preview_port > 0 || args.window;
  if (want_view) {
    if (args.window && !std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
      std::cerr << "--window: no desktop session (DISPLAY unset). Over SSH run with: DISPLAY=:0 ./build/rcws_vision ...\n";
      args.window = false;
    }
    const bool stack = args.preview_view == "stack";
    preview.set_stack(stack);
    preview.set_web_root(args.web_root);
    per.set_preview_capture(stack);
    if (preview.start(args.preview_port, args.window, cfg.rt.logger_cpu, &err)) {
      if (args.preview_port > 0) std::cerr << "web page: open http://<pi-address>:" << args.preview_port << "/ in a browser\n";
      if (args.window) std::cerr << "preview: popup window 'RCWS Stage 1' on the Pi desktop\n";
    } else {
      std::cerr << "preview disabled: " << err << "\n";
    }
  }
  double last_fps = 0, last_proc = 0;
  int64_t t_prev_frame = 0;
  int64_t t_preview = 0;
  int n_rec = 0;
  if (!args.record.empty() && args.source == "camera")
    std::cerr << "--record renders on the vision thread: use it with synthetic or replay sources\n";

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
    // ---- operator commands from the web page (applied between frames)
    for (const std::string& cmd : preview.take_commands()) {
      const int64_t tc = now_ns();
      const std::string k = web::json_str(cmd, "cmd").value_or("");
      const int id = int(web::json_num(cmd, "id").value_or(-1));
      std::string say;
      if (k == "stop_track" || k == "keep") preview.mark_answered(id);
      if (k == "stop_track") { per.op_stop_target(id); say = "#" + std::to_string(id) + " 추적 중지 (다시 잡지 않음)"; }
      else if (k == "keep") { say = "#" + std::to_string(id) + " 계속 추적"; }
      else if (k == "select") { per.op_select(id); op.set_mode(OpMode::AUTO, tc); say = "#" + std::to_string(id) + " 대상으로 전환"; }
      else if (k == "clear_stops") { per.op_clear_exclusions(); say = "추적 중지 목록 해제 (다시 잡을 수 있음)"; }
      else if (k == "mode") {
        const bool aut = web::json_str(cmd, "value").value_or("") == "auto";
        say = op.set_mode(aut ? OpMode::AUTO : OpMode::MANUAL, tc) ? (aut ? "자동 추적 모드" : "수동 모드 (정지 유지)")
                                                                   : "모드 변경 거부: 먼저 비상정지를 해제하세요";
      } else if (k == "jog") {
        op.jog(web::json_num(cmd, "pan").value_or(0), web::json_num(cmd, "tilt").value_or(0), tc);
      } else if (k == "home") { op.home(tc); say = "홈 위치(정면)로 복귀"; }
      else if (k == "estop") { op.estop(tc); say = "비상정지(소프트웨어): 서보 토크 끔"; }
      else if (k == "estop_clear") { op.estop_clear(tc); say = "비상정지 해제: 수동 모드, 서보는 천천히 다시 힘을 받음"; }
      else if (k == "class") {
        const std::string nm = web::json_str(cmd, "name").value_or("");
        int coco = -1;
        for (const auto& wc : kClasses) if (nm == wc.name) coco = wc.coco;
        if (coco >= 0 && det && det->set_target_class(coco)) {
          target_coco = coco;
          per.op_retarget();
          last_capture_lock = -1;
          say = std::string("검출 클래스 변경: ") + nm + (coco == 67 ? "" : " (휴대폰 기준으로 조정된 추적기라 성능이 낮을 수 있음)");
        } else {
          say = "이 검출기는 클래스 변경을 지원하지 않음 (합성/재생 시험)";
        }
      } else if (k == "selftest") { check.t_start = tc; say = "연결 점검 시작 (3초)"; }
      else if (k == "quit") { say = "웹에서 종료 요청"; g_stop = true; }
      if (!say.empty()) preview.push_event(event_json("info", say));
    }
    if (check.t_start && check.frames0 == 0) { check.frames0 = n + 1; check.batches0 = per.stats().det_batches; }

    const FrameControls c = per.on_frame(fr.view);
    if (link.is_open()) {   // U1: aim to the STM32 right after the frame is processed
      const LinkStatusView ls0 = link.status();
      const LinkAim_t la = op.frame(per.aim(), link_seq++, args.link_lead_ms * 1e-3, now_ns(), &ls0);
      link.send(la);
      if (log.is_open()) {
        RecLinkTx r{per.aim().t, fr.view.seq, la.seq, la.mode, la.flags, 0, la.err_pan, la.err_tilt, la.rate_pan, la.rate_tilt};
        log.write(RecType::LINK_TX, r);
        const LinkStatusView ls = link.status();
        if (ls.valid && ls.frames != link_rx_logged) {
          link_rx_logged = ls.frames;
          RecLinkRx q{ls.t_ns, ls.s.seq_echo, ls.s.guard, ls.s.flags, 0, ls.s.pan_goal, ls.s.tilt_goal, ls.s.rx_ok, ls.s.rx_err};
          log.write(RecType::LINK_RX, q);
        }
      }
    }
    const double ms = (now_ns() - t0) * 1e-6;
    proc_max_ms = std::max(proc_max_ms, ms);
    proc_sum_ms += ms;
    if (clip.active()) clip.push(fr.view);
    // ---- web page: events, captures, status (cheap: strings and plane copies only)
    if (want_view && args.preview_port > 0) {
      const int64_t tw = now_ns();
      const TrackState st = per.state();
      if (st != prev_state) {
        preview.push_event(event_json("state", std::string(to_string(prev_state)) + " -> " + to_string(st) +
                                                   (per.lock_id() >= 0 ? "  (track #" + std::to_string(per.lock_id()) + ")" : "")));
        prev_state = st;
      }
      if (st == TrackState::TRACK && per.lock_id() >= 0 && per.lock_id() != last_capture_lock) {
        last_capture_lock = per.lock_id();
        WebCapture cap;
        cap.track_id = per.lock_id();
        cap.t_ns = tw;
        cap.cls = class_name(target_coco);
        if (const MotTrack* k = per.board().find(per.lock_id())) cap.score = k->score;
        if (auto b = per.target_box()) cap.box = *b;
        cap.frame = copy_web_frame(fr.view);
        preview.push_capture(std::move(cap));
      }
      const auto& lb0 = per.last_detections();
      if (lb0.frame_seq != last_det_seq && lb0.t_done_ns > lb0.t_submit_ns) {
        last_det_seq = lb0.frame_seq;
        det_ms = det_ms == 0 ? (lb0.t_done_ns - lb0.t_submit_ns) * 1e-6 : 0.9 * det_ms + 0.1 * (lb0.t_done_ns - lb0.t_submit_ns) * 1e-6;
      }
      if (link.is_open()) {
        const LinkStatusView ls = link.status();
        const bool fresh = ls.valid && tw - ls.t_ns < 300000000LL;
        if (fresh != prev_link_fresh) {
          preview.push_event(event_json(fresh ? "info" : "warn", fresh ? "STM32 링크 연결됨" : "STM32 응답 없음 (0.3초 이상)"));
          prev_link_fresh = fresh;
        }
        if (fresh && int(ls.s.guard) != prev_guard) {
          preview.push_event(event_json(ls.s.guard == 4 ? "warn" : "info", std::string("STM32 상태: ") + guard_name(ls.s.guard)));
          prev_guard = ls.s.guard;
        }
      }
      if (check.t_start && tw - check.t_start > 3000000000LL) {   // connection check result
        const bool cam_ok = n + 1 > check.frames0 + 30;
        const bool det_ok = per.stats().det_batches > check.batches0;
        const LinkStatusView ls = link.status();
        const bool link_ok = link.is_open() && ls.valid && tw - ls.t_ns < 300000000LL;
        const bool ping = link_ok && (ls.s.flags & LINK_S_PING_DONE);
        char j[512];
        std::snprintf(j, sizeof j,
                      "{\"type\":\"selfcheck\",\"camera\":%s,\"detector\":%s,\"link\":\"%s\",\"pan\":\"%s\",\"tilt\":\"%s\","
                      "\"web\":true}",
                      cam_ok ? "true" : "false", det_ok ? "true" : "false",
                      !link.is_open() ? "off" : link_ok ? "ok" : "silent",
                      !ping ? "unknown" : (ls.s.flags & LINK_S_PAN_PING_OK) ? "ok" : "no answer",
                      !ping ? "unknown" : (ls.s.flags & LINK_S_TILT_PING_OK) ? "ok" : "no answer");
        preview.push_event(j);
        check = SelfCheck{};
      }
    }
    {
      const int64_t t = now_ns();
      if (t_prev_frame) last_fps = 0.9 * last_fps + 0.1 * (1e9 / std::max<int64_t>(1, t - t_prev_frame));
      t_prev_frame = t;
      last_proc = 0.9 * last_proc + 0.1 * ms;
      // live preview at a low rate: only a small luma copy happens on this thread
      const bool view_due = want_view && t - t_preview >= 1000000000LL / std::max(1, args.preview_fps);
      const bool rec_due = !args.record.empty() && fr.view.seq % uint32_t(std::max(1, cfg.cam.fps / 30)) == 0;
      if (view_due || rec_due) {
        if (view_due) t_preview = t;
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
        for (const auto& k : per.board().tracks())
          ov.tracks.push_back({k.id, k.st, k.box(), k.priority, k.id == per.lock_id(), k.id == per.designated_id()});
        ov.aim_mode = per.aim().mode;
        ov.aim_reasons = per.aim().reasons;
        ov.hint_dx = per.aim().hint_dx;
        ov.hint_dy = per.aim().hint_dy;
        ov.qflags = per.quality_flags();
        if (args.preview_view == "stack") {   // buffer headers only; the template is small and cloned
          ov.det_seq = lb.frame_seq;
          ov.det_in = per.det_input();
          ov.ncc = per.ncc_debug();
          ov.ncc.tmpl = ov.ncc.tmpl.clone();
        }
        const EstState e = per.estimate(fr.view.t_sof_ns);
        if (e.valid) {
          ov.est_valid = true;
          ov.psi_deg = deg(e.psi); ov.theta_deg = deg(e.theta);
          ov.rate_psi = deg(e.psi_d); ov.rate_theta = deg(e.theta_d);
          ov.range_m = e.range_m;
        }
        if (view_due && args.preview_port > 0) {
          preview.publish_frame(copy_web_frame(fr.view), ov);
          std::string j;
          j.reserve(2048);
          char b[640];
          const int64_t tw = now_ns();
          std::snprintf(b, sizeof b,
                        "{\"type\":\"status\",\"t\":%.3f,\"frame\":%u,\"fps\":%.1f,\"proc_ms\":%.2f,\"det_ms\":%.1f,"
                        "\"drop\":%llu,\"state\":\"%s\",\"lock\":%d,\"designated\":%d,\"cls\":\"%s\",\"aim\":\"%s\","
                        "\"aim_text\":\"%s\",\"quality\":%u,\"img\":{\"w\":%d,\"h\":%d,\"scale\":%.5f},",
                        tw * 1e-9, fr.view.seq, last_fps, last_proc, det_ms, (unsigned long long)dropped, to_string(per.state()),
                        per.lock_id(), per.designated_id(), class_name(target_coco), to_string(per.aim().mode),
                        web::json_escape(to_string(per.aim().mode)).c_str(), unsigned(per.quality_flags()), gray.cols, gray.rows, scale);
          j += b;
          const EstState e2 = per.estimate(fr.view.t_sof_ns);
          if (e2.valid && per.lock_id() >= 0) {
            const auto tb = per.target_box();
            std::snprintf(b, sizeof b,
                          "\"target\":{\"id\":%d,\"psi\":%.3f,\"theta\":%.3f,\"psi_d\":%.2f,\"theta_d\":%.2f,\"range\":%.2f,"
                          "\"u\":%.1f,\"v\":%.1f,\"w\":%.1f,\"h\":%.1f},",
                          per.lock_id(), deg(e2.psi), deg(e2.theta), deg(e2.psi_d), deg(e2.theta_d), e2.range_m,
                          tb ? tb->cx() : 0.f, tb ? tb->cy() : 0.f, tb ? tb->w : 0.f, tb ? tb->h : 0.f);
            j += b;
          } else {
            j += "\"target\":null,";
          }
          j += "\"tracks\":[";
          bool first = true;
          for (const auto& k : per.board().tracks()) {
            if (k.st == MotState::LOST) continue;
            const RectF bx = k.box();
            std::snprintf(b, sizeof b,
                          "%s{\"id\":%d,\"st\":\"%s\",\"prio\":%.2f,\"score\":%.2f,\"lock\":%s,\"stopped\":%s,"
                          "\"u\":%.0f,\"v\":%.0f,\"w\":%.0f,\"h\":%.0f}",
                          first ? "" : ",", k.id, to_string(k.st), k.priority, k.score, k.id == per.lock_id() ? "true" : "false",
                          per.board().excluded(k.id) ? "true" : "false", bx.cx(), bx.cy(), bx.w, bx.h);
            j += b;
            first = false;
          }
          j += "],\"stopped\":[";
          first = true;
          for (int id : per.board().excluded_ids()) { j += (first ? "" : ",") + std::to_string(id); first = false; }
          std::snprintf(b, sizeof b, "],\"op\":{\"mode\":\"%s\",\"activity\":\"%s\",\"estop\":%s,\"preferred\":%d,\"jog_dps\":%.0f},",
                        to_string(op.mode()), op.activity(tw), op.estopped() ? "true" : "false", per.board().preferred(), args.jog_dps);
          j += b;
          if (link.is_open()) {
            const LinkStatusView ls = link.status();
            const bool fresh = ls.valid && tw - ls.t_ns < 300000000LL;
            std::snprintf(b, sizeof b,
                          "\"link\":{\"open\":true,\"fresh\":%s,\"guard\":\"%s\",\"flags\":%u,\"pan_deg\":%.2f,\"tilt_deg\":%.2f,"
                          "\"pan_goal\":%u,\"tilt_goal\":%u,\"rx_ok\":%u,\"rx_err\":%u,\"sent\":%llu,\"werr\":%llu,\"crc_err\":%u}}",
                          fresh ? "true" : "false", ls.valid ? guard_name(ls.s.guard) : "-", unsigned(ls.s.flags),
                          goal_deg(ls.s.pan_goal), goal_deg(ls.s.tilt_goal), ls.s.pan_goal, ls.s.tilt_goal, ls.s.rx_ok, ls.s.rx_err,
                          (unsigned long long)link.sent(), (unsigned long long)link.write_errors(), ls.crc_err);
            j += b;
          } else {
            j += "\"link\":{\"open\":false}}";
          }
          preview.set_status(std::move(j));
        } else if (view_due) {
          preview.publish(gray, ov);
        }
        if (rec_due) {   // demo recording: rendered on this thread (not for live camera runs)
          char fn[64];
          std::snprintf(fn, sizeof fn, "/f_%05d.jpg", n_rec++);
          cv::imwrite(args.record + fn, render_preview(gray, ov, args.preview_view == "stack", 0));
        }
      }
    }
    if (synth && log.is_open()) {
      if (auto tr = synth->truth(fr.view.seq)) {
        RecTruth rt{tr->t_ns, fr.view.seq, tr->box.cx(), tr->box.cy(), tr->box.w, tr->box.h, uint8_t(tr->visible), {0, 0, 0}};
        log.write(RecType::TRUTH, rt);
      }
      const auto all = synth->truth_all(fr.view.seq);
      for (size_t o = 1; o < all.size(); ++o) {
        const auto& tr = all[o];
        RecTruth2 rt{tr.t_ns, fr.view.seq, int32_t(o), tr.box.cx(), tr.box.cy(), tr.box.w, tr.box.h, uint8_t(tr.visible), {0, 0, 0}};
        log.write(RecType::TRUTH2, rt);
      }
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
      if (link.is_open()) {
        static const char* kGuard[] = {"RUN", "HOLD", "TORQUE_OFF", "RAMP", "ESTOP", "?"};
        const LinkStatusView ls = link.status();
        if (ls.valid && now_ns() - ls.t_ns < 200000000LL)
          std::fprintf(stderr, "  stm32 %s pan %u tilt %u rx ok/err %u/%u%s", kGuard[std::min<int>(ls.s.guard, 5)],
                       ls.s.pan_goal, ls.s.tilt_goal, ls.s.rx_ok, ls.s.rx_err,
                       (ls.s.flags & (LINK_S_ESTOP_B1 | LINK_S_ESTOP_SW)) ? " ESTOP" : "");
        else
          std::fprintf(stderr, "  stm32 silent");
      }
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
  if (link.is_open()) {   // last word to the STM32: hold where you are
    LinkAim_t h{};
    h.seq = link_seq++;
    for (int i = 0; i < 3; ++i) link.send(h);
    usleep(20000);
    std::fprintf(stderr, "STM32 link: AIM sent %llu  write errors %llu  stale flushes %llu  STATUS received %u (crc err %u)\n",
                 (unsigned long long)link.sent(), (unsigned long long)link.write_errors(),
                 (unsigned long long)link.stale_flushes(), link.status().frames, link.status().crc_err);
    link.close();
  }
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
  const auto& v = per.scenario_stats();
  std::fprintf(stderr,
               "v4: track ids %d  other-target skipped %llu  size-jump rejected %llu  re-anchor blocked %llu  "
               "lock re-pointed %llu  ambiguity events %llu  aim FOLLOW/PREDICT/HOLD %llu/%llu/%llu  bad-image frames %llu\n",
               per.board().next_id() - 1, (unsigned long long)v.other_target_skipped, (unsigned long long)v.size_jump_rejected,
               (unsigned long long)v.reanchor_blocked, (unsigned long long)v.lock_repointed,
               (unsigned long long)per.board().id_switch_guards(), (unsigned long long)v.aim_follow_frames,
               (unsigned long long)v.aim_predict_frames, (unsigned long long)v.aim_hold_frames,
               (unsigned long long)v.quality_bad_frames);
  return 0;
}
