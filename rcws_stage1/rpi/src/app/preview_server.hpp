// Live MJPEG preview over HTTP for headless (SSH) operation.
// Open http://<pi-address>:<port>/ in a browser on the PC.
// The vision thread only copies a small luma image + overlay data (non-blocking);
// drawing, JPEG encoding and network I/O run on the server's own thread.
#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <opencv2/core.hpp>

#include "common/types.hpp"
#include "detect/detector.hpp"
#include "perception/aim.hpp"
#include "perception/mot.hpp"
#include "perception/zncc_tracker.hpp"

namespace rcws {

// v4: one track of the multi-target board, for drawing
struct MotView {
  int id = 0;
  MotState st = MotState::TENTATIVE;
  RectF box;
  float prio = 0;
  bool lock = false, designated = false;
};

struct PreviewOverlay {
  TrackState state = TrackState::SEARCH;
  bool has_target = false;
  RectF target;                  // main-stream pixels
  std::vector<Detection> dets;   // main-stream pixels
  RectF det_crop;                // region the latest detection ran on
  float scale = 1.f;             // main px -> preview px
  double fps = 0, proc_ms = 0;
  bool est_valid = false;
  double psi_deg = 0, theta_deg = 0, rate_psi = 0, rate_theta = 0, range_m = 0;
  uint32_t seq = 0;
  // v4 (scenarios A-J): board tracks, aim output gate, image quality
  std::vector<MotView> tracks;
  AimMode aim_mode = AimMode::HOLD;
  uint16_t aim_reasons = 0;
  float hint_dx = 0, hint_dy = 0;
  uint8_t qflags = 0;
  // "stack" view only (--preview-view stack)
  uint32_t det_seq = 0;          // frame of the batch in dets
  DetInputSnap det_in;           // model input that produced dets
  NccDebug ncc;                  // last NCC match (template already cloned)
};

// Compose the preview image. stack = false: annotated frame only (v3.1 view).
// stack = true: [annotated frame | detector input | NCC template, search, correlation map].
// `img` is the preview frame, grey (CV_8UC1) or colour (CV_8UC3, BGR).
cv::Mat render_preview(const cv::Mat& img, const PreviewOverlay& ov, bool stack, int clients);

// Owned copy of the preview-size I420 planes (lores stream); u / v empty for a grey source.
struct WebFrame {
  cv::Mat y, u, v;
  float scale = 1.f;   // main px -> these px
};
// Vision thread: plane copies only (~0.35 MB memcpy at 640x360), no colour conversion.
WebFrame copy_web_frame(const FrameView& f);
cv::Mat web_frame_bgr(const WebFrame& f);

// A newly acquired target, shown on the web page with "keep tracking / stop tracking".
struct WebCapture {
  int track_id = -1;
  int64_t t_ns = 0;            // CLOCK_MONOTONIC
  std::string cls;             // target class name
  float score = 0;
  RectF box;                   // main-stream pixels
  WebFrame frame;
};

// HTTP server for the operator web page (single port):
//   GET /              web/index.html (page: live view, captures, controls, dashboard)
//   GET /stream.mjpg   annotated live view (MJPEG)
//   GET /ws            WebSocket: server pushes {"type":"status"|"event"|"capture"|"sys"|"hello"}
//                      JSON, the page sends {"cmd":...} commands (take_commands())
//   GET /snap/N.jpg    capture N (whole frame with the box), /crop/N.jpg the target crop
//   GET /thumb/ID.jpg  current crop of board track ID (candidate list)
// Everything heavy (colour conversion, drawing, JPEG, sockets) runs on the server thread, pinned
// to the logger core; the vision thread only copies planes and strings (non-blocking).
class PreviewServer {
 public:
  ~PreviewServer() { stop(); }
  // port > 0: web page + MJPEG over HTTP; window: on-screen popup (needs a desktop session, DISPLAY)
  bool start(int port, bool window, int cpu, std::string* err);
  void set_stack(bool on) { stack_ = on; }            // call before start()
  void set_web_root(const std::string& dir) { web_root_ = dir; }   // call before start()
  void stop();
  // Non-blocking: drops the frame if the server is busy with the previous one.
  void publish(const cv::Mat& gray, const PreviewOverlay& ov);
  void publish_frame(WebFrame&& f, const PreviewOverlay& ov);
  void set_status(std::string json);       // latest state, sent to WebSocket clients
  void push_event(std::string json);       // one-off message to all WebSocket clients
  void push_capture(WebCapture&& c);       // stored (JPEG) and announced on the server thread
  void mark_answered(int track_id);        // operator answered the prompt: not offered to new pages
  std::vector<std::string> take_commands();
  int clients() const { return n_clients_.load(); }
  int ws_clients() const { return n_ws_.load(); }

 private:
  struct Conn {
    int fd = -1;
    int kind = 0;              // 0 request pending, 1 MJPEG stream, 2 WebSocket
    std::string in;
    int64_t t0 = 0;
  };
  struct Snap {
    int n = 0;
    std::vector<uchar> full, crop;
    std::string meta;          // JSON object
    int track = -1;
    bool answered = false;
  };
  void run(int cpu);
  void handle_request(Conn& c, std::vector<Conn>& conns);
  void broadcast(const std::string& text, std::vector<Conn>& conns);
  void store_capture(WebCapture& c, std::vector<Conn>& conns);
  std::string hello() const;

  int listen_fd_ = -1;
  bool window_ = false;
  bool stack_ = false;
  std::string web_root_ = "web";
  std::thread th_;
  std::atomic<bool> stop_{false};
  std::atomic<int> n_clients_{0}, n_ws_{0};
  std::mutex mu_;
  WebFrame pending_;
  PreviewOverlay pending_ov_;
  bool has_pending_ = false;
  std::string status_;
  bool status_dirty_ = false;
  std::vector<std::string> events_;
  std::vector<WebCapture> captures_in_;
  std::vector<std::string> commands_;
  std::vector<int> answered_in_;
  // server thread only
  cv::Mat last_bgr_;
  PreviewOverlay last_ov_;
  std::vector<Snap> snaps_;
  int next_snap_ = 1;
  std::string last_status_sent_;
};

}  // namespace rcws
