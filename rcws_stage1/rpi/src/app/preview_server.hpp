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
#include "perception/zncc_tracker.hpp"

namespace rcws {

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
  // "stack" view only (--preview-view stack)
  uint32_t det_seq = 0;          // frame of the batch in dets
  DetInputSnap det_in;           // model input that produced dets
  NccDebug ncc;                  // last NCC match (template already cloned)
};

// Compose the preview image. stack = false: annotated frame only (v3.1 view).
// stack = true: [annotated frame | detector input | NCC template, search, correlation map].
cv::Mat render_preview(const cv::Mat& gray, const PreviewOverlay& ov, bool stack, int clients);

class PreviewServer {
 public:
  ~PreviewServer() { stop(); }
  // port > 0: MJPEG over HTTP; window: on-screen popup (needs a desktop session, DISPLAY)
  bool start(int port, bool window, int cpu, std::string* err);
  void set_stack(bool on) { stack_ = on; }   // call before start()
  void stop();
  // Non-blocking: drops the frame if the server is busy with the previous one.
  void publish(const cv::Mat& gray, const PreviewOverlay& ov);
  int clients() const { return n_clients_.load(); }

 private:
  void run(int cpu);

  int listen_fd_ = -1;
  bool window_ = false;
  bool stack_ = false;
  std::thread th_;
  std::atomic<bool> stop_{false};
  std::atomic<int> n_clients_{0};
  std::mutex mu_;
  cv::Mat pending_;
  PreviewOverlay pending_ov_;
  bool has_pending_ = false;
};

}  // namespace rcws
