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
};

class PreviewServer {
 public:
  ~PreviewServer() { stop(); }
  // port > 0: MJPEG over HTTP; window: on-screen popup (needs a desktop session, DISPLAY)
  bool start(int port, bool window, int cpu, std::string* err);
  void stop();
  // Non-blocking: drops the frame if the server is busy with the previous one.
  void publish(const cv::Mat& gray, const PreviewOverlay& ov);
  int clients() const { return n_clients_.load(); }

 private:
  void run(int cpu);
  cv::Mat render(const cv::Mat& gray, const PreviewOverlay& ov) const;

  int listen_fd_ = -1;
  bool window_ = false;
  std::thread th_;
  std::atomic<bool> stop_{false};
  std::atomic<int> n_clients_{0};
  std::mutex mu_;
  cv::Mat pending_;
  PreviewOverlay pending_ov_;
  bool has_pending_ = false;
};

}  // namespace rcws
