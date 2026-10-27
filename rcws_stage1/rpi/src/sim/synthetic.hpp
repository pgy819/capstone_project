// Synthetic scene + oracle detector for development without camera/NPU and for tests.
// The scene renders a textured phone-like target over a cluttered background with
// sensor noise; the oracle detector returns noisy boxes with latency and misses.
#pragma once
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <random>

#include "camera/frame_source.hpp"
#include "detect/detector.hpp"

namespace rcws {

struct SynthTraj {
  // centre position [px] at time t [s]
  std::function<cv::Point2d(double)> center;
  double tgt_w = 36, tgt_h = 74;          // ~2 m phone at f=982 px
  double occl_t0 = -1, occl_t1 = -1;      // full occlusion interval [s]
  double noise_sigma = 3.0;               // luma noise
  bool decoy = false;                     // a look-alike rectangle elsewhere
  bool plain = false;                     // low-texture phone (uniform back, thin bezel)
};

class SyntheticSource final : public IFrameSource {
 public:
  SyntheticSource(const CameraCfg& cam, SynthTraj traj, int n_frames, uint32_t seed = 1);
  bool start(std::string*) override { return true; }
  bool grab(Frame* f, int) override;
  void release(Frame&, const FrameControls& c) override { last_ctrl_ = c; }
  void stop() override {}
  bool eos() const override { return seq_ >= uint32_t(n_); }

  // ground truth of a delivered frame (visible=false while occluded)
  struct Truth { RectF box; bool visible; int64_t t_ns; };  // t_ns: capture time of the target row
  std::optional<Truth> truth(uint32_t seq) const;
  uint32_t current_seq() const { return seq_; }
  const FrameControls& last_controls() const { return last_ctrl_; }
  double t_of(uint32_t seq) const { return seq / double(cam_.fps); }

 private:
  CameraCfg cam_;
  SynthTraj traj_;
  int n_;
  uint32_t seq_ = 0;
  std::mt19937 rng_;
  cv::Mat bg_, tex_, y_, u_, v_, lo_y_, lo_u_, lo_v_;
  std::map<uint32_t, Truth> truth_;
  FrameControls last_ctrl_;
};

struct OracleCfg {
  int latency_frames = 2;
  double box_sigma_px = 1.5;   // per-edge jitter in main-stream pixels
  double box_sigma_rel = 0.0;  // additional jitter as a fraction of box size (large, close targets)
  double p_miss = 0.05;
  double p_false = 0.02;       // per request
  int min_side_in = 8;         // smaller than this in model input -> missed
};

class OracleDetector final : public IDetector {
 public:
  OracleDetector(const SyntheticSource& src, cv::Size input, OracleCfg c, uint32_t seed = 2);
  cv::Size input_size() const override { return in_; }
  int in_flight() const override { return int(pending_.size()); }
  bool submit(DetRequest&& r) override;
  std::optional<DetectionBatch> poll() override;
  int max_in_flight = 2;

 private:
  const SyntheticSource& src_;
  cv::Size in_;
  OracleCfg c_;
  std::mt19937 rng_;
  std::deque<std::pair<uint32_t, DetectionBatch>> pending_;  // ready_at_seq, batch
};

}  // namespace rcws
