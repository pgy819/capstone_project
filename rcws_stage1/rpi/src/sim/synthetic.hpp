// Synthetic scene + oracle detector for development without camera/NPU and for tests.
// The scene renders a textured phone-like target over a cluttered background with
// sensor noise; the oracle detector returns noisy boxes with latency and misses.
#pragma once
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "camera/frame_source.hpp"
#include "detect/detector.hpp"

namespace rcws {

// v4: additional phones for the multi-target scenarios (A-D)
struct SynthObj {
  std::function<cv::Point2d(double)> center;
  double w = 36, h = 74;
  uint8_t u = 128, v = 128;               // case colour (chroma), 128/128 = grey
  double occl_t0 = -1, occl_t1 = -1;
};

struct SynthTraj {
  // centre position [px] at time t [s]
  std::function<cv::Point2d(double)> center;
  double tgt_w = 36, tgt_h = 74;          // ~2 m phone at f=982 px
  double occl_t0 = -1, occl_t1 = -1;      // full occlusion interval [s]
  double noise_sigma = 3.0;               // luma noise
  bool decoy = false;                     // a look-alike rectangle elsewhere
  bool plain = false;                     // low-texture phone (uniform back, thin bezel)
  // v4 scenarios
  uint8_t u = 128, v = 128;               // primary case colour
  std::vector<SynthObj> extra;            // other phones
  cv::Rect occluder;                      // static pillar drawn in front of every phone
  double dark_t0 = -1, dark_t1 = -1;      // low-light interval: luma x dark_gain, detector misses
  double dark_gain = 0.12;
  double merge_p = 0.7;                   // oracle reports two overlapping phones as one box
  double p_false = -1;                    // oracle false alarms per request (-1: OracleCfg value)
};

// Named scenarios for tests and the demo (rcws_vision --source synthetic --scenario NAME):
// sine, parallel, cross, merge, pillar, longocc, exit, dark, clutter
SynthTraj make_scenario(const std::string& name, const CameraCfg& cam, int* frames, bool* ok);
const char* scenario_names();

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
  // all phones of a frame: [0] primary, [1..] SynthTraj::extra
  std::vector<Truth> truth_all(uint32_t seq) const;
  bool dark(uint32_t seq) const { const double t = t_of(seq); return t >= traj_.dark_t0 && t < traj_.dark_t1; }
  const SynthTraj& traj() const { return traj_; }
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
  std::map<uint32_t, std::vector<Truth>> truth_all_;
  std::vector<cv::Mat> extra_tex_;
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
