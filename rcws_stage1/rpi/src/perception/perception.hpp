// Flow 1 core (Sec 3): detection/tracking split, track state machine, measurement
// fusion into the estimator, detector scheduling, camera control policy.
// Single-threaded: everything runs on the vision thread, so a replayed input
// reproduces the same decisions.
#pragma once
#include <deque>
#include <optional>
#include <opencv2/core.hpp>

#include "camera/frame_source.hpp"
#include "common/config.hpp"
#include "common/types.hpp"
#include "detect/detector.hpp"
#include "estimation/estimator.hpp"
#include "perception/zncc_tracker.hpp"
#include "record/binlog.hpp"

namespace rcws {

enum class StateReason : uint8_t {
  NONE = 0, DET_ACQUIRE, SPRT_ACCEPT, SPRT_REJECT, ACQ_TIMEOUT, NO_MEAS, NCC_ONLY, MEAS_BACK,
  COAST_TIMEOUT, REACQ_DET, REACQ_TIMEOUT
};

struct PerceptionStats {
  uint64_t frames = 0, ncc_ok = 0, ncc_fail = 0, det_batches = 0, det_submitted = 0, det_rejected = 0;
  uint64_t meas_gated_out = 0, template_reinit = 0, stale_det = 0;
};

class Perception {
 public:
  Perception(const Config& cfg, const CameraModel& cam, IDetector* det, BinLog* log, GimbalAt gimbal);

  // Process one frame. Returns controls for the next camera request.
  FrameControls on_frame(const FrameView& f);

  TrackState state() const { return state_; }
  EstState estimate(int64_t t_ns) const { return est_.predict(t_ns); }
  const PerceptionStats& stats() const { return stats_; }
  std::optional<RectF> target_box() const;
  // Latest detector output (for the live preview only)
  const DetectionBatch& last_detections() const { return last_batch_; }

 private:
  struct HistFrame {
    uint32_t seq = 0;
    int64_t t_sof = 0;
    int32_t exp_us = 0;
    cv::Mat y;  // owned copy of main luma
  };
  struct NccHist {
    uint32_t seq;
    double u, v;
    float rho;
  };

  int64_t t_cap_of(const FrameView& f, double row) const;
  int64_t t_cap_of(int64_t t_sof, int32_t exp_us, double row) const;
  const HistFrame* hist(uint32_t seq) const;
  void push_hist(const FrameView& f);

  void handle_batch(const DetectionBatch& b, uint32_t now_seq, int64_t now);
  void run_ncc(const FrameView& f, int64_t now);
  void update_timers(int64_t now);
  void schedule(const FrameView& f);
  FrameControls controls(const FrameView& f);
  void set_state(TrackState s, StateReason why, int64_t t);
  void reset_track();

  VisionMeas det_meas(const Detection& d, const DetectionBatch& b) const;
  bool submit_main_crop(const FrameView& f, cv::Rect r, DetKind kind);
  bool submit_global(const FrameView& f);
  void log_meas(const VisionMeas& m, bool accepted, double d2);
  void log_est(int64_t t);

  const Config& cfg_;
  const CameraModel& cam_;
  IDetector* det_;
  BinLog* log_;
  Estimator est_;
  ZnccTracker ncc_;

  TrackState state_ = TrackState::SEARCH;
  int64_t t_state_ = 0;
  bool coast_ncc_only_ = false;   // COAST entered because only NCC was confirming
  int64_t t_last_meas_ = 0;       // any accepted measurement
  int64_t t_last_confirm_ = 0;    // detector-confirmed
  double sprt_L_ = 0;
  RectF tgt_box_{};               // last confirmed target size/position
  cv::Point2d ncc_off_;           // DET centre - NCC template centre (anchor bias, Sec 3.6)
  int off_n_ = 0;
  int strong_rejects_ = 0;        // consecutive strong phone detections rejected by the gate
  float gain_ = 0;

  std::deque<HistFrame> hist_;
  std::deque<NccHist> ncc_hist_;
  std::deque<cv::Rect> person_rois_;
  std::vector<cv::Rect> tiles_;
  size_t tile_idx_ = 0;
  uint32_t frame_counter_ = 0;
  int frames_since_det_ = 0;
  uint32_t cur_seq_ = 0;
  PerceptionStats stats_;
  DetectionBatch last_batch_;
};

}  // namespace rcws
