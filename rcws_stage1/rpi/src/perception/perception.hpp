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
#include "perception/aim.hpp"
#include "perception/mot.hpp"
#include "perception/quality.hpp"
#include "perception/zncc_tracker.hpp"
#include "record/binlog.hpp"

namespace rcws {

enum class StateReason : uint8_t {
  NONE = 0, DET_ACQUIRE, SPRT_ACCEPT, SPRT_REJECT, ACQ_TIMEOUT, NO_MEAS, NCC_ONLY, MEAS_BACK,
  COAST_TIMEOUT, REACQ_DET, REACQ_TIMEOUT
};

// v4 (scenarios A-J) additions to the statistics
struct ScenarioStats {
  uint64_t other_target_skipped = 0;   // detections of a different board track kept out of this track
  uint64_t size_jump_rejected = 0;
  uint64_t reanchor_blocked = 0;       // strong detection elsewhere was another target: no re-anchor
  uint64_t lock_repointed = 0;         // board lost the lock track while Perception kept it
  uint64_t aim_hold_frames = 0, aim_follow_frames = 0, aim_predict_frames = 0;
  uint64_t quality_bad_frames = 0;
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
  const ScenarioStats& scenario_stats() const { return sstats_; }
  // v4: multi-target board, lock, quality and aim output (scenarios A-J)
  const TrackBoard& board() const { return mot_; }
  int lock_id() const { return lock_id_; }
  int designated_id() const { return designated_; }
  const AimOut& aim() const { return aim_; }
  const QualitySample& quality() const { return qual_.last(); }
  uint8_t quality_flags() const { return qual_.flags(); }
  std::optional<RectF> target_box() const;
  // Latest detector output (for the live preview only)
  const DetectionBatch& last_detections() const { return last_batch_; }
  // Live preview "stack" view: keep the model input of each request and the last NCC
  // correlation map. Off by default; when off nothing is retained (v3.1 behaviour).
  void set_preview_capture(bool on) { capture_ = on; ncc_.set_debug(on); }
  // Model input that produced last_detections() (empty until the first matching batch)
  const DetInputSnap& det_input() const { return shown_input_; }
  const NccDebug& ncc_debug() const { return ncc_.debug(); }

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

  void handle_batch(const DetectionBatch& b, const FrameView& f, int64_t now);
  // v4 helpers
  bool other_target(int id) const;          // detection belongs to an established track that is not ours
  bool reanchor_allowed(int id) const;
  void log_board(const DetectionBatch& b);
  void update_aim(const FrameView& f, int64_t now);
  void run_ncc(const FrameView& f, int64_t now);
  void update_timers(int64_t now);
  void schedule(const FrameView& f);
  FrameControls controls(const FrameView& f);
  void set_state(TrackState s, StateReason why, int64_t t);
  void reset_track();

  VisionMeas det_meas(const Detection& d, const DetectionBatch& b) const;
  bool submit_main_crop(const FrameView& f, cv::Rect r, DetKind kind);
  bool submit_global(const FrameView& f);
  bool submit_req(DetRequest&& req, const char* path);
  void log_meas(const VisionMeas& m, bool accepted, double d2);
  void log_est(int64_t t);

  const Config& cfg_;
  const CameraModel& cam_;
  IDetector* det_;
  BinLog* log_;
  GimbalAt gimbal_;
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
  bool capture_ = false;
  std::deque<DetInputSnap> inputs_;   // submitted, awaiting results (preview only)
  DetInputSnap shown_input_;
  // v4
  TrackBoard mot_;
  QualityMonitor qual_;
  AimOut aim_;
  ScenarioStats sstats_;
  int lock_id_ = -1;                  // board track this single-target loop follows
  int64_t lock_t_seen_ = 0;           // last time the lock track was seen (kept if the board drops it)
  bool lock_has_hist_ = false;
  int designated_ = -1;               // board's choice when not locked (priority + hysteresis)
  int64_t t_board_ = 0;               // capture time of the previous batch (ego shift)
  std::vector<int> batch_ids_;        // board id per detection of the batch being handled
};

}  // namespace rcws
