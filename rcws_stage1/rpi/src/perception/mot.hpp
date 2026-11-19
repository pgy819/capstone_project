// Multi-target track board (scenarios A-D): every phone detection is associated to a set of
// tracks so the single-target loop (Perception) knows WHICH phone it is following.
//
//   association : ByteTrack-style two stages (confident detections first, weak ones only to
//                 tracks left over), cost = 1 - IoU + normalised centre distance + appearance,
//                 optimal assignment (Hungarian) on the small cost matrix.
//   motion      : constant-velocity Kalman per track in main-stream pixels; gimbal motion is
//                 removed with the camera model before prediction (ego shift, scenario E).
//   appearance  : 8x8 U/V chroma + 8 luma bins histogram inside the box (cheap colour re-ID).
//   states      : TENTATIVE -> CONFIRMED <-> OVERLAP / OCCLUDED / EXITING -> LOST (kept for re-ID)
//                 AMBIGUOUS after two tracks separate and the identities cannot be told apart.
//   priority    : weighted score (centre, size, approach, confidence, age) + hysteresis.
//
// Detections only cover the requested crop (ROI, tile or whole frame): a track is counted as
// missed only when its prediction lies inside the crop that was searched.
// Runs on the vision thread, deterministic for a replayed input.
#pragma once
#include <array>
#include <cstdint>
#include <vector>

#include "common/config.hpp"
#include "common/types.hpp"

namespace rcws {

enum class MotState : uint8_t { TENTATIVE = 0, CONFIRMED, OVERLAP, OCCLUDED, EXITING, AMBIGUOUS, LOST };
const char* to_string(MotState s);

constexpr int kHistBins = 8 * 8 + 8;   // U x V chroma + luma
using AppHist = std::array<float, kHistBins>;

// Box colour histogram from the main-stream planes (U/V may be null: luma only).
bool box_histogram(const FrameView& f, const RectF& box, AppHist* out);
// Bhattacharyya distance 0 (same) .. 1 (disjoint)
float hist_distance(const AppHist& a, const AppHist& b);

struct MotTrack {
  int id = 0;
  MotState st = MotState::TENTATIVE;
  // Kalman state (centre u, v [px], velocity [px/s]) and covariance
  double x[4] = {0, 0, 0, 0};
  double Pu[4] = {0}, Pv[4] = {0};   // per-axis 2x2 covariance [pp pv; vp vv]
  float w = 0, h = 0;              // smoothed box size
  float log_size_rate = 0;         // d ln(sqrt(area)) / dt: > 0 approaching
  int64_t t = 0;                   // time of the state (prediction time)
  int64_t t_seen = 0;              // last associated detection (capture time)
  int64_t t_born = 0;
  int64_t t_state = 0;             // entry time of the current state
  int hits = 0, misses = 0;        // consecutive misses while observable
  float score = 0;                 // smoothed detection score
  AppHist hist{};
  bool has_hist = false;
  int partner = -1;                // id of the track it overlaps / was confused with
  float exit_dx = 0, exit_dy = 0;  // unit direction it left the frame (LOST after EXITING)
  bool exited = false;
  bool was_overlapped = false;     // set while overlapping, resolved on separation
  bool revived = false;            // re-identified from LOST, not yet confirmed again
  int64_t t_ambiguous_until = 0;
  float priority = 0;
  RectF box() const { return {float(x[0] - w / 2), float(x[1] - h / 2), w, h}; }
};

struct MotUpdate {
  std::vector<int> det_track;      // per detection in the batch: track id, -1 = none
};

class TrackBoard {
 public:
  TrackBoard(const MotCfg& c, int img_w, int img_h) : c_(c), W_(img_w), H_(img_h) {}

  // Ego motion (scenario E): pixel shift the scene underwent between two times because the
  // gimbal moved. Supplied by Perception from the gimbal angles and the camera model.
  void set_ego_shift(double du, double dv) { ego_du_ = du; ego_dv_ = dv; }

  // Associate one detector batch. `f` (the current frame) supplies colour for the histograms.
  MotUpdate update(const DetectionBatch& b, const FrameView* f, float thr_high, float thr_low);
  // Time-based transitions (OCCLUDED -> LOST, LOST -> deleted, AMBIGUOUS expiry, EXITING).
  void tick(int64_t t);

  // Designated target with hysteresis; `locked` is the id Perception follows (-1 none).
  // While a locked track still exists (even LOST) no other track is designated: the gimbal must
  // not jump to a different phone when its own leaves the view (scenario D).
  int designate(int locked, int64_t t);

  const std::vector<MotTrack>& tracks() const { return tracks_; }
  const MotTrack* find(int id) const;
  // ids are never reused
  int next_id() const { return next_id_; }
  uint64_t id_switch_guards() const { return ambiguous_events_; }

 private:
  void predict(MotTrack& k, int64_t t) const;
  void correct(MotTrack& k, const Detection& d, int64_t t, const AppHist* h);
  double cost(const MotTrack& k, const Detection& d, const AppHist* h, bool use_iou_only) const;
  void set_state(MotTrack& k, MotState s, int64_t t);
  bool near_edge(const MotTrack& k, float* dx, float* dy) const;
  void check_separation(int64_t t);
  float priority(const MotTrack& k, int64_t t) const;

  MotCfg c_;
  int W_, H_;
  std::vector<MotTrack> tracks_;
  int next_id_ = 1;
  double ego_du_ = 0, ego_dv_ = 0;
  int designated_ = -1;
  int64_t t_designated_ = 0;
  int challenger_ = -1, challenger_n_ = 0;
  int64_t t_challenge_ = 0;
  uint64_t ambiguous_events_ = 0;
};

// Optimal assignment for a small rectangular cost matrix (rows x cols, row-major).
// Returns for each row the assigned column or -1. Pairs whose cost >= max_cost are dropped.
std::vector<int> hungarian(const std::vector<double>& cost, int rows, int cols, double max_cost);

}  // namespace rcws
