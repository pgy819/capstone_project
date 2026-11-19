#include "perception/aim.hpp"

namespace rcws {

const char* to_string(AimMode m) {
  switch (m) {
    case AimMode::HOLD: return "HOLD";
    case AimMode::FOLLOW: return "FOLLOW";
    case AimMode::PREDICT: return "PREDICT";
  }
  return "?";
}

AimOut decide_aim(const AimCfg& c, const AimInputs& in) {
  AimOut o;
  o.t = in.t;
  o.reasons = 0;
  o.target_id = in.lock ? in.lock->id : -1;
  if (in.lock) {
    if (in.lock->st == MotState::AMBIGUOUS) o.reasons |= A_AMBIGUOUS;
    if (in.lock->st == MotState::OVERLAP) o.reasons |= A_OVERLAP;      // information only
    if (in.lock->st == MotState::EXITING) o.reasons |= A_EXITING;      // information only: keep following
    if (in.lock->st == MotState::LOST && in.lock->exited) { o.hint_dx = in.lock->exit_dx; o.hint_dy = in.lock->exit_dy; }
  }
  if (in.quality_flags) o.reasons |= A_QUALITY;

  bool move = false;
  switch (in.state) {
    case TrackState::SEARCH: o.reasons |= A_NO_TARGET; break;
    case TrackState::ACQUIRE: o.reasons |= A_ACQUIRING; break;
    case TrackState::TRACK: move = true; o.mode = AimMode::FOLLOW; break;
    case TrackState::COAST:
      if (in.since_meas_s <= c.coast_follow_s) { move = true; o.mode = AimMode::PREDICT; }
      else o.reasons |= A_COAST_LONG;
      break;
    case TrackState::REACQ: o.reasons |= A_REACQ; break;
  }
  if (move && !in.est.valid) { move = false; o.reasons |= A_NO_ESTIMATE; }
  if (move && (o.reasons & (A_QUALITY | A_AMBIGUOUS))) move = false;
  if (!move) {
    o.mode = AimMode::HOLD;
    return o;
  }
  o.psi = in.est.psi;
  o.theta = in.est.theta;
  o.psi_d = in.est.psi_d;
  o.theta_d = in.est.theta_d;
  return o;
}

}  // namespace rcws
