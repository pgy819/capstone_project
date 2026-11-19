// Aim output gate (scenarios I, J): decides every frame whether the estimate may drive the
// gimbal (FOLLOW / PREDICT) or the gimbal must stay where it is (HOLD), and records why.
// This is the value the RPi -> STM32 link (U1) will send; until then it is logged and drawn.
//
//   SEARCH, ACQUIRE                  -> HOLD   (no confirmed target)
//   TRACK                            -> FOLLOW (HOLD if the picture is bad or identity is ambiguous)
//   COAST <= coast_follow_s          -> PREDICT (bridge a short gap with the filter)
//   COAST longer, REACQ              -> HOLD   (never keep turning with the last velocity)
//   REACQ after the target left the frame: HOLD + search hint (direction it left), no other target
#pragma once
#include <cstdint>

#include "common/config.hpp"
#include "common/types.hpp"
#include "estimation/estimator.hpp"
#include "perception/mot.hpp"

namespace rcws {

enum class AimMode : uint8_t { HOLD = 0, FOLLOW = 1, PREDICT = 2 };
const char* to_string(AimMode m);

enum AimReason : uint16_t {
  A_NO_TARGET = 1, A_ACQUIRING = 2, A_COAST_LONG = 4, A_REACQ = 8, A_QUALITY = 16,
  A_AMBIGUOUS = 32, A_OVERLAP = 64, A_EXITING = 128, A_NO_ESTIMATE = 256
};

struct AimOut {
  int64_t t = 0;
  AimMode mode = AimMode::HOLD;
  uint16_t reasons = A_NO_TARGET;
  int target_id = -1;              // track board id being followed
  double psi = 0, theta = 0;       // target LOS [rad] (FOLLOW / PREDICT)
  double psi_d = 0, theta_d = 0;   // [rad/s], zero in HOLD
  float hint_dx = 0, hint_dy = 0;  // REACQ after exit: image direction the target left through
};

struct AimInputs {
  int64_t t = 0;
  TrackState state = TrackState::SEARCH;
  EstState est;
  double since_meas_s = 0;
  const MotTrack* lock = nullptr;  // nullptr: no board or lock not on it
  uint8_t quality_flags = 0;
};

AimOut decide_aim(const AimCfg& c, const AimInputs& in);

}  // namespace rcws
