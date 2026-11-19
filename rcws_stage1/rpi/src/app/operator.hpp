// Operator layer between the perception aim output and the STM32 link (web page commands).
//
//   AUTO    the aim gate drives the gimbal (FOLLOW / PREDICT / HOLD frames)
//   MANUAL  the gimbal holds (OPERATOR_HOLD); jog and home are allowed
//   ESTOP   software emergency stop: every frame carries ESTOP_REQ (STM32: torque off, latched);
//           released only by an explicit estop_clear, which leaves the operator in MANUAL
//
// Jog is dead-man: each web jog message keeps the motion alive for jog_hold_s; a lost
// connection or a released button stops it. Home runs until the STM32 reports both goals at the
// centre (or a timeout) and then holds. Pure logic on the caller's clock: unit-tested.
#pragma once
#include <cstdint>
#include <string>

#include "link/uart_link.hpp"
#include "perception/aim.hpp"

namespace rcws {

enum class OpMode : uint8_t { AUTO = 0, MANUAL = 1 };
const char* to_string(OpMode m);

struct OperatorCfg {
  double jog_dps = 20.0;      // jog rate at full deflection
  double jog_hold_s = 0.30;   // dead-man: motion stops this long after the last jog message
  double home_timeout_s = 15.0;
  double clear_s = 0.5;       // ESTOP_CLEAR is repeated this long (a frame may be lost)
  int home_tol_counts = 4;
};

class Operator {
 public:
  explicit Operator(OperatorCfg c = {}) : c_(c) {}

  bool set_mode(OpMode m, int64_t now);           // false while ESTOP (clear it first)
  void jog(double pan, double tilt, int64_t now);  // -1..1 each; switches to MANUAL
  void home(int64_t now);                          // switches to MANUAL
  void estop(int64_t now);
  void estop_clear(int64_t now);

  // Frame for this camera frame. `st` = latest STM32 status (nullptr if none).
  LinkAim_t frame(const AimOut& aim, uint8_t seq, double lead_s, int64_t now, const LinkStatusView* st);

  OpMode mode() const { return mode_; }
  bool estopped() const { return estop_; }
  bool jogging(int64_t now) const { return now < jog_until_; }
  bool homing() const { return homing_; }
  const char* activity(int64_t now) const;   // "AUTO", "MANUAL", "JOG", "HOME", "ESTOP"

 private:
  OperatorCfg c_;
  OpMode mode_ = OpMode::AUTO;
  bool estop_ = false;
  int64_t clear_until_ = 0;
  int64_t jog_until_ = 0;
  double jog_pan_ = 0, jog_tilt_ = 0;
  bool homing_ = false;
  int64_t home_t0_ = 0;
};

}  // namespace rcws
