#include "app/operator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace rcws {

namespace {
int64_t ns(double s) { return int64_t(s * 1e9); }
}  // namespace

const char* to_string(OpMode m) { return m == OpMode::AUTO ? "AUTO" : "MANUAL"; }

bool Operator::set_mode(OpMode m, int64_t now) {
  if (estop_) return false;
  mode_ = m;
  homing_ = false;
  jog_until_ = 0;
  (void)now;
  return true;
}

void Operator::jog(double pan, double tilt, int64_t now) {
  if (estop_) return;
  mode_ = OpMode::MANUAL;
  homing_ = false;
  jog_pan_ = std::clamp(pan, -1.0, 1.0);
  jog_tilt_ = std::clamp(tilt, -1.0, 1.0);
  jog_until_ = (jog_pan_ == 0 && jog_tilt_ == 0) ? 0 : now + ns(c_.jog_hold_s);
}

void Operator::home(int64_t now) {
  if (estop_) return;
  mode_ = OpMode::MANUAL;
  jog_until_ = 0;
  homing_ = true;
  home_t0_ = now;
}

void Operator::estop(int64_t now) {
  (void)now;
  estop_ = true;
  mode_ = OpMode::MANUAL;
  homing_ = false;
  jog_until_ = 0;
  clear_until_ = 0;
}

void Operator::estop_clear(int64_t now) {
  if (!estop_) return;
  estop_ = false;
  mode_ = OpMode::MANUAL;   // never resume tracking straight out of an emergency stop
  clear_until_ = now + ns(c_.clear_s);
}

bool Operator::set_home(int64_t now) {
  if (!estop_) return false;
  set_home_until_ = now + ns(c_.set_home_s);
  return true;
}

const char* Operator::activity(int64_t now) const {
  if (estop_) return "ESTOP";
  if (mode_ == OpMode::AUTO) return "AUTO";
  if (jogging(now)) return "JOG";
  if (homing_) return "HOME";
  return "MANUAL";
}

LinkAim_t Operator::frame(const AimOut& aim, uint8_t seq, double lead_s, int64_t now, const LinkStatusView* st) {
  LinkAim_t f{};
  f.seq = seq;
  f.mode = LINK_MODE_HOLD;
  if (estop_) {
    f.flags = LINK_F_ESTOP_REQ | LINK_F_OPERATOR_HOLD;
    if (now < set_home_until_) f.mode = LINK_MODE_SET_HOME;
    return f;
  }
  const uint8_t clr = now < clear_until_ ? LINK_F_ESTOP_CLEAR : 0;
  if (mode_ == OpMode::AUTO) {
    f = make_link_aim(aim, seq, clr, lead_s);
    return f;
  }
  f.flags = uint8_t(LINK_F_OPERATOR_HOLD | clr);
  if (jogging(now)) {
    f.mode = LINK_MODE_JOG;
    const double cps = c_.jog_dps / 360.0 * LINK_COUNTS_PER_REV;   // deg/s -> count/s
    f.rate_pan = int16_t(std::lround(jog_pan_ * cps));
    f.rate_tilt = int16_t(std::lround(jog_tilt_ * cps));
    return f;
  }
  if (homing_) {
    const bool at_centre = st && st->valid && st->t_ns >= home_t0_ &&
                           std::abs(int(st->s.pan_goal) - int(LINK_PAN_HOME)) <= c_.home_tol_counts &&
                           std::abs(int(st->s.tilt_goal) - int(LINK_TILT_HOME)) <= c_.home_tol_counts;
    if (at_centre || now - home_t0_ > ns(c_.home_timeout_s)) {
      homing_ = false;
    } else {
      f.mode = LINK_MODE_HOME;
    }
  }
  return f;
}

}  // namespace rcws
