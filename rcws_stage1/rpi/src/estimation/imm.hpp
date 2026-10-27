// Per-axis IMM (CV + CA) on [angle, rate, accel] (Sec 7.1, 7.2).
// The filter state is always the posterior at the last measurement time;
// extrapolation for commands/ROI prediction never writes back (Sec 7.2).
#pragma once
#include <opencv2/core.hpp>

#include "common/config.hpp"

namespace rcws {

struct Innovation {
  double nu = 0, S = 1;  // innovation and its variance
};

class Imm1D {
 public:
  Imm1D(const EstimatorCfg& c, bool wrap_angle);

  bool initialized() const { return init_; }
  void reset() { init_ = false; }
  void init(int64_t t_ns, double z, double R);

  // Innovation of a measurement at t without changing state (for gating).
  Innovation innovation(int64_t t_ns, double z, double R) const;
  // Full IMM cycle: mix -> predict -> update -> mode probabilities.
  void update(int64_t t_ns, double z, double R);
  // Combined (moment-matched) prediction at t: x, P (no state change).
  void predict(int64_t t_ns, cv::Vec3d* x, cv::Matx33d* P) const;

  int64_t t_last() const { return t_; }
  double mu_ca() const { return mu_[1]; }

 private:
  void model_fq(int m, double dt, cv::Matx33d* F, cv::Matx33d* Q) const;
  void mix(cv::Vec3d x0[2], cv::Matx33d P0[2], double cbar[2]) const;

  EstimatorCfg cfg_;
  bool wrap_;
  bool init_ = false;
  int64_t t_ = 0;
  cv::Vec3d x_[2];
  cv::Matx33d P_[2];
  double mu_[2] = {0.5, 0.5};
};

double wrap_pi(double a);

// Inverse-range random-walk filter (Sec 7.1). rho = 1/R.
class InvRangeFilter {
 public:
  void update(int64_t t_ns, double rho_meas, double var);
  bool valid() const { return init_; }
  double range_m() const { return init_ && rho_ > 1e-3 ? 1.0 / rho_ : 0.0; }
  double rho() const { return rho_; }

 private:
  bool init_ = false;
  int64_t t_ = 0;
  double rho_ = 0, P_ = 1;
  double q_ = 0.01;  // [1/m^2 per s]
};

}  // namespace rcws
