// Angle-space target estimator: pixel measurement -> LOS -> IMM (Sec 6, 7).
// In Stage 1 the gimbal is fixed, so gimbal angles are supplied as constants;
// the interface already takes them so Step 3 can plug MotorHistory in.
#pragma once
#include <functional>

#include "common/types.hpp"
#include "estimation/camera_model.hpp"
#include "estimation/imm.hpp"

namespace rcws {

struct GimbalPose {
  double psi = 0, theta = 0;
};
using GimbalAt = std::function<GimbalPose(int64_t t_ns)>;

struct EstState {
  bool valid = false;
  int64_t t_ns = 0;
  double psi = 0, psi_d = 0, psi_dd = 0;
  double theta = 0, theta_d = 0, theta_dd = 0;
  double sig_psi = 0, sig_theta = 0;
  double mu_ca = 0;
  double range_m = 0;
};

struct GateResult {
  bool pass = false;
  double d2 = 0;
};

class Estimator {
 public:
  Estimator(const Config& cfg, const CameraModel& cam, GimbalAt gimbal);

  void reset();
  bool initialized() const { return psi_.initialized(); }

  // Gate a pixel measurement against the current prediction (does not update).
  GateResult gate(const VisionMeas& m) const;
  // Initialise (if needed) and update with a measurement that passed gating.
  void update(const VisionMeas& m);
  // Range from bbox long side (Sec 3.9); only for DET measurements.
  void update_range(const VisionMeas& m);

  EstState predict(int64_t t_ns) const;
  // Predicted pixel position and 1-sigma radius [px] at t (for ROI/search windows).
  bool predict_pixel(int64_t t_ns, cv::Point2d* px, double* sigma_px) const;

 private:
  void meas_to_angles(const VisionMeas& m, double* psi, double* theta, double* Rpsi, double* Rth) const;

  const Config& cfg_;
  const CameraModel& cam_;
  GimbalAt gimbal_;
  Imm1D psi_, theta_;
  InvRangeFilter range_;
};

}  // namespace rcws
