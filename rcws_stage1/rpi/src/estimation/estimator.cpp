#include "estimation/estimator.hpp"

#include <algorithm>
#include <cmath>

namespace rcws {

Estimator::Estimator(const Config& cfg, const CameraModel& cam, GimbalAt gimbal)
    : cfg_(cfg), cam_(cam), gimbal_(std::move(gimbal)), psi_(cfg.est, true), theta_(cfg.est, false) {}

void Estimator::reset() {
  psi_.reset();
  theta_.reset();
}

void Estimator::meas_to_angles(const VisionMeas& m, double* psi, double* theta, double* Rpsi,
                               double* Rth) const {
  const GimbalPose g = gimbal_(m.t_cap_ns);
  const Los los = cam_.pixel_to_los(m.u, m.v, g.psi, g.theta, range_.range_m());
  *psi = los.psi;
  *theta = los.theta;
  const cv::Point2d rpp = cam_.rad_per_px(m.u, m.v);
  const double s = std::max<double>(m.sigma_px, 0.05);
  *Rpsi = (s * rpp.x) * (s * rpp.x);
  *Rth = (s * rpp.y) * (s * rpp.y);
}

GateResult Estimator::gate(const VisionMeas& m) const {
  GateResult r;
  if (!initialized()) { r.pass = true; return r; }
  double zp, zt, Rp, Rt;
  meas_to_angles(m, &zp, &zt, &Rp, &Rt);
  // gate against the same bounded prediction used for ROI placement (not the raw CA extrapolation)
  const EstState s = predict(m.t_cap_ns);
  const double np = wrap_pi(zp - s.psi), nt = zt - s.theta;
  r.d2 = np * np / (s.sig_psi * s.sig_psi + Rp) + nt * nt / (s.sig_theta * s.sig_theta + Rt);
  r.pass = r.d2 <= cfg_.trk.gate_chi2;
  return r;
}

void Estimator::update(const VisionMeas& m) {
  double zp, zt, Rp, Rt;
  meas_to_angles(m, &zp, &zt, &Rp, &Rt);
  if (!initialized()) {
    psi_.init(m.t_cap_ns, zp, Rp);
    theta_.init(m.t_cap_ns, zt, Rt);
    return;
  }
  if (m.t_cap_ns < psi_.t_last()) return;  // never update out of sequence (Sec 3.7)
  if ((m.t_cap_ns - psi_.t_last()) * 1e-9 > 0.25) {
    // long gap: the old rate/accel is no longer informative -> restart from this measurement
    psi_.init(m.t_cap_ns, zp, Rp);
    theta_.init(m.t_cap_ns, zt, Rt);
    return;
  }
  psi_.update(m.t_cap_ns, zp, Rp);
  theta_.update(m.t_cap_ns, zt, Rt);
}

void Estimator::update_range(const VisionMeas& m) {
  const double side = std::max(m.w, m.h);
  if (side < 4.0) return;
  const double rho = side / (cam_.fx() * cfg_.trk.phone_height_m);
  // tilt of the phone shortens the long side -> rho biased low; inflate variance
  const double var = std::pow(0.2 * rho + 0.5 / (cam_.fx() * cfg_.trk.phone_height_m), 2);
  range_.update(m.t_cap_ns, rho, var);
}

EstState Estimator::predict(int64_t t_ns) const {
  EstState s;
  if (!initialized()) return s;
  cv::Vec3d xp, xt;
  cv::Matx33d Pp, Pt;
  // Beyond kHorizon past the last measurement the CA model would integrate a stale
  // acceleration without bound (field data: estimates ran to -400 deg while coasting).
  // Predict normally up to the horizon, then let the velocity decay (Sec 3.8 COAST).
  constexpr double kHorizon = 0.05, kTau = 0.3;
  const double ext = (t_ns - psi_.t_last()) * 1e-9;
  const int64_t t_model = ext > kHorizon ? psi_.t_last() + int64_t(kHorizon * 1e9) : t_ns;
  psi_.predict(t_model, &xp, &Pp);
  theta_.predict(t_model, &xt, &Pt);
  if (ext > kHorizon) {
    const double tau = ext - kHorizon, g = kTau * (1.0 - std::exp(-tau / kTau)), d = std::exp(-tau / kTau);
    for (auto* x : {&xp, &xt}) { (*x)[0] += (*x)[1] * g; (*x)[1] *= d; (*x)[2] = 0; }
    cv::Vec3d dummy;
    cv::Matx33d Pl;
    psi_.predict(t_ns, &dummy, &Pl);  Pp = Pl;   // uncertainty keeps growing
    theta_.predict(t_ns, &dummy, &Pl); Pt = Pl;
  }
  s.valid = true;
  s.t_ns = t_ns;
  s.psi = wrap_pi(xp[0]); s.psi_d = xp[1]; s.psi_dd = xp[2];
  s.theta = std::clamp(xt[0], -M_PI / 2, M_PI / 2); s.theta_d = xt[1]; s.theta_dd = xt[2];
  s.sig_psi = std::sqrt(std::max(0.0, Pp(0, 0)));
  s.sig_theta = std::sqrt(std::max(0.0, Pt(0, 0)));
  s.mu_ca = 0.5 * (psi_.mu_ca() + theta_.mu_ca());
  s.range_m = range_.range_m();
  return s;
}

bool Estimator::predict_pixel(int64_t t_ns, cv::Point2d* px, double* sigma_px) const {
  const EstState s = predict(t_ns);
  if (!s.valid) return false;
  const GimbalPose g = gimbal_(t_ns);
  if (!cam_.los_to_pixel({s.psi, s.theta}, g.psi, g.theta, s.range_m, px)) return false;
  const cv::Point2d rpp = cam_.rad_per_px(px->x, px->y);
  const double sx = s.sig_psi / std::max(rpp.x, 1e-6);
  const double sy = s.sig_theta / std::max(rpp.y, 1e-6);
  *sigma_px = std::sqrt(0.5 * (sx * sx + sy * sy));
  return true;
}

}  // namespace rcws
