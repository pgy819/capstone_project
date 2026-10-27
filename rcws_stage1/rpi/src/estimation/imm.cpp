#include "estimation/imm.hpp"

#include <cmath>

namespace rcws {

double wrap_pi(double a) { return std::remainder(a, 2.0 * M_PI); }

Imm1D::Imm1D(const EstimatorCfg& c, bool wrap_angle) : cfg_(c), wrap_(wrap_angle) {}

void Imm1D::model_fq(int m, double dt, cv::Matx33d* F, cv::Matx33d* Q) const {
  const double d2 = dt * dt, d3 = d2 * dt, d4 = d3 * dt, d5 = d4 * dt;
  if (m == 0) {  // CV: white acceleration, acceleration state held at zero
    *F = cv::Matx33d(1, dt, 0, 0, 1, 0, 0, 0, 0);
    const double q = cfg_.q_cv;
    *Q = cv::Matx33d(q * d3 / 3, q * d2 / 2, 0, q * d2 / 2, q * dt, 0, 0, 0, 1e-6);
  } else {       // CA: white jerk
    *F = cv::Matx33d(1, dt, d2 / 2, 0, 1, dt, 0, 0, 1);
    const double q = cfg_.q_ca;
    *Q = cv::Matx33d(q * d5 / 20, q * d4 / 8, q * d3 / 6, q * d4 / 8, q * d3 / 3, q * d2 / 2, q * d3 / 6,
                     q * d2 / 2, q * dt);
  }
}

void Imm1D::init(int64_t t_ns, double z, double R) {
  for (int m = 0; m < 2; ++m) {
    x_[m] = cv::Vec3d(z, 0, 0);
    P_[m] = cv::Matx33d(R, 0, 0, 0, 1.0, 0, 0, 0, 10.0);  // rate/accel unknown
  }
  mu_[0] = 0.8; mu_[1] = 0.2;
  t_ = t_ns;
  init_ = true;
}

void Imm1D::mix(cv::Vec3d x0[2], cv::Matx33d P0[2], double cbar[2]) const {
  const double* pi = cfg_.pi.data();  // row-major 2x2: pi[i*2+j] = P(i->j)
  for (int j = 0; j < 2; ++j) {
    cbar[j] = pi[0 * 2 + j] * mu_[0] + pi[1 * 2 + j] * mu_[1];
    double w[2];
    for (int i = 0; i < 2; ++i) w[i] = cbar[j] > 0 ? pi[i * 2 + j] * mu_[i] / cbar[j] : 0.5;
    x0[j] = w[0] * x_[0] + w[1] * x_[1];
    P0[j] = cv::Matx33d::zeros();
    for (int i = 0; i < 2; ++i) {
      const cv::Vec3d d = x_[i] - x0[j];
      P0[j] += w[i] * (P_[i] + d * d.t());
    }
  }
}

void Imm1D::predict(int64_t t_ns, cv::Vec3d* x, cv::Matx33d* P) const {
  const double dt = std::max(0.0, (t_ns - t_) * 1e-9);
  cv::Vec3d xm[2];
  cv::Matx33d Pm[2];
  cv::Vec3d x0[2];
  cv::Matx33d P0[2];
  double cbar[2];
  mix(x0, P0, cbar);
  for (int m = 0; m < 2; ++m) {
    cv::Matx33d F, Q;
    model_fq(m, dt, &F, &Q);
    xm[m] = F * x0[m];
    Pm[m] = F * P0[m] * F.t() + Q;
  }
  *x = cbar[0] * xm[0] + cbar[1] * xm[1];
  *P = cv::Matx33d::zeros();
  for (int m = 0; m < 2; ++m) {
    const cv::Vec3d d = xm[m] - *x;
    *P += cbar[m] * (Pm[m] + d * d.t());
  }
}

Innovation Imm1D::innovation(int64_t t_ns, double z, double R) const {
  cv::Vec3d x;
  cv::Matx33d P;
  predict(t_ns, &x, &P);
  Innovation in;
  in.nu = wrap_ ? wrap_pi(z - x[0]) : z - x[0];
  in.S = P(0, 0) + R;
  return in;
}

void Imm1D::update(int64_t t_ns, double z, double R) {
  const double dt = std::max(0.0, (t_ns - t_) * 1e-9);
  cv::Vec3d x0[2];
  cv::Matx33d P0[2];
  double cbar[2];
  mix(x0, P0, cbar);
  double lik[2];
  for (int m = 0; m < 2; ++m) {
    cv::Matx33d F, Q;
    model_fq(m, dt, &F, &Q);
    cv::Vec3d xp = F * x0[m];
    cv::Matx33d Pp = F * P0[m] * F.t() + Q;
    double nu = z - xp[0];
    if (wrap_) nu = wrap_pi(nu);
    const double S = Pp(0, 0) + R;
    const cv::Vec3d K(Pp(0, 0) / S, Pp(1, 0) / S, Pp(2, 0) / S);
    x_[m] = xp + K * nu;
    // Joseph form for numerical robustness
    const cv::Matx33d I = cv::Matx33d::eye();
    const cv::Matx13d H(1, 0, 0);
    const cv::Matx33d IKH = I - cv::Matx31d(K[0], K[1], K[2]) * H;
    P_[m] = IKH * Pp * IKH.t() + cv::Matx31d(K[0], K[1], K[2]) * R * cv::Matx13d(K[0], K[1], K[2]);
    lik[m] = std::exp(-0.5 * nu * nu / S) / std::sqrt(2.0 * M_PI * S);
  }
  const double norm = lik[0] * cbar[0] + lik[1] * cbar[1];
  if (norm > 1e-300) {
    mu_[0] = lik[0] * cbar[0] / norm;
    mu_[1] = lik[1] * cbar[1] / norm;
  }
  // keep both models alive
  mu_[0] = std::min(std::max(mu_[0], 1e-4), 1 - 1e-4);
  mu_[1] = 1.0 - mu_[0];
  if (wrap_) for (int m = 0; m < 2; ++m) x_[m][0] = wrap_pi(x_[m][0]);
  t_ = t_ns;
}

void InvRangeFilter::update(int64_t t_ns, double rho_meas, double var) {
  if (!init_) { rho_ = rho_meas; P_ = var; t_ = t_ns; init_ = true; return; }
  const double dt = std::max(0.0, (t_ns - t_) * 1e-9);
  P_ += q_ * dt;
  const double K = P_ / (P_ + var);
  rho_ += K * (rho_meas - rho_);
  P_ *= (1 - K);
  t_ = t_ns;
}

}  // namespace rcws
