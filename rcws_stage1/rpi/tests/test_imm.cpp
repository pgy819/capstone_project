// IMM estimator: convergence on constant velocity, response to a manoeuvre, wrap at +-pi.
#include <random>

#include "estimation/imm.hpp"
#include "test_util.hpp"

using namespace rcws;

static int64_t ns(double s) { return int64_t(s * 1e9); }

// Filter mechanics are checked with low process noise (smooth target); the runtime default
// q is tuned for hand-held motion and trades velocity smoothness for responsiveness.
static EstimatorCfg smooth_cfg() {
  EstimatorCfg c;
  c.q_cv = 0.02;
  c.q_ca = 20.0;
  return c;
}

TEST(imm_tracks_constant_velocity) {
  EstimatorCfg c = smooth_cfg();
  Imm1D f(c, false);
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0, 5e-4);
  const double v = 0.5;  // rad/s (~29 deg/s)
  f.init(0, 0, 1e-6);
  for (int k = 1; k <= 240; ++k) {
    const double t = k / 120.0;
    f.update(ns(t), v * t + n(rng), 2.5e-7);
  }
  cv::Vec3d x;
  cv::Matx33d P;
  f.predict(ns(2.0), &x, &P);
  CHECK_NEAR(x[0], 1.0, 2e-3);
  CHECK_NEAR(x[1], v, 0.03);
  CHECK(f.mu_ca() < 0.5);  // CV model should dominate on a straight path
}

TEST(imm_switches_to_ca_on_manoeuvre) {
  EstimatorCfg c = smooth_cfg();
  Imm1D f(c, false);
  f.init(0, 0, 1e-6);
  double mu_max = 0;
  for (int k = 1; k <= 360; ++k) {
    const double t = k / 120.0;
    const double z = t < 1.0 ? 0.3 * t : 0.3 + 0.3 * (t - 1) + 3.0 * (t - 1) * (t - 1);  // sudden 6 rad/s^2
    f.update(ns(t), z, 2.5e-7);
    if (t > 1.0 && t < 1.5) mu_max = std::max(mu_max, f.mu_ca());
  }
  CHECK(mu_max > 0.6);
}

TEST(imm_wraps_azimuth) {
  EstimatorCfg c;
  Imm1D f(c, true);
  f.init(0, M_PI - 0.01, 1e-6);
  for (int k = 1; k <= 120; ++k) {
    const double t = k / 120.0;
    f.update(ns(t), wrap_pi(M_PI - 0.01 + 0.05 * t), 2.5e-7);  // crosses +pi -> -pi
  }
  const Innovation in = f.innovation(ns(1.0 + 1 / 120.0), wrap_pi(M_PI - 0.01 + 0.05 * (1.0 + 1 / 120.0)), 2.5e-7);
  CHECK(std::abs(in.nu) < 1e-3);  // no 2*pi jump in the innovation
}
