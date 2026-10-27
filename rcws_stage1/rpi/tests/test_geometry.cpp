// Camera model: composeLOS round trips, tilt composition (B3), parallax (Sec 3.9).
#include <opencv2/core.hpp>

#include "estimation/camera_model.hpp"
#include "perception/roi.hpp"
#include "test_util.hpp"

using namespace rcws;

static IntrinsicsCfg wide() {
  IntrinsicsCfg c;
  c.fx = c.fy = 982; c.cx = 768; c.cy = 432;
  c.dist = {-0.05, 0.01, 0.0005, -0.0003, 0};
  return c;
}

TEST(centre_pixel_is_boresight) {
  CameraModel m(wide());
  const Los l = m.pixel_to_los(768, 432, 0, 0, 0);
  CHECK_NEAR(l.psi, 0, 1e-9);
  CHECK_NEAR(l.theta, 0, 1e-9);
}

TEST(pixel_los_round_trip_with_tilt_and_distortion) {
  CameraModel m(wide());
  for (double th : {0.0, 0.5, 1.0, -0.4})
    for (double u : {50.0, 400.0, 768.0, 1400.0})
      for (double v : {40.0, 432.0, 820.0}) {
        const Los l = m.pixel_to_los(u, v, 0.3, th, 0);
        cv::Point2d p;
        CHECK(m.los_to_pixel(l, 0.3, th, 0, &p));
        CHECK_NEAR(p.x, u, 1e-3);
        CHECK_NEAR(p.y, v, 1e-3);
      }
}

TEST(tilt_composition_not_additive) {
  // B3: at 60 deg tilt a horizontal offset maps to ~2x the pan increment of the flat case
  IntrinsicsCfg c = wide();
  c.dist = {0, 0, 0, 0, 0};
  CameraModel m(c);
  const double x = 0.05;  // normalised
  const double u = 768 + x * 982;
  const Los flat = m.pixel_to_los(u, 432, 0, 0, 0);
  const Los tilted = m.pixel_to_los(u, 432, 0, M_PI / 3, 0);
  CHECK_NEAR(flat.psi, std::atan2(x, 1.0), 1e-9);
  CHECK_NEAR(tilted.psi, std::atan2(x, std::cos(M_PI / 3)), 1e-9);
  CHECK(tilted.psi > 1.9 * flat.psi);
}

TEST(parallax_offset_vanishes_at_infinity_and_matters_near) {
  IntrinsicsCfg c = wide();
  c.offset_m = {0.03, 0, 0};  // optical centre 3 cm to the right of the gimbal axes
  CameraModel m(c);
  const Los far = m.pixel_to_los(768, 432, 0, 0, 1e6);
  const Los inf = m.pixel_to_los(768, 432, 0, 0, 0);
  const Los near = m.pixel_to_los(768, 432, 0, 0, 2.0);
  CHECK_NEAR(far.psi, inf.psi, 1e-6);
  CHECK_NEAR(near.psi, std::atan2(0.03, 2.0), 1e-6);  // 15 mrad (0.86 deg)
  cv::Point2d p;
  CHECK(m.los_to_pixel(near, 0, 0, 2.0, &p));
  CHECK_NEAR(p.x, 768, 1e-3);  // inverse of the parallax path
}

TEST(roi_scale_rule_and_tiles) {
  RoiCfg r;
  const cv::Rect a = track_roi(r, {768, 432}, 7, 15, 0.0, 1536, 864);
  CHECK(a.width == 96);  // clamped to s_min
  const cv::Rect b = track_roi(r, {768, 432}, 35, 74, 2.0, 1536, 864);
  CHECK_NEAR(b.width, 4 * 74 + 12, 2);
  const cv::Rect edge = track_roi(r, {5, 5}, 35, 74, 0.0, 1536, 864);
  CHECK(edge.x == 0 && edge.y == 0);
  const auto t = make_tiles(r, 1536, 864);
  CHECK(t.size() == 9);
  CHECK(t[1].x - t[0].x == 448);   // 640 - 448 = 192 px overlap (Sec 3.5)
  CHECK(t[3].y - t[0].y == 240);   // 384 - 240 = 144 px overlap
}
