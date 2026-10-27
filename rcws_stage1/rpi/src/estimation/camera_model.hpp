// Camera / gimbal geometry (Sec 6): pixel <-> line-of-sight angles.
// Gimbal frame axes: forward, right, up. Psi positive right, Theta positive up.
// Camera frame: x right, y down, z forward (OpenCV convention).
#pragma once
#include <array>
#include <opencv2/core.hpp>

#include "common/config.hpp"

namespace rcws {

struct Los {
  double psi = 0, theta = 0;  // absolute azimuth / elevation [rad]
};

class CameraModel {
 public:
  explicit CameraModel(const IntrinsicsCfg& c);

  // Pixel (distorted, main stream) -> normalized undistorted ray (x/z, y/z).
  cv::Point2d undistort(double u, double v) const;

  // composeLOS (Sec 6.2). range_m <= 0 means "infinite" (no parallax correction).
  Los pixel_to_los(double u, double v, double psi_m, double theta_m, double range_m) const;

  // Inverse: target LOS -> predicted pixel (used to place ROIs, Sec 3.5).
  // Returns false if the point is behind the camera.
  bool los_to_pixel(const Los& los, double psi_m, double theta_m, double range_m, cv::Point2d* px) const;

  // Local angular size of one pixel at (u,v) [rad/px] per axis (includes distortion).
  cv::Point2d rad_per_px(double u, double v) const;

  double fx() const { return K_.at<double>(0, 0); }

 private:
  cv::Mat K_, D_;
  cv::Matx33d Rcb_;      // camera->boresight-corrected camera
  cv::Vec3d d_cam_;      // optical-centre offset, camera frame [m]
};

}  // namespace rcws
