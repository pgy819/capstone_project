#include "estimation/camera_model.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <vector>

namespace rcws {
namespace {

cv::Matx33d rot_rpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const cv::Matx33d Rz(cr, -sr, 0, sr, cr, 0, 0, 0, 1);   // roll about optical axis
  const cv::Matx33d Rx(1, 0, 0, 0, cp, -sp, 0, sp, cp);   // pitch about camera x
  const cv::Matx33d Ry(cy, 0, sy, 0, 1, 0, -sy, 0, cy);   // yaw about camera y
  return Ry * Rx * Rz;
}

// camera (x right, y down, z fwd) -> gimbal tilt frame (fwd, right, up)
inline cv::Vec3d cam_to_fru(const cv::Vec3d& c) { return {c[2], c[0], -c[1]}; }
inline cv::Vec3d fru_to_cam(const cv::Vec3d& g) { return {g[1], -g[2], g[0]}; }

// tilt frame -> pan frame (rotation about the 'right' axis by theta, positive up)
inline cv::Vec3d tilt_to_pan(const cv::Vec3d& g, double th) {
  const double c = std::cos(th), s = std::sin(th);
  return {g[0] * c - g[2] * s, g[1], g[0] * s + g[2] * c};
}
inline cv::Vec3d pan_to_tilt(const cv::Vec3d& p, double th) {
  const double c = std::cos(th), s = std::sin(th);
  return {p[0] * c + p[2] * s, p[1], -p[0] * s + p[2] * c};
}

}  // namespace

CameraModel::CameraModel(const IntrinsicsCfg& c) {
  K_ = (cv::Mat_<double>(3, 3) << c.fx, 0, c.cx, 0, c.fy, c.cy, 0, 0, 1);
  D_ = (cv::Mat_<double>(1, 5) << c.dist[0], c.dist[1], c.dist[2], c.dist[3], c.dist[4]);
  Rcb_ = rot_rpy(c.rcb_rpy[0], c.rcb_rpy[1], c.rcb_rpy[2]);
  d_cam_ = {c.offset_m[0], c.offset_m[1], c.offset_m[2]};
}

cv::Point2d CameraModel::undistort(double u, double v) const {
  std::vector<cv::Point2d> src{{u, v}}, dst;
  // iterative solver: tighter criteria than the default 5 iterations (sub-0.001 px round trip)
  cv::undistortPoints(src, dst, K_, D_, cv::noArray(), cv::noArray(),
                      cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 1e-12));
  return dst[0];
}

Los CameraModel::pixel_to_los(double u, double v, double psi_m, double theta_m, double range_m) const {
  const cv::Point2d n = undistort(u, v);
  cv::Vec3d ray = Rcb_ * cv::Vec3d(n.x, n.y, 1.0);
  cv::Vec3d g;
  if (range_m > 0) {
    ray *= range_m / cv::norm(ray);
    g = cam_to_fru(d_cam_ + ray);   // position relative to gimbal origin (tilt frame)
  } else {
    g = cam_to_fru(ray);            // pure direction
  }
  const cv::Vec3d p = tilt_to_pan(g, theta_m);
  Los out;
  out.psi = psi_m + std::atan2(p[1], p[0]);
  out.theta = std::atan2(p[2], std::hypot(p[0], p[1]));
  return out;
}

bool CameraModel::los_to_pixel(const Los& los, double psi_m, double theta_m, double range_m,
                               cv::Point2d* px) const {
  const double dpsi = los.psi - psi_m;
  const cv::Vec3d dir{std::cos(los.theta) * std::cos(dpsi), std::cos(los.theta) * std::sin(dpsi), std::sin(los.theta)};
  const cv::Vec3d u = pan_to_tilt(dir, theta_m);  // unit direction from the gimbal origin, tilt frame
  cv::Vec3d c;
  if (range_m > 0) {
    // range is measured from the camera (bbox size): find t with |t*u - d| = range
    const cv::Vec3d d = cam_to_fru(d_cam_);
    const double ud = u.dot(d);
    const double disc = ud * ud - d.dot(d) + range_m * range_m;
    const double t = ud + std::sqrt(std::max(0.0, disc));
    c = fru_to_cam(t * u) - d_cam_;
  } else {
    c = fru_to_cam(u);
  }
  c = Rcb_.t() * c;
  if (c[2] <= 1e-9) return false;
  std::vector<cv::Point3d> obj{{c[0] / c[2], c[1] / c[2], 1.0}};
  std::vector<cv::Point2d> img;
  cv::projectPoints(obj, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K_, D_, img);
  *px = img[0];
  return true;
}

cv::Point2d CameraModel::rad_per_px(double u, double v) const {
  const Los a = pixel_to_los(u, v, 0, 0, 0);
  const Los bx = pixel_to_los(u + 1.0, v, 0, 0, 0);
  const Los by = pixel_to_los(u, v + 1.0, 0, 0, 0);
  return {std::abs(bx.psi - a.psi), std::abs(by.theta - a.theta)};
}

}  // namespace rcws
