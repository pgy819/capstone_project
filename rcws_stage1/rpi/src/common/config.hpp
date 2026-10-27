// Runtime configuration. Defaults follow design document V2.5 Sec 3.
// Values tagged [measure] must be replaced by calibration results (Sec 12.3).
#pragma once
#include <array>
#include <map>
#include <string>

namespace rcws {

struct CameraCfg {
  int main_w = 1536, main_h = 864;     // sensor mode 1536x864 @120 (Sec 3.2) [confirm]
  int lores_w = 640, lores_h = 360;
  int fps = 120;
  int exposure_search_us = 1000;        // SEARCH preset (blur budget, Sec 3.2)
  int exposure_track_us = 3000;         // TRACK preset
  float gain_init = 4.f;
  float gain_min = 1.f, gain_max = 16.f;
  bool roi_gain_control = true;         // Sec 3.13.5
  float gain_kappa = 0.3f;
  float roi_target_luma = 110.f;
  float lens_position = 0.81f;          // hyperfocal ~1.23 m (Sec 3.13.7) [measure]
  bool force_sensor_mode = true;
  int sensor_bit_depth = 10;
  int sensor_w = 1536, sensor_h = 864;
  int buffer_count = 4;
  // time base (Sec 5.1)
  int timestamp_sign = +1;              // sigma: +1 exposure-start, -1 readout-start [measure]
  double line_time_ns = 8320000.0 / 864.0;  // per sensor row, replace with PIXEL_RATE/HBLANK [measure]
};

struct IntrinsicsCfg {                  // main-stream pixels [measure]
  double fx = 982, fy = 982, cx = 768, cy = 432;
  std::array<double, 5> dist{0, 0, 0, 0, 0};  // k1 k2 p1 p2 k3
  std::array<double, 3> rcb_rpy{0, 0, 0};     // boresight misalignment roll/pitch/yaw [rad]
  std::array<double, 3> offset_m{0, 0, 0};    // optical centre offset d in camera frame [m]
};

struct DetectorCfg {
  std::string hef_path = "models/yolov8s_h8.hef";
  int class_phone = 67;                 // COCO 'cell phone' until custom 2-class HEF
  int class_person = 0;                 // COCO 'person'
  float score_min = 0.15f;              // raw floor; state thresholds below
  float thr_acquire = 0.60f;            // hysteresis (Sec 3.4)
  float thr_track = 0.30f;
  int max_in_flight = 2;
};

struct RoiCfg {
  float k_s = 4.f;                      // Sec 3.5
  float s_min = 96.f, s_max = 864.f;
  int tile_w = 640, tile_h = 384;
  int tiles_per_frame = 1;
  int track_det_period_hi = 4;          // frames between ROI detections when NCC is good
  int track_det_period_lo = 1;
};

struct NccCfg {
  int tmpl_min = 24, tmpl_max = 96;
  float tmpl_scale = 1.2f;
  int search_min = 8, search_max = 48;
  float rho_min = 0.6f;
  float peak_margin = 0.08f;            // best - second best
  float update_alpha = 0.1f;
  float update_iou = 0.6f, update_score = 0.5f;
  float drift_frac = 0.3f;
  double ncc_only_max_s = 0.5;
  float sigma_n_px = 0.25f, sigma_floor_px = 0.2f;
  float sigma_growth_px_per_s = 2.0f;   // sigma inflation without detector confirmation
};

struct TrackCfg {
  double coast_s = 0.5, reacq_s = 1.5;
  // SPRT acquisition (Sec 3.13.2)
  double sprt_pd = 0.8, sprt_pfa = 0.05;
  double sprt_alpha = 0.01, sprt_beta = 0.05, sprt_forget = 0.95;
  float gate_chi2 = 9.21f;
  float phone_height_m = 0.15f;
  int strong_reject_reinit = 3;         // consecutive strong detections outside the gate -> re-anchor
};

struct EstimatorCfg {
  // Field run 2 (hand-held phone, 0.4-1 m): median 34 deg/s, ~400 deg/s^2 -> much larger than first guess
  double q_cv = 5.0;                    // rad^2/s^3 white-accel PSD (CV) [tune on P4 logs]
  double q_ca = 500.0;                  // rad^2/s^5 white-jerk PSD (CA) [tune on P4 logs]
  std::array<double, 4> pi{0.97, 0.03, 0.05, 0.95};
  double sigma_det_px0 = 1.5, sigma_det_c = 0.0005;  // R model sqrt(s0^2 + c*A) [measure]
  double lead_s = 0.005;
};

struct RuntimeCfg {
  int vision_cpu = 1;
  int vision_prio = 70;                 // SCHED_FIFO; 0 = leave default
  int logger_cpu = 0;
  std::string log_path = "run.rlog";
  double clip_seconds = 0;              // RAM clip of main Y for replay (0 = off)
  std::string clip_path = "run.rclip";
  bool roi_temporal_avg = false;        // Sec 3.13.4 (not implemented yet)
};

struct Config {
  CameraCfg cam;
  IntrinsicsCfg intr;
  DetectorCfg det;
  RoiCfg roi;
  NccCfg ncc;
  TrackCfg trk;
  EstimatorCfg est;
  RuntimeCfg rt;

  // Loads "section.key = value" lines from an INI file; unknown keys are reported.
  bool load_ini(const std::string& path, std::string* err);
  std::string dump() const;  // written into the log header
};

}  // namespace rcws
