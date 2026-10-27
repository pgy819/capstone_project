// Asynchronous detector interface (Sec 3.4). Implementations: HailoDetector (Pi 5),
// ScriptedDetector (tests/synthetic), NullDetector.
#pragma once
#include <memory>
#include <optional>
#include <opencv2/core.hpp>

#include "common/types.hpp"

namespace rcws {

// How model-input pixels map back to main-stream pixels:
//   main = crop.xy + (in - pad) / scale * src_to_main
struct InputMap {
  RectF crop;            // crop in main-stream pixels
  float scale = 1.f;     // source px -> model input px
  float pad_x = 0, pad_y = 0;
  float src_to_main = 1.f;  // 1 for main-stream sources, main_w/lo_w for lores
};

struct DetRequest {
  uint32_t frame_seq = 0;
  int64_t t_sof_ns = 0;
  DetKind kind = DetKind::Global;
  InputMap map;
  cv::Mat rgb;           // CV_8UC3, exactly input_size()
  int64_t t_submit_ns = 0;
};

class IDetector {
 public:
  virtual ~IDetector() = default;
  virtual cv::Size input_size() const = 0;
  virtual int in_flight() const = 0;
  virtual bool submit(DetRequest&& req) = 0;          // false if it cannot accept now
  virtual std::optional<DetectionBatch> poll() = 0;   // completed results, in completion order
};

// Resize/letterbox a source region into the detector input. src is RGB (CV_8UC3).
DetRequest make_request(const cv::Mat& src_rgb, const cv::Rect& src_roi, cv::Size input, float src_to_main,
                        uint32_t seq, int64_t t_sof, DetKind kind);

// Map raw detections (model input pixels) back to main-stream coordinates.
Detection map_back(const InputMap& m, float x0, float y0, float x1, float y1, float score, ObjClass cls);

std::unique_ptr<IDetector> make_null_detector(cv::Size input);

}  // namespace rcws
