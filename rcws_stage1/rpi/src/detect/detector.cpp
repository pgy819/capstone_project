#include "detect/detector.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>

#include "common/clock.hpp"

namespace rcws {

DetRequest make_request(const cv::Mat& src_rgb, const cv::Rect& src_roi, cv::Size input, float src_to_main,
                        uint32_t seq, int64_t t_sof, DetKind kind) {
  DetRequest r;
  r.frame_seq = seq;
  r.t_sof_ns = t_sof;
  r.kind = kind;
  const cv::Rect roi = src_roi & cv::Rect(0, 0, src_rgb.cols, src_rgb.rows);
  const float s = std::min(float(input.width) / roi.width, float(input.height) / roi.height);
  const int nw = std::max(1, int(std::lround(roi.width * s)));
  const int nh = std::max(1, int(std::lround(roi.height * s)));
  r.rgb.create(input, CV_8UC3);
  r.rgb.setTo(cv::Scalar(114, 114, 114));  // YOLO letterbox grey
  const int px = (input.width - nw) / 2, py = (input.height - nh) / 2;
  // INTER_LINEAR both ways: upsampling small far targets is intended (scale normalisation, Sec 3.5)
  cv::resize(src_rgb(roi), r.rgb(cv::Rect(px, py, nw, nh)), cv::Size(nw, nh), 0, 0,
             s < 1.f ? cv::INTER_AREA : cv::INTER_LINEAR);
  r.map.crop = {roi.x * src_to_main, roi.y * src_to_main, roi.width * src_to_main, roi.height * src_to_main};
  r.map.scale = s;
  r.map.pad_x = float(px);
  r.map.pad_y = float(py);
  r.map.src_to_main = src_to_main;
  r.t_submit_ns = now_ns();
  return r;
}

Detection map_back(const InputMap& m, float x0, float y0, float x1, float y1, float score, ObjClass cls) {
  const float k = m.src_to_main / m.scale;
  Detection d;
  d.box.x = m.crop.x + (x0 - m.pad_x) * k;
  d.box.y = m.crop.y + (y0 - m.pad_y) * k;
  d.box.w = (x1 - x0) * k;
  d.box.h = (y1 - y0) * k;
  d.score = score;
  d.cls = cls;
  return d;
}

namespace {
class NullDetector final : public IDetector {
 public:
  explicit NullDetector(cv::Size s) : size_(s) {}
  cv::Size input_size() const override { return size_; }
  int in_flight() const override { return 0; }
  bool submit(DetRequest&&) override { return true; }
  std::optional<DetectionBatch> poll() override { return std::nullopt; }

 private:
  cv::Size size_;
};
}  // namespace

std::unique_ptr<IDetector> make_null_detector(cv::Size input) { return std::make_unique<NullDetector>(input); }

}  // namespace rcws
