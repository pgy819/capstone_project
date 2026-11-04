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

DetRequest make_request_i420(const I420View& src, cv::Rect roi, cv::Size input, float src_to_main,
                             uint32_t seq, int64_t t_sof, DetKind kind, bool linear_mild_downscale) {
  roi.x &= ~1; roi.y &= ~1; roi.width &= ~1; roi.height &= ~1;
  roi &= cv::Rect(0, 0, src.w & ~1, src.h & ~1);
  DetRequest r;
  r.frame_seq = seq;
  r.t_sof_ns = t_sof;
  r.kind = kind;
  const float s = std::min(float(input.width) / roi.width, float(input.height) / roi.height);
  const int nw = std::max(2, int(std::lround(roi.width * s)));
  const int nh = std::max(2, int(std::lround(roi.height * s)));
  const int ew = (nw + 1) & ~1, eh = (nh + 1) & ~1;  // I420 needs even size; extra col/row replicated
  // INTER_AREA with a non-integer ratio (864 -> 640) is ~8x slower than INTER_LINEAR. Below 2x
  // decimation bilinear aliasing is small, so it is offered as an option (detector.linear_mild_downscale).
  const bool area = s < 1.f && !(linear_mild_downscale && s >= 0.5f);
  const int interp = area ? cv::INTER_AREA : cv::INTER_LINEAR;

  // vision thread only: scratch buffers reused across frames (no per-request allocation)
  thread_local cv::Mat i420, rgb;
  i420.create(eh * 3 / 2, ew, CV_8UC1);
  const cv::Mat Y(src.h, src.w, CV_8UC1, const_cast<uint8_t*>(src.y), size_t(src.y_stride));
  cv::Mat yd = i420(cv::Rect(0, 0, ew, eh));
  cv::resize(Y(roi), yd(cv::Rect(0, 0, nw, nh)), cv::Size(nw, nh), 0, 0, interp);
  if (ew != nw) yd(cv::Rect(nw - 1, 0, 1, nh)).copyTo(yd(cv::Rect(nw, 0, 1, nh)));
  if (eh != nh) yd(cv::Rect(0, nh - 1, ew, 1)).copyTo(yd(cv::Rect(0, nh, ew, 1)));

  if (src.u && src.v) {
    uint8_t* pu = i420.ptr(eh);
    uint8_t* pv = pu + size_t(ew / 2) * (eh / 2);
    cv::Mat ud(eh / 2, ew / 2, CV_8UC1, pu), vd(eh / 2, ew / 2, CV_8UC1, pv);
    const cv::Rect rc(roi.x / 2, roi.y / 2, roi.width / 2, roi.height / 2);
    const cv::Mat U(src.h / 2, src.w / 2, CV_8UC1, const_cast<uint8_t*>(src.u), size_t(src.uv_stride));
    const cv::Mat V(src.h / 2, src.w / 2, CV_8UC1, const_cast<uint8_t*>(src.v), size_t(src.uv_stride));
    cv::resize(U(rc), ud, ud.size(), 0, 0, interp);
    cv::resize(V(rc), vd, vd.size(), 0, 0, interp);
    cv::cvtColor(i420, rgb, cv::COLOR_YUV2RGB_I420);
  } else {
    cv::cvtColor(yd, rgb, cv::COLOR_GRAY2RGB);
  }

  r.rgb.create(input, CV_8UC3);  // owned by the in-flight job: fresh buffer per request
  r.rgb.setTo(cv::Scalar(114, 114, 114));
  const int px = (input.width - nw) / 2, py = (input.height - nh) / 2;
  rgb(cv::Rect(0, 0, nw, nh)).copyTo(r.rgb(cv::Rect(px, py, nw, nh)));
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
