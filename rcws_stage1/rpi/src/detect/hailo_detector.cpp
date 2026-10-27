// HailoRT InferModel async API (HailoRT 4.20+, Hailo-8 is supported by the 4.x line). Expected NMS output layout
// "by class": for each class: float count, then count x {y_min, x_min, y_max, x_max, score}
// normalised to [0,1]. The layout is verified at start-up against get_nms_shape().
#include "detect/hailo_detector.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <hailo/hailort.hpp>
#include <mutex>
#include <cstdio>
#include <thread>

#include "common/clock.hpp"

namespace rcws {
namespace {

using namespace hailort;

class HailoDetector final : public IDetector {
 public:
  ~HailoDetector() override {
    // Callbacks touch mu_/done_; drain in-flight jobs before members are destroyed.
    const auto t0 = std::chrono::steady_clock::now();
    while (in_flight_.load() > 0 && std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(500))
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (in_flight_.load() > 0)
      std::fprintf(stderr, "[hailo] %d request(s) never completed (NPU stalled?)\n", in_flight_.load());
    if (configured_) configured_->shutdown();
  }
  bool open(const DetectorCfg& cfg, std::string* err) {
    cfg_ = cfg;
    auto vdev = VDevice::create();
    if (!vdev) { *err = "VDevice::create failed: " + std::to_string(int(vdev.status())); return false; }
    vdev_ = std::move(vdev.value());
    auto model = vdev_->create_infer_model(cfg.hef_path);
    if (!model) { *err = "create_infer_model failed for " + cfg.hef_path; return false; }
    model_ = model.value();
    model_->set_batch_size(1);  // latency over throughput (Sec 3.4)
    model_->input()->set_format_type(HAILO_FORMAT_TYPE_UINT8);
    model_->output()->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);
    model_->output()->set_format_order(HAILO_FORMAT_ORDER_HAILO_NMS_BY_CLASS);  // HailoRT >= 4.20

    const auto shape = model_->input()->shape();
    in_size_ = cv::Size(int(shape.width), int(shape.height));
    if (shape.features != 3) { *err = "detector input must be RGB (3 channels)"; return false; }

    auto nms = model_->output()->get_nms_shape();
    if (!nms) { *err = "output is not an NMS output (HEF must include YOLO NMS post-process)"; return false; }
    n_classes_ = int(nms.value().number_of_classes);
    max_boxes_ = int(nms.value().max_bboxes_per_class);
    out_bytes_ = model_->output()->get_frame_size();
    const size_t expect = size_t(n_classes_) * (1 + size_t(max_boxes_) * 5) * sizeof(float);
    if (out_bytes_ < expect) {
      *err = "unexpected NMS output size " + std::to_string(out_bytes_) + " < " + std::to_string(expect) +
             " (check HailoRT version / NMS format order)";
      return false;
    }
    auto conf = model_->configure();
    if (!conf) { *err = "configure failed"; return false; }
    configured_ = std::make_unique<ConfiguredInferModel>(std::move(conf.value()));
    if (auto q = configured_->get_async_queue_size(); q && int(q.value()) < cfg_.max_in_flight) {
      std::fprintf(stderr, "[hailo] async queue size %zu < max_in_flight %d, clamping\n", q.value(), cfg_.max_in_flight);
      cfg_.max_in_flight = int(q.value());
    }
    std::fprintf(stderr, "[hailo] %s input %dx%d, %d classes, max %d boxes/class\n", cfg.hef_path.c_str(),
                 in_size_.width, in_size_.height, n_classes_, max_boxes_);
    return true;
  }

  cv::Size input_size() const override { return in_size_; }
  int in_flight() const override { return in_flight_.load(); }

  bool submit(DetRequest&& req) override {
    if (in_flight_.load() >= cfg_.max_in_flight) return false;
    auto job = std::make_shared<Job>();
    job->req = std::move(req);
    if (!job->req.rgb.isContinuous()) job->req.rgb = job->req.rgb.clone();
    job->out.resize(out_bytes_ / sizeof(float));
    auto bind = configured_->create_bindings();
    if (!bind) return false;
    auto b = std::move(bind.value());
    if (b.input()->set_buffer(MemoryView(job->req.rgb.data, job->req.rgb.total() * 3)) != HAILO_SUCCESS) return false;
    if (b.output()->set_buffer(MemoryView(job->out.data(), out_bytes_)) != HAILO_SUCCESS) return false;
    in_flight_++;
    auto run = configured_->run_async(b, [this, job](const AsyncInferCompletionInfo& info) {
      DetectionBatch out;
      out.frame_seq = job->req.frame_seq;
      out.t_sof_ns = job->req.t_sof_ns;
      out.kind = job->req.kind;
      out.crop = job->req.map.crop;
      out.t_submit_ns = job->req.t_submit_ns;
      out.t_done_ns = now_ns();
      if (info.status == HAILO_SUCCESS) parse(*job, &out);
      else if (n_err_.fetch_add(1) < 5) std::fprintf(stderr, "[hailo] inference failed, status %d\n", int(info.status));
      {
        std::lock_guard<std::mutex> lk(mu_);
        done_.push_back(std::move(out));
      }
      in_flight_--;
    });
    if (!run) {
      in_flight_--;
      if (n_err_.fetch_add(1) < 5) std::fprintf(stderr, "[hailo] run_async failed, status %d\n", int(run.status()));
      return false;
    }
    run.value().detach();
    return true;
  }

  std::optional<DetectionBatch> poll() override {
    std::lock_guard<std::mutex> lk(mu_);
    if (done_.empty()) return std::nullopt;
    DetectionBatch b = std::move(done_.front());
    done_.pop_front();
    return b;
  }

 private:
  struct Job {
    DetRequest req;
    std::vector<float> out;
  };

  void parse(const Job& job, DetectionBatch* out) const {
    const float* p = job.out.data();
    const float* end = p + job.out.size();
    for (int c = 0; c < n_classes_ && p < end; ++c) {
      const int count = std::min(int(*p++), max_boxes_);
      ObjClass cls = ObjClass::Other;
      if (c == cfg_.class_phone) cls = ObjClass::Phone;
      else if (c == cfg_.class_person) cls = ObjClass::Person;
      for (int i = 0; i < count && p + 5 <= end; ++i, p += 5) {
        const float score = p[4];
        if (cls == ObjClass::Other || score < cfg_.score_min) continue;
        const float W = float(in_size_.width), H = float(in_size_.height);
        out->dets.push_back(map_back(job.req.map, p[1] * W, p[0] * H, p[3] * W, p[2] * H, score, cls));
      }
    }
  }

  DetectorCfg cfg_;
  std::unique_ptr<VDevice> vdev_;
  std::shared_ptr<InferModel> model_;
  std::unique_ptr<ConfiguredInferModel> configured_;
  cv::Size in_size_;
  int n_classes_ = 0, max_boxes_ = 0;
  size_t out_bytes_ = 0;
  std::atomic<int> in_flight_{0};
  std::atomic<int> n_err_{0};
  mutable std::mutex mu_;
  std::deque<DetectionBatch> done_;
};

}  // namespace

std::unique_ptr<IDetector> make_hailo_detector(const DetectorCfg& cfg, std::string* err) {
  auto d = std::make_unique<HailoDetector>();
  if (!d->open(cfg, err)) return nullptr;
  return d;
}

}  // namespace rcws
