// libcamera frame source for Raspberry Pi 5 + Camera Module 3 Wide (Sec 3.2, 3.3, 5.1).
// Two ISP outputs from the same request: main 1536x864 YUV420 and lores 640x360 YUV420.
// AE off (fixed exposure presets), manual focus at the hyperfocal lens position,
// frame duration pinned to the 120 fps sensor mode.
#include <sys/mman.h>

#include <condition_variable>
#include <cstring>
#include <deque>
#include <iostream>
#include <libcamera/libcamera.h>
#include <map>
#include <mutex>

#include "camera/frame_source.hpp"
#include "common/clock.hpp"

namespace rcws {
namespace {

using namespace libcamera;

struct Mapped {
  std::vector<std::pair<void*, size_t>> maps;
  const uint8_t* plane[3] = {nullptr, nullptr, nullptr};
};

class LibcameraSource final : public IFrameSource {
 public:
  explicit LibcameraSource(const CameraCfg& c) : cfg_(c) {}
  ~LibcameraSource() override { stop(); }

  bool start(std::string* err) override {
    cm_ = std::make_unique<CameraManager>();
    if (cm_->start()) { *err = "CameraManager start failed"; return false; }
    if (cm_->cameras().empty()) { *err = "no camera found"; return false; }
    cam_ = cm_->cameras()[0];
    if (cam_->acquire()) { *err = "camera acquire failed"; return false; }

    conf_ = cam_->generateConfiguration({StreamRole::VideoRecording, StreamRole::Viewfinder});
    if (!conf_ || conf_->size() != 2) { *err = "cannot create main+lores configuration"; return false; }
    StreamConfiguration& mc = conf_->at(0);
    mc.pixelFormat = formats::YUV420;
    mc.size = Size(cfg_.main_w, cfg_.main_h);
    mc.bufferCount = cfg_.buffer_count;
    StreamConfiguration& lc = conf_->at(1);
    lc.pixelFormat = formats::YUV420;
    lc.size = Size(cfg_.lores_w, cfg_.lores_h);
    lc.bufferCount = cfg_.buffer_count;
    if (cfg_.force_sensor_mode) {
      SensorConfiguration sc;
      sc.bitDepth = cfg_.sensor_bit_depth;
      sc.outputSize = Size(cfg_.sensor_w, cfg_.sensor_h);
      conf_->sensorConfig = sc;
    }
    const auto v = conf_->validate();
    if (v == CameraConfiguration::Invalid) { *err = "camera configuration invalid"; return false; }
    if (v == CameraConfiguration::Adjusted)
      std::cerr << "[cam] configuration adjusted: main " << mc.toString() << " lores " << lc.toString() << "\n";
    if (cam_->configure(conf_.get())) { *err = "camera configure failed"; return false; }
    main_ = mc.stream();
    lores_ = lc.stream();
    main_stride_ = int(mc.stride);
    lores_stride_ = int(lc.stride);
    main_size_ = mc.size;
    lores_size_ = lc.size;

    alloc_ = std::make_unique<FrameBufferAllocator>(cam_);
    for (Stream* s : {main_, lores_})
      if (alloc_->allocate(s) < 0) { *err = "buffer allocation failed"; return false; }
    const auto& mb = alloc_->buffers(main_);
    const auto& lb = alloc_->buffers(lores_);
    const size_t n = std::min(mb.size(), lb.size());
    for (size_t i = 0; i < n; ++i) {
      std::unique_ptr<Request> r = cam_->createRequest(i);
      if (!r || r->addBuffer(main_, mb[i].get()) || r->addBuffer(lores_, lb[i].get())) {
        *err = "request setup failed";
        return false;
      }
      map_buffer(mb[i].get());
      map_buffer(lb[i].get());
      requests_.push_back(std::move(r));
    }

    cam_->requestCompleted.connect(this, &LibcameraSource::on_complete);
    ControlList ctl(controls::controls);
    ctl.set(controls::AeEnable, false);
    ctl.set(controls::ExposureTime, cfg_.exposure_search_us);
    ctl.set(controls::AnalogueGain, cfg_.gain_init);
    const int64_t fd_us = 1000000 / cfg_.fps;
    ctl.set(controls::FrameDurationLimits, {fd_us, fd_us});
    if (cam_->controls().count(&controls::AfMode)) {
      ctl.set(controls::AfMode, controls::AfModeManual);   // Sec 6.1, 3.13.7
      ctl.set(controls::LensPosition, cfg_.lens_position);
    } else {
      std::cerr << "[cam] no AF control on this module\n";
    }
    if (cam_->start(&ctl)) { *err = "camera start failed"; return false; }
    started_ = true;
    for (auto& r : requests_) cam_->queueRequest(r.get());
    return true;
  }

  bool grab(Frame* f, int timeout_ms) override {
    Request* r = nullptr;
    {
      std::unique_lock<std::mutex> lk(mu_);
      if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] { return !done_.empty(); })) return false;
      r = done_.front();
      done_.pop_front();
    }
    if (r->status() == Request::RequestCancelled) return false;
    FrameBuffer* mb = r->findBuffer(main_);
    FrameBuffer* lb = r->findBuffer(lores_);
    const Mapped& mm = mapped_.at(mb);
    const Mapped& lm = mapped_.at(lb);
    const auto& md = r->metadata();

    FrameView& v = f->view;
    v = FrameView{};
    v.seq = mb->metadata().sequence;
    const int64_t boot_off = boottime_minus_monotonic_ns();
    if (auto ts = md.get(controls::SensorTimestamp))
      v.t_sof_ns = *ts - boot_off;      // CLOCK_BOOTTIME -> CLOCK_MONOTONIC (Sec 5.1 rule 2)
    else
      v.t_sof_ns = int64_t(mb->metadata().timestamp);
    if (auto e = md.get(controls::ExposureTime)) v.exposure_us = *e;
    if (auto g = md.get(controls::AnalogueGain)) v.analog_gain = *g;
    if (auto l = md.get(controls::LensPosition)) v.lens_position = *l;
    v.main_y = mm.plane[0]; v.main_u = mm.plane[1]; v.main_v = mm.plane[2];
    v.main_w = int(main_size_.width); v.main_h = int(main_size_.height);
    v.main_stride = main_stride_; v.main_uv_stride = main_stride_ / 2;
    v.lo_y = lm.plane[0]; v.lo_u = lm.plane[1]; v.lo_v = lm.plane[2];
    v.lo_w = int(lores_size_.width); v.lo_h = int(lores_size_.height);
    v.lo_stride = lores_stride_; v.lo_uv_stride = lores_stride_ / 2;
    f->dropped = (have_last_ && v.seq > last_seq_ + 1) ? uint16_t(std::min<uint32_t>(v.seq - last_seq_ - 1, 65535)) : 0;
    last_seq_ = v.seq;
    have_last_ = true;
    boot_off_ = boot_off;
    f->handle = r;
    return true;
  }

  void release(Frame& f, const FrameControls& c) override {
    auto* r = static_cast<Request*>(f.handle);
    if (!r || !started_) return;
    r->reuse(Request::ReuseBuffers);
    if (c.exposure_us > 0) r->controls().set(controls::ExposureTime, c.exposure_us);
    if (c.analog_gain > 0) r->controls().set(controls::AnalogueGain, c.analog_gain);
    cam_->queueRequest(r);
    f.handle = nullptr;
  }

  void stop() override {
    if (started_) { cam_->stop(); started_ = false; }
    if (cam_) cam_->requestCompleted.disconnect(this);
    requests_.clear();
    for (auto& [buf, m] : mapped_)
      for (auto& [p, n] : m.maps) munmap(p, n);
    mapped_.clear();
    alloc_.reset();
    if (cam_) { cam_->release(); cam_.reset(); }
    if (cm_) { cm_->stop(); cm_.reset(); }
  }

 private:
  void on_complete(Request* r) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      done_.push_back(r);
    }
    cv_.notify_one();
  }

  // Map every dmabuf once; planes of one buffer usually share an fd with offsets.
  void map_buffer(FrameBuffer* b) {
    Mapped m;
    std::map<int, size_t> len;
    for (const auto& p : b->planes()) len[p.fd.get()] = std::max(len[p.fd.get()], size_t(p.offset + p.length));
    std::map<int, uint8_t*> base;
    for (auto& [fd, n] : len) {
      void* addr = mmap(nullptr, n, PROT_READ, MAP_SHARED, fd, 0);
      if (addr == MAP_FAILED) { std::cerr << "[cam] mmap failed\n"; continue; }
      m.maps.push_back({addr, n});
      base[fd] = static_cast<uint8_t*>(addr);
    }
    for (size_t i = 0; i < b->planes().size() && i < 3; ++i) {
      const auto& p = b->planes()[i];
      if (base.count(p.fd.get())) m.plane[i] = base[p.fd.get()] + p.offset;
    }
    // single-plane YUV420 export: derive U/V positions from the stride
    if (b->planes().size() == 1 && m.plane[0]) {
      const bool is_main = (alloc_->buffers(main_).size() && std::any_of(alloc_->buffers(main_).begin(),
                            alloc_->buffers(main_).end(), [&](const auto& x) { return x.get() == b; }));
      const int stride = is_main ? main_stride_ : lores_stride_;
      const Size sz = is_main ? main_size_ : lores_size_;
      m.plane[1] = m.plane[0] + size_t(stride) * sz.height;
      m.plane[2] = m.plane[1] + size_t(stride / 2) * (sz.height / 2);
    }
    mapped_[b] = std::move(m);
  }

  CameraCfg cfg_;
  std::unique_ptr<CameraManager> cm_;
  std::shared_ptr<Camera> cam_;
  std::unique_ptr<CameraConfiguration> conf_;
  std::unique_ptr<FrameBufferAllocator> alloc_;
  Stream* main_ = nullptr;
  Stream* lores_ = nullptr;
  int main_stride_ = 0, lores_stride_ = 0;
  Size main_size_, lores_size_;
  std::vector<std::unique_ptr<Request>> requests_;
  std::map<FrameBuffer*, Mapped> mapped_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<Request*> done_;
  bool started_ = false, have_last_ = false;
  uint32_t last_seq_ = 0;
  int64_t boot_off_ = 0;
};

}  // namespace

std::unique_ptr<IFrameSource> make_libcamera_source(const CameraCfg& cfg) {
  return std::make_unique<LibcameraSource>(cfg);
}

}  // namespace rcws
