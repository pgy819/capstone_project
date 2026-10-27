// RAM clip recorder and replay (Sec 12.4).
// Full-resolution frames at 120 fps (~280 MB/s) exceed SD-card bandwidth, so the
// recorder keeps the last N seconds in a preallocated RAM ring and writes it out
// on exit. ReplaySource feeds a clip back through the same pipeline; ReplayDetector
// re-injects the detections recorded in the matching .rlog at the frame where they
// were originally consumed, so detector-independent changes (NCC, IMM, SPRT, timers)
// can be A/B compared deterministically.
#pragma once
#include <cstdio>
#include <deque>
#include <functional>
#include <optional>
#include <map>
#include <string>
#include <vector>

#include "camera/frame_source.hpp"
#include "detect/detector.hpp"

namespace rcws {

class ClipRecorder {
 public:
  bool init(const FrameView& first, double seconds, int fps);
  bool active() const { return !ring_.empty(); }
  void push(const FrameView& f);
  bool save(const std::string& path) const;

 private:
  struct Slot {
    uint32_t seq = 0;
    int64_t t_sof = 0;
    int32_t exp = 0;
    float gain = 0, lens = 0;
    std::vector<uint8_t> data;
  };
  int mw_ = 0, mh_ = 0, lw_ = 0, lh_ = 0;
  size_t frame_bytes_ = 0;
  std::vector<Slot> ring_;
  size_t head_ = 0, count_ = 0;
};

class ReplaySource final : public IFrameSource {
 public:
  explicit ReplaySource(std::string path) : path_(std::move(path)) {}
  ~ReplaySource() override { stop(); }
  bool start(std::string* err) override;
  bool grab(Frame* f, int timeout_ms) override;
  void release(Frame&, const FrameControls&) override {}
  void stop() override;
  bool eos() const override { return eos_; }
  uint32_t current_seq() const { return cur_seq_; }

 private:
  std::string path_;
  std::FILE* f_ = nullptr;
  int mw_ = 0, mh_ = 0, lw_ = 0, lh_ = 0;
  uint32_t n_ = 0, read_ = 0, cur_seq_ = 0;
  bool eos_ = false;
  std::vector<uint8_t> buf_;
};

class ReplayDetector final : public IDetector {
 public:
  // current_seq: frame being processed (the pipeline polls at its start)
  ReplayDetector(const std::string& rlog_path, cv::Size input, std::function<uint32_t()> current_seq);
  bool ok() const { return ok_; }
  cv::Size input_size() const override { return in_; }
  int in_flight() const override { return 0; }
  bool submit(DetRequest&&) override { return true; }  // recorded results are used instead
  std::optional<DetectionBatch> poll() override;

 private:
  bool ok_ = false;
  cv::Size in_;
  std::function<uint32_t()> cur_;
  std::multimap<uint32_t, DetectionBatch> by_consumed_;
};

}  // namespace rcws
