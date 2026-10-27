// Frame source abstraction: libcamera (Pi 5), replay clip, synthetic scene.
#pragma once
#include <memory>
#include <string>

#include "common/config.hpp"
#include "common/types.hpp"

namespace rcws {

// Per-frame camera controls requested by the pipeline (applied a few frames later
// by the sensor; the applied values come back in FrameView metadata).
struct FrameControls {
  int exposure_us = 0;
  float analog_gain = 0.f;   // 0 = leave unchanged
};

struct Frame {
  FrameView view;
  void* handle = nullptr;    // source-private
  uint16_t dropped = 0;      // frames lost since the previous delivered frame
};

class IFrameSource {
 public:
  virtual ~IFrameSource() = default;
  virtual bool start(std::string* err) = 0;
  // Blocks up to timeout_ms. false on timeout or end of stream (see eos()).
  virtual bool grab(Frame* f, int timeout_ms) = 0;
  // Return the frame's buffers to the source, applying controls to the next request.
  virtual void release(Frame& f, const FrameControls& next) = 0;
  virtual void stop() = 0;
  virtual bool eos() const { return false; }
};

std::unique_ptr<IFrameSource> make_libcamera_source(const CameraCfg& cfg);  // nullptr if not built

}  // namespace rcws
