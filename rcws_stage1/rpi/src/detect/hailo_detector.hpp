// HailoRT async detector (Pi 5 + Hailo-8). Built only with RCWS_WITH_HAILO.
#pragma once
#include <memory>
#include <string>

#include "common/config.hpp"
#include "detect/detector.hpp"

namespace rcws {

// Returns nullptr and fills err on failure (no device, bad HEF, unexpected output layout).
std::unique_ptr<IDetector> make_hailo_detector(const DetectorCfg& cfg, std::string* err);

}  // namespace rcws
