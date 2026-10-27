#include "camera/frame_source.hpp"

namespace rcws {
std::unique_ptr<IFrameSource> make_libcamera_source(const CameraCfg&) { return nullptr; }
}  // namespace rcws
