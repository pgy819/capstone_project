#include "common/types.hpp"

#include <algorithm>

namespace rcws {

float iou(const RectF& a, const RectF& b) {
  const float x0 = std::max(a.x, b.x), y0 = std::max(a.y, b.y);
  const float x1 = std::min(a.x + a.w, b.x + b.w), y1 = std::min(a.y + a.h, b.y + b.h);
  const float inter = std::max(0.f, x1 - x0) * std::max(0.f, y1 - y0);
  const float uni = a.area() + b.area() - inter;
  return uni > 0.f ? inter / uni : 0.f;
}

const char* to_string(TrackState s) {
  switch (s) {
    case TrackState::SEARCH: return "SEARCH";
    case TrackState::ACQUIRE: return "ACQUIRE";
    case TrackState::TRACK: return "TRACK";
    case TrackState::COAST: return "COAST";
    case TrackState::REACQ: return "REACQ";
  }
  return "?";
}

}  // namespace rcws
