// ROI and search-region planning (Sec 3.5, 3.13.1).
#pragma once
#include <vector>
#include <opencv2/core.hpp>

#include "common/config.hpp"
#include "common/types.hpp"

namespace rcws {

// Scale-normalised square ROI around the predicted target (Sec 3.5 eq.).
// Returned rect is clamped inside the image (shifted, not shrunk, when possible).
cv::Rect track_roi(const RoiCfg& c, cv::Point2d center, float tgt_w, float tgt_h, double sigma_px,
                   int img_w, int img_h);

// 3x3 (or as needed) overlapping tiles of tile_w x tile_h covering the image.
std::vector<cv::Rect> make_tiles(const RoiCfg& c, int img_w, int img_h);

// Region where a phone held by this person can be (Sec 3.13.1): upper body, widened.
cv::Rect person_hand_roi(const RectF& person, int img_w, int img_h);

cv::Rect clamp_shift(cv::Rect r, int img_w, int img_h);

}  // namespace rcws
