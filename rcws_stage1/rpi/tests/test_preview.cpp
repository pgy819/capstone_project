// Live preview "stack" view: detector-input snapshot and NCC correlation map are captured only
// when asked, belong to the batch/frame they claim, map coordinates consistently, and render.
#include <cstdio>
#include <cstdlib>
#include <opencv2/imgcodecs.hpp>

#include "app/preview_server.hpp"
#include "estimation/camera_model.hpp"
#include "perception/perception.hpp"
#include "sim/synthetic.hpp"
#include "test_util.hpp"

using namespace rcws;

namespace {

SynthTraj traj() {
  SynthTraj tr;
  tr.center = [](double t) { return cv::Point2d(768 + 200 * std::sin(1.3 * t), 432 + 80 * std::sin(0.9 * t)); };
  return tr;
}

}  // namespace

TEST(preview_capture_off_by_default) {
  Config cfg;
  SyntheticSource src(cfg.cam, traj(), 240);
  OracleDetector det(src, {640, 640}, OracleCfg{});
  CameraModel cam(cfg.intr);
  Perception per(cfg, cam, &det, nullptr, [](int64_t) { return GimbalPose{}; });
  Frame fr;
  while (src.grab(&fr, 0)) { const auto c = per.on_frame(fr.view); src.release(fr, c); }
  CHECK(per.state() == TrackState::TRACK);
  CHECK(per.det_input().rgb.empty());   // nothing retained: v3.1 behaviour
  CHECK(!per.ncc_debug().valid);
}

TEST(preview_capture_stack) {
  Config cfg;
  SyntheticSource src(cfg.cam, traj(), 360);
  OracleDetector det(src, {640, 640}, OracleCfg{});
  CameraModel cam(cfg.intr);
  Perception per(cfg, cam, &det, nullptr, [](int64_t) { return GimbalPose{}; });
  per.set_preview_capture(true);
  Frame fr;
  int checked = 0, ncc_seen = 0;
  cv::Mat last_img;
  while (src.grab(&fr, 0)) {
    const auto c = per.on_frame(fr.view);
    const DetInputSnap& in = per.det_input();
    const DetectionBatch& b = per.last_detections();
    if (!in.rgb.empty()) {
      // the snapshot is the request of the displayed batch
      CHECK(in.seq == b.frame_seq && in.kind == b.kind);
      CHECK(in.rgb.size() == cv::Size(640, 640) && in.rgb.type() == CV_8UC3);
      CHECK(*in.path != 0);
      // every returned box maps back into the model input (inverse of map_back)
      for (const auto& d : b.dets) {
        const cv::Point2f a = map_to_input(in.map, d.box.x, d.box.y);
        const cv::Point2f e = map_to_input(in.map, d.box.x + d.box.w, d.box.y + d.box.h);
        const Detection back = map_back(in.map, a.x, a.y, e.x, e.y, d.score, d.cls);
        CHECK_NEAR(back.box.x, d.box.x, 1e-2);
        CHECK_NEAR(back.box.w, d.box.w, 1e-2);
      }
      ++checked;
    }
    const NccDebug& n = per.ncc_debug();
    if (n.valid && n.frame_id == fr.view.seq) {
      CHECK(n.res.cols == n.search.cols - n.tmpl.cols + 1);
      CHECK(n.res.rows == n.search.rows - n.tmpl.rows + 1);
      double vmax;
      cv::minMaxLoc(n.res, nullptr, &vmax);
      CHECK_NEAR(vmax, n.rho, 1e-6);
      CHECK_NEAR(n.res.at<float>(n.peak), n.rho, 1e-6);
      ++ncc_seen;
    }
    if (fr.view.seq % 60 == 59) {
      PreviewOverlay ov;
      ov.state = per.state();
      if (auto t = per.target_box()) { ov.has_target = true; ov.target = *t; }
      ov.dets = b.dets;
      ov.det_crop = b.crop;
      ov.scale = float(fr.view.lo_w) / fr.view.main_w;
      ov.seq = fr.view.seq;
      ov.det_seq = b.frame_seq;
      ov.det_in = in;
      ov.ncc = per.ncc_debug();
      ov.ncc.tmpl = ov.ncc.tmpl.clone();
      const cv::Mat lo(fr.view.lo_h, fr.view.lo_w, CV_8UC1, const_cast<uint8_t*>(fr.view.lo_y), fr.view.lo_stride);
      last_img = render_preview(lo, ov, true, 0);
      const cv::Mat plain = render_preview(lo, ov, false, 0);
      CHECK(plain.size() == cv::Size(640, 360));
      CHECK(last_img.rows == 360 && last_img.cols == 640 + 360 + 281);
    }
    src.release(fr, c);
  }
  CHECK(checked > 200);
  CHECK(ncc_seen > 200);
  std::printf("  frames with detector snapshot %d, with NCC map %d\n", checked, ncc_seen);
  if (const char* p = std::getenv("RCWS_PREVIEW_PNG")) cv::imwrite(p, last_img);
}
