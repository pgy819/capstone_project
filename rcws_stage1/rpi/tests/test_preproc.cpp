// v3 preprocessing (detector.preproc_i420): plane-domain downscale must give the same
// input->main mapping as the v2 RGB path, near-identical pixels, and a target centroid that
// maps back to its true main-stream position.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <opencv2/imgproc.hpp>

#include "detect/detector.hpp"
#include "test_util.hpp"

using namespace rcws;

namespace {

struct I420Img {
  int w, h;
  cv::Mat y, u, v;
  I420View view() const { return {y.data, u.data, v.data, w, h, int(y.step), int(u.step)}; }
};

// smooth colour gradients + a bright square target of known centre
I420Img make_img(int w, int h, cv::Rect2d tgt) {
  I420Img im{w, h, cv::Mat(h, w, CV_8UC1), cv::Mat(h / 2, w / 2, CV_8UC1), cv::Mat(h / 2, w / 2, CV_8UC1)};
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i) im.y.at<uint8_t>(j, i) = uint8_t(40 + 60.0 * i / w + 40.0 * j / h);
  for (int j = 0; j < h / 2; ++j)
    for (int i = 0; i < w / 2; ++i) {
      im.u.at<uint8_t>(j, i) = uint8_t(100 + 50.0 * i / (w / 2));
      im.v.at<uint8_t>(j, i) = uint8_t(150 - 40.0 * j / (h / 2));
    }
  cv::rectangle(im.y, cv::Point(int(tgt.x), int(tgt.y)), cv::Point(int(tgt.x + tgt.width) - 1, int(tgt.y + tgt.height) - 1),
                cv::Scalar(250), cv::FILLED);
  return im;
}

DetRequest v2_path(const I420Img& im, cv::Rect r, cv::Size in) {
  cv::Mat i420(r.height * 3 / 2, r.width, CV_8UC1);
  im.y(r).copyTo(i420(cv::Rect(0, 0, r.width, r.height)));
  const cv::Rect rc(r.x / 2, r.y / 2, r.width / 2, r.height / 2);
  uint8_t* pu = i420.ptr(r.height);
  uint8_t* pv = pu + (r.width / 2) * (r.height / 2);
  for (int j = 0; j < rc.height; ++j) {
    std::memcpy(pu + j * rc.width, im.u.ptr(rc.y + j) + rc.x, rc.width);
    std::memcpy(pv + j * rc.width, im.v.ptr(rc.y + j) + rc.x, rc.width);
  }
  cv::Mat rgb;
  cv::cvtColor(i420, rgb, cv::COLOR_YUV2RGB_I420);
  DetRequest q = make_request(rgb, cv::Rect(0, 0, rgb.cols, rgb.rows), in, 1.f, 0, 0, DetKind::Roi);
  q.map.crop = {float(r.x), float(r.y), float(r.width), float(r.height)};
  return q;
}

// bright-pixel centroid in the model input -> main-stream pixels
cv::Point2d centroid_main(const DetRequest& q) {
  cv::Mat g;
  cv::cvtColor(q.rgb, g, cv::COLOR_RGB2GRAY);
  const cv::Moments m = cv::moments(g > 200, true);
  const float x = float(m.m10 / m.m00) + 0.5f, y = float(m.m01 / m.m00) + 0.5f;  // pixel centres
  const Detection d = map_back(q.map, x, y, x, y, 1.f, ObjClass::Phone);
  return {d.box.x, d.box.y};
}

}  // namespace

TEST(preproc_i420_matches_v2) {
  const cv::Size in(640, 640);
  const cv::Rect2d tgt(700, 380, 40, 80);           // centre (720, 420)
  const I420Img im = make_img(1536, 864, tgt);
  for (int side : {864, 760, 642}) {                 // track ROIs larger than the input (s < 1)
    const cv::Rect r((720 - side / 2) & ~1, (432 - side / 2) & ~1, side, side);
    const cv::Rect rr = r & cv::Rect(0, 0, 1536, 864);
    const DetRequest a = v2_path(im, rr, in);
    const DetRequest b = make_request_i420(im.view(), rr, in, 1.f, 0, 0, DetKind::Roi);
    CHECK_NEAR(a.map.scale, b.map.scale, 1e-6);
    CHECK_NEAR(a.map.pad_x, b.map.pad_x, 1e-6);
    CHECK_NEAR(a.map.pad_y, b.map.pad_y, 1e-6);
    CHECK_NEAR(a.map.crop.x, b.map.crop.x, 1e-6);
    CHECK_NEAR(a.map.crop.w, b.map.crop.w, 1e-6);
    cv::Mat diff;
    cv::absdiff(a.rgb, b.rgb, diff);
    const double mean = cv::mean(diff.reshape(1))[0];
    CHECK(mean < 2.0);                               // conversion order only (rounding, chroma siting)
    const cv::Point2d ca = centroid_main(a), cb = centroid_main(b);
    CHECK_NEAR(cb.x, 720.0, 0.5);
    CHECK_NEAR(cb.y, 420.0, 0.5);
    CHECK_NEAR(ca.x, cb.x, 0.3);
    CHECK_NEAR(ca.y, cb.y, 0.3);
    std::printf("  side %d: mean|diff| %.3f, centroid v2 (%.2f, %.2f) v3 (%.2f, %.2f)\n", side, mean, ca.x, ca.y,
                cb.x, cb.y);
    // linear_mild_downscale: different filter, so pixels differ more, but geometry must hold
    const DetRequest c = make_request_i420(im.view(), rr, in, 1.f, 0, 0, DetKind::Roi, true);
    cv::absdiff(a.rgb, c.rgb, diff);
    const double mean_l = cv::mean(diff.reshape(1))[0];
    const cv::Point2d cc = centroid_main(c);
    CHECK(mean_l < 4.0);
    CHECK_NEAR(cc.x, 720.0, 0.6);
    CHECK_NEAR(cc.y, 420.0, 0.6);
    std::printf("           linear: mean|diff| %.3f, centroid (%.2f, %.2f)\n", mean_l, cc.x, cc.y);
  }
}

TEST(preproc_i420_odd_size_and_gray) {
  const cv::Size in(640, 640);
  I420Img im = make_img(1536, 864, cv::Rect2d(800, 400, 30, 60));
  // 862 x 700 -> nw = 640, nh = 520 (even); 866 x 701-ish crops exercise odd nw/nh
  for (cv::Size sz : {cv::Size(862, 700), cv::Size(902, 702), cv::Size(1000, 862)}) {
    const cv::Rect r((815 - sz.width / 2) & ~1, (430 - sz.height / 2) & ~1, sz.width, sz.height);
    const cv::Rect rr = r & cv::Rect(0, 0, 1536, 864);
    const DetRequest b = make_request_i420(im.view(), rr, in, 1.f, 0, 0, DetKind::Roi);
    CHECK(b.rgb.size() == in);
    const cv::Point2d c = centroid_main(b);
    CHECK_NEAR(c.x, 815.0, 0.6);
    CHECK_NEAR(c.y, 430.0, 0.6);
  }
  I420View g = im.view();
  g.u = g.v = nullptr;
  const DetRequest b = make_request_i420(g, cv::Rect(383, 1, 863, 863), in, 1.f, 0, 0, DetKind::Roi);
  CHECK(b.rgb.size() == in);
  const cv::Vec3b px = b.rgb.at<cv::Vec3b>(320, 320);
  CHECK(px[0] == px[1] && px[1] == px[2]);
}

TEST(preproc_i420_timing) {
  const cv::Size in(640, 640);
  const I420Img im = make_img(1536, 864, cv::Rect2d(700, 380, 40, 80));
  const cv::Rect r(336, 0, 864, 864);
  const int n = 200;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) (void)v2_path(im, r, in);
  auto t1 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) (void)make_request_i420(im.view(), r, in, 1.f, 0, 0, DetKind::Roi);
  auto t2 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) (void)make_request_i420(im.view(), r, in, 1.f, 0, 0, DetKind::Roi, true);
  auto t3 = std::chrono::steady_clock::now();
  const double a = std::chrono::duration<double, std::milli>(t1 - t0).count() / n;
  const double b = std::chrono::duration<double, std::milli>(t2 - t1).count() / n;
  const double c = std::chrono::duration<double, std::milli>(t3 - t2).count() / n;
  std::printf("  864x864 ROI -> 640x640: v2 %.3f ms, v3 %.3f ms (x%.2f), v3+linear %.3f ms (x%.2f)\n", a, b, a / b, c,
              a / c);
  CHECK(b < a);
  CHECK(c < b);
}
