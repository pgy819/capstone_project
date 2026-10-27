// ZNCC tracker: sub-pixel accuracy on a moving synthetic target (P3 precursor).
#include <opencv2/imgproc.hpp>
#include <random>

#include "perception/zncc_tracker.hpp"
#include "test_util.hpp"

using namespace rcws;

static cv::Mat scene(double x, double y, double noise, cv::Mat* texture, std::mt19937& rng) {
  cv::Mat bg(400, 600, CV_8UC1, cv::Scalar(110));
  cv::randu(bg, 80, 140);
  cv::GaussianBlur(bg, bg, cv::Size(0, 0), 4);
  if (texture->empty()) {
    texture->create(60, 30, CV_8UC1);
    texture->setTo(30);
    (*texture)(cv::Rect(3, 6, 24, 48)).setTo(210);
    cv::circle(*texture, {10, 15}, 3, cv::Scalar(80), cv::FILLED);
    cv::circle(*texture, {20, 40}, 4, cv::Scalar(60), cv::FILLED);
  }
  cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, x, 0, 1, y);
  cv::Mat w, m, mask(texture->size(), CV_8UC1, cv::Scalar(255));
  cv::warpAffine(*texture, w, M, bg.size(), cv::INTER_LINEAR);
  cv::warpAffine(mask, m, M, bg.size(), cv::INTER_LINEAR);
  cv::Mat bf, wf, af;
  bg.convertTo(bf, CV_32F); w.convertTo(wf, CV_32F); m.convertTo(af, CV_32F, 1 / 255.0);
  bf = bf.mul(1 - af) + wf.mul(af);
  cv::Mat n(bg.size(), CV_32F);
  std::normal_distribution<float> N(0, float(noise));
  for (int j = 0; j < n.rows; ++j) for (int i = 0; i < n.cols; ++i) n.at<float>(j, i) = N(rng);
  bf += n;
  cv::Mat out;
  bf.convertTo(out, CV_8U);
  return out;
}

TEST(zncc_subpixel_accuracy) {
  NccCfg c;
  ZnccTracker t(c);
  std::mt19937 rng(3);
  cv::Mat tex;
  const double x0 = 200.0, y0 = 150.0;
  cv::Mat f0 = scene(x0, y0, 2.0, &tex, rng);
  RectF box{float(x0), float(y0), 30, 60};
  CHECK(t.init(f0, box));
  // template centre reference in frame 0
  const NccResult r0 = t.match(f0, {box.cx(), box.cy()}, 8, 0);
  CHECK(r0.ok);
  double se = 0;
  int n = 0;
  for (int k = 1; k <= 40; ++k) {
    const double dx = 0.37 * k, dy = -0.21 * k;
    cv::Mat f = scene(x0 + dx, y0 + dy, 2.0, &tex, rng);
    const NccResult r = t.match(f, {r0.u + dx + 0.8, r0.v + dy - 0.6}, 10, uint32_t(k));
    CHECK(r.ok);
    const double ex = (r.u - r0.u) - dx, ey = (r.v - r0.v) - dy;
    se += ex * ex + ey * ey;
    ++n;
  }
  const double rms = std::sqrt(se / n);
  std::printf("  ZNCC displacement RMS error %.3f px\n", rms);
  CHECK(rms < 0.15);
}

TEST(zncc_rejects_featureless_and_lost_target) {
  NccCfg c;
  ZnccTracker t(c);
  cv::Mat flat(200, 200, CV_8UC1, cv::Scalar(100));
  CHECK(!t.init(flat, RectF{50, 50, 30, 60}));
  std::mt19937 rng(5);
  cv::Mat tex;
  cv::Mat f0 = scene(200, 150, 2.0, &tex, rng);
  CHECK(t.init(f0, RectF{200, 150, 30, 60}));
  cv::Mat empty;
  cv::Mat dummy;
  cv::Mat f1 = scene(-500, -500, 2.0, &tex, rng);  // target gone
  const NccResult r = t.match(f1, {215, 180}, 12, 1);
  CHECK(!r.ok);
}
