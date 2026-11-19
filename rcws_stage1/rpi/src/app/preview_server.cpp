#include "app/preview_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#ifdef RCWS_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

#include "app/web_util.hpp"
#include "common/clock.hpp"
#include "common/rt.hpp"

namespace rcws {
namespace {

bool send_all(int fd, const void* p, size_t n) {
  const char* c = static_cast<const char*>(p);
  while (n > 0) {
    const ssize_t k = ::send(fd, c, n, MSG_NOSIGNAL);
    if (k <= 0) return false;
    c += k;
    n -= size_t(k);
  }
  return true;
}

cv::Scalar state_color(TrackState s) {
  switch (s) {
    case TrackState::TRACK: return {0, 220, 0};
    case TrackState::ACQUIRE: return {0, 220, 255};
    case TrackState::COAST: return {0, 140, 255};
    case TrackState::REACQ: return {255, 120, 0};
    default: return {200, 200, 200};
  }
}

cv::Scalar mot_color(MotState s) {
  switch (s) {
    case MotState::TENTATIVE: return {160, 160, 160};
    case MotState::CONFIRMED: return {80, 200, 80};
    case MotState::OVERLAP: return {0, 200, 255};
    case MotState::OCCLUDED: return {0, 140, 255};
    case MotState::EXITING: return {255, 200, 0};
    case MotState::AMBIGUOUS: return {0, 0, 255};
    case MotState::LOST: return {120, 120, 120};
  }
  return {200, 200, 200};
}

std::string aim_text(AimMode m, uint16_t reasons, uint8_t q) {
  static const char* names[] = {"no target", "acquiring", "coast too long", "reacquiring", "image quality",
                                "ambiguous identity", "overlap", "exiting", "no estimate"};
  std::string t = std::string("AIM ") + to_string(m);
  std::string r;
  for (int b = 0; b < 9; ++b)
    if (reasons & (1u << b)) r += (r.empty() ? "" : ", ") + std::string(names[b]);
  if (!r.empty()) t += "  (" + r + ")";
  if (q) {
    t += "   IMAGE:";
    if (q & 1) t += " DARK";
    if (q & 2) t += " GLARE";
    if (q & 4) t += " BLUR";
  }
  return t;
}

cv::Rect to_rect(const RectF& r, float s) {
  return cv::Rect(int(r.x * s), int(r.y * s), std::max(1, int(r.w * s)), std::max(1, int(r.h * s)));
}

}  // namespace

bool PreviewServer::start(int port, bool window, int cpu, std::string* err) {
#ifndef RCWS_HAVE_HIGHGUI
  if (window) { *err = "built without OpenCV highgui: --window unavailable"; return false; }
#endif
  window_ = window;
  if (port > 0) {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) { *err = "socket failed"; return false; }
    int one = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(uint16_t(port));
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0) {
      *err = "bind to port " + std::to_string(port) + " failed: " + std::strerror(errno);
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    ::listen(listen_fd_, 4);
  }
  stop_ = false;
  th_ = std::thread([this, cpu] { run(cpu); });
  return true;
}

void PreviewServer::stop() {
  stop_ = true;
  if (th_.joinable()) th_.join();
  if (listen_fd_ >= 0) ::close(listen_fd_);
  listen_fd_ = -1;
}

void PreviewServer::publish(const cv::Mat& gray, const PreviewOverlay& ov) {
  WebFrame f;
  gray.copyTo(f.y);
  f.scale = ov.scale;
  publish_frame(std::move(f), ov);
}

void PreviewServer::publish_frame(WebFrame&& f, const PreviewOverlay& ov) {
  std::unique_lock<std::mutex> lk(mu_, std::try_to_lock);
  if (!lk.owns_lock()) return;
  pending_ = std::move(f);
  pending_ov_ = ov;
  has_pending_ = true;
}

void PreviewServer::set_status(std::string json) {
  std::lock_guard<std::mutex> lk(mu_);
  status_ = std::move(json);
  status_dirty_ = true;
}

void PreviewServer::push_event(std::string json) {
  std::lock_guard<std::mutex> lk(mu_);
  if (events_.size() < 256) events_.push_back(std::move(json));
}

void PreviewServer::push_capture(WebCapture&& c) {
  std::lock_guard<std::mutex> lk(mu_);
  if (captures_in_.size() < 8) captures_in_.push_back(std::move(c));
}

void PreviewServer::mark_answered(int track_id) {
  std::lock_guard<std::mutex> lk(mu_);
  if (answered_in_.size() < 64) answered_in_.push_back(track_id);
}

std::vector<std::string> PreviewServer::take_commands() {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<std::string> out;
  out.swap(commands_);
  return out;
}

WebFrame copy_web_frame(const FrameView& f) {
  WebFrame w;
  if (f.lo_y) {
    cv::Mat(f.lo_h, f.lo_w, CV_8UC1, const_cast<uint8_t*>(f.lo_y), f.lo_stride).copyTo(w.y);
    if (f.lo_u && f.lo_v) {
      cv::Mat(f.lo_h / 2, f.lo_w / 2, CV_8UC1, const_cast<uint8_t*>(f.lo_u), f.lo_uv_stride).copyTo(w.u);
      cv::Mat(f.lo_h / 2, f.lo_w / 2, CV_8UC1, const_cast<uint8_t*>(f.lo_v), f.lo_uv_stride).copyTo(w.v);
    }
    w.scale = float(f.lo_w) / float(f.main_w);
  } else {
    const cv::Mat y(f.main_h, f.main_w, CV_8UC1, const_cast<uint8_t*>(f.main_y), f.main_stride);
    cv::resize(y, w.y, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    w.scale = 0.5f;
  }
  return w;
}

cv::Mat web_frame_bgr(const WebFrame& f) {
  cv::Mat bgr;
  if (f.y.empty()) return bgr;
  if (f.u.empty() || f.v.empty() || f.y.cols % 2 || f.y.rows % 2) {
    cv::cvtColor(f.y, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  cv::Mat i420(f.y.rows * 3 / 2, f.y.cols, CV_8UC1);
  f.y.copyTo(i420(cv::Rect(0, 0, f.y.cols, f.y.rows)));
  const size_t q = size_t(f.y.cols / 2) * size_t(f.y.rows / 2);
  uint8_t* pu = i420.ptr(f.y.rows);
  for (int j = 0; j < f.u.rows; ++j) std::memcpy(pu + size_t(j) * f.u.cols, f.u.ptr(j), size_t(f.u.cols));
  for (int j = 0; j < f.v.rows; ++j) std::memcpy(pu + q + size_t(j) * f.v.cols, f.v.ptr(j), size_t(f.v.cols));
  cv::cvtColor(i420, bgr, cv::COLOR_YUV2BGR_I420);
  return bgr;
}

namespace {

cv::Mat render_frame(const cv::Mat& src, const PreviewOverlay& ov, int clients) {
  cv::Mat img;
  if (src.channels() == 3) img = src.clone();
  else cv::cvtColor(src, img, cv::COLOR_GRAY2BGR);
  const float s = ov.scale;
  // crosshair = camera boresight (image centre)
  const cv::Point c(img.cols / 2, img.rows / 2);
  cv::line(img, c - cv::Point(12, 0), c + cv::Point(12, 0), {255, 255, 255}, 1);
  cv::line(img, c - cv::Point(0, 12), c + cv::Point(0, 12), {255, 255, 255}, 1);
  // region the latest detection request covered (ROI / tile / global)
  if (ov.det_crop.w > 0) cv::rectangle(img, to_rect(ov.det_crop, s), {90, 90, 90}, 1);
  // raw detections: phone = yellow, person = blue
  for (const auto& d : ov.dets) {
    const cv::Scalar col = d.cls == ObjClass::Phone ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 160, 0);
    const cv::Rect r = to_rect(d.box, s);
    cv::rectangle(img, r, col, 1);
    char t[32];
    std::snprintf(t, sizeof t, "%s %.2f", d.cls == ObjClass::Phone ? "phone" : "person", d.score);
    cv::putText(img, t, r.tl() + cv::Point(0, -3), cv::FONT_HERSHEY_SIMPLEX, 0.35, col, 1);
  }
  // v4: multi-target board (scenarios A-D). LOST tracks are not drawn; lock = magenta double box
  for (const auto& k : ov.tracks) {
    if (k.st == MotState::LOST) continue;
    const cv::Scalar col = k.lock ? cv::Scalar(255, 0, 255) : mot_color(k.st);
    const cv::Rect r = to_rect(k.box, s);
    cv::rectangle(img, r - cv::Point(3, 3) + cv::Size(6, 6), col, k.lock ? 2 : 1);
    char t[48];
    std::snprintf(t, sizeof t, "#%d %s %.2f%s%s", k.id, to_string(k.st), k.prio, k.lock ? " LOCK" : "",
                  !k.lock && k.designated ? " *" : "");
    cv::putText(img, t, r.tl() + cv::Point(-3, r.height + 16), cv::FONT_HERSHEY_SIMPLEX, 0.38, col, 1);
  }
  // the lock left the view: arrow at the edge it left through (bounded search direction, scenario D)
  if (ov.hint_dx != 0 || ov.hint_dy != 0) {
    const cv::Point a(int(img.cols / 2 + ov.hint_dx * (img.cols / 2 - 30)), int(img.rows / 2 + ov.hint_dy * (img.rows / 2 - 30)));
    cv::arrowedLine(img, a - cv::Point(int(ov.hint_dx * 40), int(ov.hint_dy * 40)), a, {255, 0, 255}, 3, cv::LINE_8, 0, 0.4);
  }
  // tracked target
  if (ov.has_target) {
    const cv::Scalar col = state_color(ov.state);
    const cv::Rect r = to_rect(ov.target, s);
    cv::rectangle(img, r, col, 2);
    cv::line(img, c, (r.tl() + r.br()) / 2, col, 1);
  }
  // status bar
  cv::rectangle(img, cv::Rect(0, 0, img.cols, 50), {0, 0, 0}, cv::FILLED);
  char l1[160], l2[160];
  std::snprintf(l1, sizeof l1, "%-7s  #%u  %.1f fps  proc %.2f ms  clients %d", to_string(ov.state), ov.seq, ov.fps,
                ov.proc_ms, clients);
  if (ov.est_valid)
    std::snprintf(l2, sizeof l2, "psi %+7.2f  theta %+7.2f deg   rate %+6.1f %+6.1f deg/s   R %.1f m", ov.psi_deg,
                  ov.theta_deg, ov.rate_psi, ov.rate_theta, ov.range_m);
  else
    std::snprintf(l2, sizeof l2, "no target estimate");
  cv::putText(img, l1, {6, 14}, cv::FONT_HERSHEY_SIMPLEX, 0.42, state_color(ov.state), 1);
  cv::putText(img, l2, {6, 29}, cv::FONT_HERSHEY_SIMPLEX, 0.42, {255, 255, 255}, 1);
  const cv::Scalar ac = ov.aim_mode == AimMode::HOLD ? cv::Scalar(0, 160, 255) : cv::Scalar(0, 230, 0);
  cv::putText(img, aim_text(ov.aim_mode, ov.aim_reasons, ov.qflags), {6, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.42, ac, 1);
  return img;
}

void title(cv::Mat& p, const std::string& t, cv::Scalar col = {230, 230, 230}) {
  cv::rectangle(p, cv::Rect(0, 0, p.cols, 18), {0, 0, 0}, cv::FILLED);
  cv::putText(p, t, {5, 13}, cv::FONT_HERSHEY_SIMPLEX, 0.38, col, 1);
}

void note(cv::Mat& p, const std::string& t, int y) {
  cv::putText(p, t, {6, y}, cv::FONT_HERSHEY_SIMPLEX, 0.38, {170, 170, 170}, 1);
}

// Fit src into dst (letterboxed, centred). Returns the scale used and the top-left offset.
double fit_into(const cv::Mat& src, cv::Mat dst, int interp, cv::Point* off) {
  const double k = std::min(double(dst.cols) / src.cols, double(dst.rows) / src.rows);
  const int w = std::max(1, int(src.cols * k)), h = std::max(1, int(src.rows * k));
  *off = {(dst.cols - w) / 2, (dst.rows - h) / 2};
  cv::resize(src, dst(cv::Rect(*off, cv::Size(w, h))), cv::Size(w, h), 0, 0, interp);
  return k;
}

const char* kind_name(DetKind k) {
  return k == DetKind::Roi ? "ROI" : k == DetKind::Tile ? "TILE" : "GLOBAL";
}

// A: what the detector saw, with the returned boxes drawn in model-input pixels.
cv::Mat det_panel(const PreviewOverlay& ov, int H) {
  cv::Mat p(H, H, CV_8UC3, cv::Scalar(28, 28, 28));
  const DetInputSnap& d = ov.det_in;
  if (d.rgb.empty()) { title(p, "DETECTOR INPUT"); note(p, "no batch yet", 40); return p; }
  cv::Mat in;
  cv::cvtColor(d.rgb, in, cv::COLOR_RGB2BGR);
  for (const auto& det : ov.dets) {   // dets belong to the same batch (ov.det_seq == d.seq)
    if (ov.det_seq != d.seq) break;
    const cv::Point2f a = map_to_input(d.map, det.box.x, det.box.y);
    const cv::Point2f b = map_to_input(d.map, det.box.x + det.box.w, det.box.y + det.box.h);
    const cv::Scalar col = det.cls == ObjClass::Phone ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 160, 0);
    cv::rectangle(in, a, b, col, 2);
    char t[24];
    std::snprintf(t, sizeof t, "%.2f", det.score);
    cv::putText(in, t, cv::Point(int(a.x), std::max(14, int(a.y) - 4)), cv::FONT_HERSHEY_SIMPLEX, 0.6, col, 2);
  }
  if (ov.has_target) {  // tracker's box for reference, in the same pixels
    const cv::Point2f a = map_to_input(d.map, ov.target.x, ov.target.y);
    const cv::Point2f b = map_to_input(d.map, ov.target.x + ov.target.w, ov.target.y + ov.target.h);
    cv::rectangle(in, a, b, state_color(ov.state), 1);
  }
  cv::Point off;
  fit_into(in, p(cv::Rect(0, 18, H, H - 18)), cv::INTER_AREA, &off);
  // crop size in source pixels and the resampling factor seen by the model
  const float src_px = d.map.crop.w / d.map.src_to_main;
  char t[96];
  std::snprintf(t, sizeof t, "DET IN %s #%u  %s %.0fpx x%.2f", kind_name(d.kind), d.seq, d.path, src_px, d.map.scale);
  title(p, t, d.seq == ov.det_seq ? cv::Scalar(0, 255, 255) : cv::Scalar(150, 150, 150));
  return p;
}

cv::Mat gray8(const cv::Mat& f32) {  // luma stored as float 0..255
  cv::Mat g;
  f32.convertTo(g, CV_8U);
  cv::cvtColor(g, g, cv::COLOR_GRAY2BGR);
  return g;
}

// C: template, search window with the matched position, and the ZNCC map.
cv::Mat ncc_panel(const PreviewOverlay& ov, int H) {
  const int W = int(std::lround(0.78 * H));
  cv::Mat p(H, W, CV_8UC3, cv::Scalar(28, 28, 28));
  const NccDebug& n = ov.ncc;
  const bool stale = ov.seq - n.frame_id > 2;
  if (!n.valid) {
    title(p, "NCC");
    note(p, n.frame_id == 0 ? "no template yet" : "no map (window off image)", 40);
    note(p, "runs in TRACK / COAST only", 58);
    return p;
  }
  char t[96];
  std::snprintf(t, sizeof t, "NCC #%u %s rho %.2f 2nd %.2f s%d%s", n.frame_id, n.ok ? "OK" : "FAIL", n.rho,
                n.second, n.scale, stale ? " stale" : "");
  title(p, t, stale ? cv::Scalar(150, 150, 150) : n.ok ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 90, 255));

  // row 1: template | search window
  const int box = (W - 15) / 2, y1 = 22;
  cv::Point o1, o2;
  const cv::Mat tb = p(cv::Rect(5, y1, box, box)), sb = p(cv::Rect(10 + box, y1, box, box));
  fit_into(gray8(n.tmpl), tb, cv::INTER_NEAREST, &o1);
  const double ks = fit_into(gray8(n.search), sb, cv::INTER_NEAREST, &o2);
  const cv::Rect m(int(n.peak.x * ks), int(n.peak.y * ks), int(n.tmpl.cols * ks), int(n.tmpl.rows * ks));
  cv::Mat sbv = sb;
  cv::rectangle(sbv, m + o2, n.ok ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 90, 255), 1);
  note(p, "template", y1 + box + 12);
  cv::putText(p, "search", {15 + box, y1 + box + 12}, cv::FONT_HERSHEY_SIMPLEX, 0.38, {170, 170, 170}, 1);

  // row 2: correlation map, -1 (blue) .. +1 (red), peak white, second peak magenta
  const int y2 = y1 + box + 18, hh = H - y2 - 20;
  if (hh > 20 && !n.res.empty()) {
    cv::Mat u8, heat;
    n.res.convertTo(u8, CV_8U, 127.5, 127.5);
    cv::applyColorMap(u8, heat, cv::COLORMAP_JET);
    cv::Point o3;
    const cv::Mat hb = p(cv::Rect(5, y2, W - 10, hh));
    const double kh = fit_into(heat, hb, cv::INTER_NEAREST, &o3);
    auto at = [&](cv::Point q) { return cv::Point(int((q.x + 0.5) * kh), int((q.y + 0.5) * kh)) + o3; };
    cv::Mat hbv = hb;
    cv::drawMarker(hbv, at(n.peak), {255, 255, 255}, cv::MARKER_CROSS, 12, 2);
    if (n.peak2.x >= 0) cv::drawMarker(hbv, at(n.peak2), {255, 0, 255}, cv::MARKER_TILTED_CROSS, 10, 2);
    char l[96];
    if (n.second > -1.5f)
      std::snprintf(l, sizeof l, "need rho>=%.2f margin>=%.2f (now %.2f)", n.rho_min, n.margin_min, n.rho - n.second);
    else
      std::snprintf(l, sizeof l, "need rho>=%.2f  (blue -1 .. red +1)", n.rho_min);
    note(p, l, H - 6);
  }
  if (stale) {  // dim: NCC did not run on recent frames
    cv::Mat r = p(cv::Rect(0, 18, W, H - 18));
    r *= 0.45;
  }
  return p;
}

}  // namespace

cv::Mat render_preview(const cv::Mat& img, const PreviewOverlay& ov, bool stack, int clients) {
  cv::Mat left = render_frame(img, ov, clients);
  if (!stack) return left;
  cv::Mat out;
  cv::hconcat(std::vector<cv::Mat>{left, det_panel(ov, left.rows), ncc_panel(ov, left.rows)}, out);
  return out;
}

namespace {

std::string read_file(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return "";
  std::ostringstream o;
  o << f.rdbuf();
  return o.str();
}

bool send_http(int fd, const char* status, const char* type, const void* body, size_t n) {
  char head[256];
  const int hn = std::snprintf(head, sizeof head,
                               "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-cache\r\n"
                               "Connection: close\r\n\r\n",
                               status, type, n);
  return send_all(fd, head, size_t(hn)) && (n == 0 || send_all(fd, body, n));
}

std::string read_sys(const char* path) {
  std::ifstream f(path);
  std::string v;
  if (f) std::getline(f, v);
  return v;
}

// crop around a main-stream box (with margin) out of a preview-size image
cv::Mat crop_box(const cv::Mat& img, const RectF& b, float scale, float margin = 0.25f) {
  if (img.empty() || b.w <= 0 || b.h <= 0) return {};
  const float mx = b.w * margin, my = b.h * margin;
  cv::Rect r(int((b.x - mx) * scale), int((b.y - my) * scale), int((b.w + 2 * mx) * scale), int((b.h + 2 * my) * scale));
  r &= cv::Rect(0, 0, img.cols, img.rows);
  if (r.width < 4 || r.height < 4) return {};
  cv::Mat c;
  const double k = 160.0 / std::max(r.width, r.height);
  cv::resize(img(r), c, cv::Size(), k, k, k < 1 ? cv::INTER_AREA : cv::INTER_LINEAR);
  return c;
}

}  // namespace

void PreviewServer::broadcast(const std::string& text, std::vector<Conn>& conns) {
  const std::string f = web::ws_frame(text);
  for (auto& c : conns)
    if (c.kind == 2 && c.fd >= 0 && !send_all(c.fd, f.data(), f.size())) { ::close(c.fd); c.fd = -1; }
}

std::string PreviewServer::hello() const {
  // a page opened later only gets the newest capture, and only while nobody has answered it
  std::string j = "{\"type\":\"hello\",\"captures\":[";
  if (!snaps_.empty() && !snaps_.back().answered) j += snaps_.back().meta;
  return j + "]}";
}

void PreviewServer::store_capture(WebCapture& c, std::vector<Conn>& conns) {
  const cv::Mat bgr = web_frame_bgr(c.frame);
  if (bgr.empty()) return;
  Snap s;
  s.n = next_snap_++;
  const std::vector<int> q{cv::IMWRITE_JPEG_QUALITY, 80};
  cv::Mat full = bgr.clone();
  const float k = c.frame.scale;
  cv::rectangle(full, cv::Rect(int(c.box.x * k), int(c.box.y * k), int(c.box.w * k), int(c.box.h * k)), {255, 0, 255}, 2);
  cv::imencode(".jpg", full, s.full, q);
  const cv::Mat cr = crop_box(bgr, c.box, k, 0.35f);
  if (!cr.empty()) cv::imencode(".jpg", cr, s.crop, q);
  char m[320];
  std::snprintf(m, sizeof m,
                "{\"n\":%d,\"track\":%d,\"t\":%.3f,\"cls\":\"%s\",\"score\":%.2f,\"img\":\"/snap/%d.jpg\",\"crop\":\"/crop/%d.jpg\"}",
                s.n, c.track_id, c.t_ns * 1e-9, web::json_escape(c.cls).c_str(), c.score, s.n, s.n);
  s.meta = m;
  s.track = c.track_id;
  broadcast("{\"type\":\"capture\",\"capture\":" + s.meta + "}", conns);
  snaps_.push_back(std::move(s));
  if (snaps_.size() > 40) snaps_.erase(snaps_.begin());
}

void PreviewServer::handle_request(Conn& c, std::vector<Conn>& conns) {
  const std::string path = web::http_path(c.in);
  auto num_of = [&](const char* prefix) {
    return std::atoi(path.c_str() + std::strlen(prefix));
  };
  if (path == "/ws" && !web::http_header(c.in, "Sec-WebSocket-Key").empty()) {
    const std::string acc = web::ws_accept(web::http_header(c.in, "Sec-WebSocket-Key"));
    const std::string resp = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                             "Sec-WebSocket-Accept: " + acc + "\r\n\r\n";
    c.in.clear();
    if (!send_all(c.fd, resp.data(), resp.size())) { ::close(c.fd); c.fd = -1; return; }
    c.kind = 2;
    const std::string h = web::ws_frame(hello());
    const std::string st = web::ws_frame(last_status_sent_.empty() ? "{\"type\":\"status\"}" : last_status_sent_);
    if (!send_all(c.fd, h.data(), h.size()) || !send_all(c.fd, st.data(), st.size())) { ::close(c.fd); c.fd = -1; }
    return;
  }
  if (path == "/stream.mjpg") {
    const char* hdr =
        "HTTP/1.0 200 OK\r\nCache-Control: no-cache\r\nConnection: close\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=rcwsframe\r\n\r\n";
    c.in.clear();
    if (send_all(c.fd, hdr, std::strlen(hdr))) c.kind = 1;
    else { ::close(c.fd); c.fd = -1; }
    return;
  }
  bool ok = false;
  if (path == "/" || path == "/index.html") {
    const std::string page = read_file(web_root_ + "/index.html");
    if (!page.empty()) ok = send_http(c.fd, "200 OK", "text/html; charset=utf-8", page.data(), page.size());
    else {
      const std::string msg = "web/index.html not found (" + web_root_ + "). Live view: /stream.mjpg";
      ok = send_http(c.fd, "404 Not Found", "text/plain; charset=utf-8", msg.data(), msg.size());
    }
  } else if (path.rfind("/snap/", 0) == 0 || path.rfind("/crop/", 0) == 0) {
    const bool crop = path[1] == 'c';
    const int n = num_of(crop ? "/crop/" : "/snap/");
    for (const auto& sn : snaps_)
      if (sn.n == n) {
        const auto& b = crop ? sn.crop : sn.full;
        ok = send_http(c.fd, "200 OK", "image/jpeg", b.data(), b.size());
      }
  } else if (path.rfind("/thumb/", 0) == 0) {
    const int id = num_of("/thumb/");
    for (const auto& k : last_ov_.tracks)
      if (k.id == id) {
        const cv::Mat cr = crop_box(last_bgr_, k.box, last_ov_.scale);
        std::vector<uchar> jpg;
        if (!cr.empty() && cv::imencode(".jpg", cr, jpg, {cv::IMWRITE_JPEG_QUALITY, 75}))
          ok = send_http(c.fd, "200 OK", "image/jpeg", jpg.data(), jpg.size());
      }
  }
  if (!ok) send_http(c.fd, "404 Not Found", "text/plain", "not found", 9);
  ::close(c.fd);
  c.fd = -1;
}

void PreviewServer::run(int cpu) {
  if (cpu >= 0) set_thread_rt(cpu, 0, "preview");
  std::vector<Conn> conns;
  const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 70};
  std::vector<uchar> jpg;
  int64_t t_sys = 0;
  while (!stop_) {
    // ---- network: accept, requests, WebSocket input
    std::vector<pollfd> pf;
    if (listen_fd_ >= 0) pf.push_back({listen_fd_, POLLIN, 0});
    for (const auto& c : conns) pf.push_back({c.fd, POLLIN, 0});
    if (pf.empty()) ::poll(nullptr, 0, 20);
    else if (::poll(pf.data(), nfds_t(pf.size()), 20) > 0) {
      size_t k = 0;
      if (listen_fd_ >= 0) {
        if (pf[0].revents & POLLIN) {
          const int fd = ::accept(listen_fd_, nullptr, nullptr);
          if (fd >= 0) {
            timeval tv{0, 300000};   // a stalled browser must not block the server thread for long
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
            conns.push_back({fd, 0, "", now_ns()});
          }
        }
        k = 1;
      }
      for (size_t i = 0; i + k < pf.size() && i < conns.size(); ++i) {
        Conn& c = conns[i];
        if (!(pf[i + k].revents & (POLLIN | POLLHUP | POLLERR))) continue;
        char buf[2048];
        const ssize_t n = ::recv(c.fd, buf, sizeof buf, MSG_DONTWAIT);
        if (n <= 0) { ::close(c.fd); c.fd = -1; continue; }
        if (c.kind == 1) continue;                       // MJPEG viewers send nothing useful
        c.in.append(buf, size_t(n));
        if (c.kind == 0) {
          if (c.in.find("\r\n\r\n") != std::string::npos) handle_request(c, conns);
          else if (c.in.size() > 16384) { ::close(c.fd); c.fd = -1; }
          continue;
        }
        std::vector<web::WsMsg> msgs;
        if (!web::ws_parse(&c.in, &msgs)) { ::close(c.fd); c.fd = -1; continue; }
        for (auto& m : msgs) {
          if (m.opcode == 8) { ::close(c.fd); c.fd = -1; break; }
          if (m.opcode == 9) { const auto p = web::ws_frame(m.payload, 10); send_all(c.fd, p.data(), p.size()); }
          if (m.opcode == 1) { std::lock_guard<std::mutex> lk(mu_); if (commands_.size() < 256) commands_.push_back(m.payload); }
        }
      }
    }
    const int64_t tnow = now_ns();
    for (auto& c : conns)
      if (c.kind == 0 && c.fd >= 0 && tnow - c.t0 > 3000000000LL) { ::close(c.fd); c.fd = -1; }

    // ---- outgoing messages
    std::string status;
    std::vector<std::string> events;
    std::vector<WebCapture> caps;
    std::vector<int> answered;
    WebFrame frame;
    PreviewOverlay ov;
    bool have_frame = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (status_dirty_) { status = status_; status_dirty_ = false; }
      events.swap(events_);
      caps.swap(captures_in_);
      answered.swap(answered_in_);
      if (has_pending_) { frame = std::move(pending_); ov = pending_ov_; has_pending_ = false; have_frame = true; }
    }
    for (auto& c : caps) store_capture(c, conns);
    for (int id : answered) {
      for (auto& sn : snaps_)
        if (sn.track == id) sn.answered = true;
      broadcast("{\"type\":\"answered\",\"track\":" + std::to_string(id) + "}", conns);   // other open pages drop the prompt
    }
    for (const auto& e : events) broadcast(e, conns);
    if (!status.empty()) { last_status_sent_ = status; broadcast(status, conns); }
    if (tnow - t_sys > 2000000000LL) {   // board health for the dashboard
      t_sys = tnow;
      const std::string temp = read_sys("/sys/class/thermal/thermal_zone0/temp");
      const std::string thr = read_sys("/sys/devices/platform/soc/soc:firmware/get_throttled");
      char j[160];
      std::snprintf(j, sizeof j, "{\"type\":\"sys\",\"temp_c\":%s,\"throttled\":%s%s%s}",
                    temp.empty() ? "null" : std::to_string(std::atoi(temp.c_str()) / 1000.0).c_str(),
                    thr.empty() ? "" : "\"", thr.empty() ? "null" : thr.c_str(), thr.empty() ? "" : "\"");
      broadcast(j, conns);
    }
    conns.erase(std::remove_if(conns.begin(), conns.end(), [](const Conn& c) { return c.fd < 0; }), conns.end());
    int nm = 0, nw = 0;
    for (const auto& c : conns) { nm += c.kind == 1; nw += c.kind == 2; }
    n_clients_ = nm;
    n_ws_ = nw;

    // ---- live view
    if (!have_frame) continue;
    last_bgr_ = web_frame_bgr(frame);
    last_ov_ = ov;
    if (nm == 0 && !window_) continue;
    const cv::Mat img = render_preview(last_bgr_, ov, stack_, nm);
#ifdef RCWS_HAVE_HIGHGUI
    if (window_) {
      cv::imshow("RCWS Stage 1", img);
      cv::waitKey(1);  // lets the window repaint; key presses are ignored (stop with Ctrl-C)
    }
#endif
    if (nm == 0) continue;
    cv::imencode(".jpg", img, jpg, params);
    char head[128];
    const int hn = std::snprintf(head, sizeof head, "--rcwsframe\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                                 jpg.size());
    for (auto& c : conns) {
      if (c.kind != 1) continue;
      if (!(send_all(c.fd, head, size_t(hn)) && send_all(c.fd, jpg.data(), jpg.size()) && send_all(c.fd, "\r\n", 2))) {
        ::close(c.fd);
        c.fd = -1;
      }
    }
  }
  for (auto& c : conns) if (c.fd >= 0) ::close(c.fd);
}

}  // namespace rcws
