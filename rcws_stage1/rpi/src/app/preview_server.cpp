#include "app/preview_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#ifdef RCWS_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

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
  std::unique_lock<std::mutex> lk(mu_, std::try_to_lock);
  if (!lk.owns_lock()) return;
  gray.copyTo(pending_);
  pending_ov_ = ov;
  has_pending_ = true;
}

namespace {

cv::Mat render_frame(const cv::Mat& gray, const PreviewOverlay& ov, int clients) {
  cv::Mat img;
  cv::cvtColor(gray, img, cv::COLOR_GRAY2BGR);
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

cv::Mat render_preview(const cv::Mat& gray, const PreviewOverlay& ov, bool stack, int clients) {
  cv::Mat left = render_frame(gray, ov, clients);
  if (!stack) return left;
  cv::Mat out;
  cv::hconcat(std::vector<cv::Mat>{left, det_panel(ov, left.rows), ncc_panel(ov, left.rows)}, out);
  return out;
}

void PreviewServer::run(int cpu) {
  if (cpu >= 0) set_thread_rt(cpu, 0, "preview");
  std::vector<int> clients;
  const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 70};
  std::vector<uchar> jpg;
  while (!stop_) {
    pollfd p{listen_fd_, POLLIN, 0};
    if (listen_fd_ < 0) ::poll(nullptr, 0, 30);
    else if (::poll(&p, 1, 30) > 0 && (p.revents & POLLIN)) {
      const int fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd >= 0) {
        char req[1024];
        ::recv(fd, req, sizeof req, MSG_DONTWAIT);  // request content is ignored
        const char* hdr =
            "HTTP/1.0 200 OK\r\nCache-Control: no-cache\r\nConnection: close\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=rcwsframe\r\n\r\n";
        if (send_all(fd, hdr, std::strlen(hdr))) clients.push_back(fd);
        else ::close(fd);
      }
    }
    cv::Mat gray;
    PreviewOverlay ov;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!has_pending_) { n_clients_ = int(clients.size()); continue; }
      gray = pending_.clone();
      ov = pending_ov_;
      has_pending_ = false;
    }
    if (clients.empty() && !window_) continue;
    const cv::Mat img = render_preview(gray, ov, stack_, n_clients_.load());
#ifdef RCWS_HAVE_HIGHGUI
    if (window_) {
      cv::imshow("RCWS Stage 1", img);
      cv::waitKey(1);  // lets the window repaint; key presses are ignored (stop with Ctrl-C)
    }
#endif
    if (clients.empty()) continue;
    cv::imencode(".jpg", img, jpg, params);
    char head[128];
    const int hn = std::snprintf(head, sizeof head, "--rcwsframe\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                                 jpg.size());
    for (size_t i = 0; i < clients.size();) {
      const int fd = clients[i];
      if (send_all(fd, head, size_t(hn)) && send_all(fd, jpg.data(), jpg.size()) && send_all(fd, "\r\n", 2)) {
        ++i;
      } else {
        ::close(fd);
        clients.erase(clients.begin() + long(i));
      }
    }
    n_clients_ = int(clients.size());
  }
  for (int fd : clients) ::close(fd);
}

}  // namespace rcws
