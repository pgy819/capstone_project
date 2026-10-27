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

cv::Mat PreviewServer::render(const cv::Mat& gray, const PreviewOverlay& ov) const {
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
  // tracked target
  if (ov.has_target) {
    const cv::Scalar col = state_color(ov.state);
    const cv::Rect r = to_rect(ov.target, s);
    cv::rectangle(img, r, col, 2);
    cv::line(img, c, (r.tl() + r.br()) / 2, col, 1);
  }
  // status bar
  cv::rectangle(img, cv::Rect(0, 0, img.cols, 34), {0, 0, 0}, cv::FILLED);
  char l1[160], l2[160];
  std::snprintf(l1, sizeof l1, "%-7s  #%u  %.1f fps  proc %.2f ms  clients %d", to_string(ov.state), ov.seq, ov.fps,
                ov.proc_ms, n_clients_.load());
  if (ov.est_valid)
    std::snprintf(l2, sizeof l2, "psi %+7.2f  theta %+7.2f deg   rate %+6.1f %+6.1f deg/s   R %.1f m", ov.psi_deg,
                  ov.theta_deg, ov.rate_psi, ov.rate_theta, ov.range_m);
  else
    std::snprintf(l2, sizeof l2, "no target estimate");
  cv::putText(img, l1, {6, 14}, cv::FONT_HERSHEY_SIMPLEX, 0.42, state_color(ov.state), 1);
  cv::putText(img, l2, {6, 29}, cv::FONT_HERSHEY_SIMPLEX, 0.42, {255, 255, 255}, 1);
  return img;
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
    const cv::Mat img = render(gray, ov);
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
