#include "link/uart_link.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#include "common/clock.hpp"

namespace rcws {

namespace {
int16_t sat16(double v, double lim) {
  v = std::clamp(v, -lim, lim);
  return int16_t(std::lround(v));
}
speed_t baud_const(int b) {
  switch (b) {
    case 9600: return B9600;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return 0;
  }
}
}  // namespace

LinkAim_t make_link_aim(const AimOut& a, uint8_t seq, uint8_t flags, double lead_s) {
  LinkAim_t f{};
  f.seq = seq;
  f.flags = flags;
  f.mode = uint8_t(a.mode);
  if (a.mode == AimMode::HOLD) return f;   // zero error / rate: the STM32 brakes and holds
  const double dt = std::clamp(lead_s, 0.0, 0.1);
  const double psi = a.psi + a.psi_d * dt;      // +right
  const double th = a.theta + a.theta_d * dt;   // +up
  const double lim = 16.0 * 700;                // VISION_ERR_LIMIT in protocol.h (+-61.5 deg)
  f.err_pan = sat16(psi * kCountsPerRad * LINK_ERR_SCALE, lim);
  f.err_tilt = sat16(th * kCountsPerRad * LINK_ERR_SCALE, lim);
  f.rate_pan = sat16(a.psi_d * kCountsPerRad, 17000);
  f.rate_tilt = sat16(a.theta_d * kCountsPerRad, 17000);
  return f;
}

bool UartLink::open(const std::string& dev, int baud, std::string* err) {
  const speed_t sp = baud_const(baud);
  if (!sp) { *err = "unsupported baud " + std::to_string(baud); return false; }
  fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    *err = dev + ": " + std::strerror(errno) +
           (errno == EACCES ? " (add the user to group dialout)" : errno == ENOENT ? " (enable_uart=1 / dtparam=uart0=on in /boot/firmware/config.txt)" : "");
    return false;
  }
  termios t{};
  if (tcgetattr(fd_, &t) != 0) { *err = std::string("tcgetattr: ") + std::strerror(errno); close(); return false; }
  cfmakeraw(&t);
  t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  t.c_cflag |= CLOCAL | CREAD | CS8;
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  cfsetispeed(&t, sp);
  cfsetospeed(&t, sp);
  if (tcsetattr(fd_, TCSANOW, &t) != 0) { *err = std::string("tcsetattr: ") + std::strerror(errno); close(); return false; }
  tcflush(fd_, TCIOFLUSH);
  stop_ = false;
  rx_ = std::thread([this] { rx_loop(); });
  return true;
}

void UartLink::close() {
  stop_ = true;
  if (rx_.joinable()) rx_.join();
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

void UartLink::send(const LinkAim_t& a) {
  if (fd_ < 0) return;
  int queued = 0;
  if (ioctl(fd_, TIOCOUTQ, &queued) == 0 && queued > int(LINK_FRAME_LEN)) {
    tcflush(fd_, TCOFLUSH);   // stale aims behind a stalled line: drop them, the newest goes next
    flushes_++;
  }
  uint8_t b[LINK_FRAME_LEN];
  Link_EncodeAim(b, &a);
  const ssize_t n = ::write(fd_, b, sizeof b);
  if (n == ssize_t(sizeof b)) sent_++;
  else werr_++;   // partial frame: the STM32 frame finder discards it (CRC)
}

LinkStatusView UartLink::status() const {
  std::lock_guard<std::mutex> lk(mu_);
  return st_;
}

void UartLink::rx_loop() {
  LinkParser_t p;
  Link_ParserInit(&p, LINK_SYNC1_UP);
  uint8_t buf[256], f[LINK_FRAME_LEN];
  while (!stop_) {
    pollfd pf{fd_, POLLIN, 0};
    if (poll(&pf, 1, 50) <= 0) continue;
    const ssize_t n = ::read(fd_, buf, sizeof buf);
    if (n <= 0) continue;
    for (ssize_t i = 0; i < n; ++i) {
      if (!Link_ParseByte(&p, buf[i], f)) continue;
      LinkStatus_t s;
      if (!Link_DecodeStatus(f, &s)) continue;
      std::lock_guard<std::mutex> lk(mu_);
      st_.valid = true;
      st_.t_ns = now_ns();
      st_.s = s;
      st_.frames = p.frames;
      st_.crc_err = p.crc_err;
    }
  }
}

}  // namespace rcws
