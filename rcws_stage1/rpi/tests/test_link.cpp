// U1: packet v2 link on the Pi side. Angle -> frame conversion (sign, units, saturation, HOLD),
// and an end-to-end exchange over a pseudo terminal: AIM frames reach a fake STM32 intact and in
// order, its STATUS frames come back through the reader thread.
#include <chrono>
#include <cmath>
#include <thread>

#include "link/uart_link.hpp"
#include "test_util.hpp"

// after the rcws headers: <termios.h> defines a macro NCC that collides with MeasSrc::NCC
#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

using namespace rcws;

TEST(link_aim_conversion) {
  AimOut a;
  a.mode = AimMode::FOLLOW;
  a.psi = 0.10;      // 0.1 rad right = 65.19 counts
  a.theta = -0.05;   // below the axis
  a.psi_d = 0.5;
  a.theta_d = 0;
  const LinkAim_t f = make_link_aim(a, 7, LINK_F_OPERATOR_HOLD, 0);
  CHECK(f.seq == 7 && f.mode == LINK_MODE_FOLLOW && f.flags == LINK_F_OPERATOR_HOLD);
  CHECK_NEAR(f.err_pan, 0.10 * 4096 / (2 * M_PI) * 16, 0.6);
  CHECK_NEAR(f.err_tilt, -0.05 * 4096 / (2 * M_PI) * 16, 0.6);
  CHECK_NEAR(f.rate_pan, 0.5 * 4096 / (2 * M_PI), 0.6);
  // lead: 20 ms at 0.5 rad/s = 10 mrad more
  const LinkAim_t g = make_link_aim(a, 0, 0, 0.020);
  CHECK_NEAR(g.err_pan - f.err_pan, 0.010 * 4096 / (2 * M_PI) * 16, 1.0);
  CHECK(make_link_aim(a, 0, 0, 5.0).err_pan == make_link_aim(a, 0, 0, 0.1).err_pan);   // lead clamped
  // far off-axis: saturates at the STM32 sanity gate instead of wrapping
  a.psi = 3.0;
  CHECK(make_link_aim(a, 0, 0, 0).err_pan == 16 * 700);
  a.psi = -3.0;
  CHECK(make_link_aim(a, 0, 0, 0).err_pan == -16 * 700);
  // HOLD: zero error and rate whatever the estimate says
  a.mode = AimMode::HOLD;
  const LinkAim_t h = make_link_aim(a, 1, 0, 0);
  CHECK(h.mode == LINK_MODE_HOLD && h.err_pan == 0 && h.err_tilt == 0 && h.rate_pan == 0 && h.rate_tilt == 0);
}

TEST(link_pty_roundtrip) {
  int master = -1, slave = -1;
  char name[128];
  if (openpty(&master, &slave, name, nullptr, nullptr) != 0) {
    std::printf("  (no pty available, skipped)\n");
    return;
  }
  termios t{};
  tcgetattr(master, &t);
  cfmakeraw(&t);
  tcsetattr(master, TCSANOW, &t);
  UartLink link;
  std::string err;
  CHECK(link.open(name, 115200, &err));
  if (!link.is_open()) { std::printf("  %s\n", err.c_str()); close(master); close(slave); return; }

  // Pi -> fake STM32: 50 AIM frames
  AimOut a;
  a.mode = AimMode::FOLLOW;
  for (int i = 0; i < 50; ++i) {
    a.psi = 0.001 * i;
    link.send(make_link_aim(a, uint8_t(i), 0, 0));
  }
  LinkParser_t p;
  Link_ParserInit(&p, LINK_SYNC1_DOWN);
  uint8_t buf[256], f[LINK_FRAME_LEN];
  int got = 0, in_order = 1;
  for (int tries = 0; tries < 100 && got < 50; ++tries) {
    const ssize_t n = read(master, buf, sizeof buf);
    if (n <= 0) { std::this_thread::sleep_for(std::chrono::milliseconds(2)); continue; }
    for (ssize_t i = 0; i < n; ++i) {
      LinkAim_t d;
      if (Link_ParseByte(&p, buf[i], f) && Link_DecodeAim(f, &d)) {
        if (d.seq != got) in_order = 0;
        ++got;
      }
    }
  }
  CHECK(got == 50 && in_order && p.crc_err == 0);
  CHECK(link.sent() == 50 && link.write_errors() == 0);

  // fake STM32 -> Pi: STATUS with a byte of garbage in front
  LinkStatus_t s{49, 0, LINK_S_OPERATOR_HOLD, 2100, 1990, 50, 0};
  uint8_t out[LINK_FRAME_LEN + 1];
  out[0] = 0x13;
  Link_EncodeStatus(out + 1, &s);
  CHECK(write(master, out, sizeof out) == ssize_t(sizeof out));
  LinkStatusView v;
  for (int tries = 0; tries < 100 && !v.valid; ++tries) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    v = link.status();
  }
  CHECK(v.valid && v.s.seq_echo == 49 && v.s.pan_goal == 2100 && v.s.tilt_goal == 1990 && v.s.flags == LINK_S_OPERATOR_HOLD);
  link.close();
  close(master);
  close(slave);
}
