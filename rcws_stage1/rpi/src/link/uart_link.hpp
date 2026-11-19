// RPi -> STM32 link (U1): packet v2 over the team UART wiring (Pi GPIO14 TXD -> STM32 PC7,
// GPIO15 RXD <- PC6, 115200 8N1). Frame layout: stm32_fw/include/link_proto.h (shared C source).
//
// Downlink: one AIM frame per processed camera frame, written straight from the vision thread
// (non-blocking, 16 B = 1.4 ms on the wire). Latest frame wins: if the kernel TX queue still holds
// more than one older frame (the line stalled), it is flushed instead of sending stale aims.
// Uplink: a reader thread decodes the 50 Hz STATUS frames; the latest one is read with status().
// No heartbeat is needed: a silent vision thread means stale aims, so the STM32 watchdog holding
// the gimbal after 0.5 s is the intended behaviour.
#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "link_proto.h"
#include "perception/aim.hpp"

namespace rcws {

// AimOut -> AIM frame. lead_s: extra prediction beyond aim.t (aim.psi is already the estimate at
// processing time); clamped to [0, 0.1] s. HOLD sends zero error and zero rate.
LinkAim_t make_link_aim(const AimOut& a, uint8_t seq, uint8_t flags, double lead_s);

// angle conversions shared with the tests
constexpr double kCountsPerRad = LINK_COUNTS_PER_REV / (2.0 * 3.14159265358979323846);

struct LinkStatusView {
  bool valid = false;        // at least one STATUS frame received
  int64_t t_ns = 0;          // CLOCK_MONOTONIC of the last frame
  LinkStatus_t s{};
  uint32_t frames = 0, crc_err = 0;
};

class UartLink {
 public:
  ~UartLink() { close(); }
  bool open(const std::string& dev, int baud, std::string* err);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // vision thread
  void send(const LinkAim_t& a);
  LinkStatusView status() const;

  uint64_t sent() const { return sent_.load(); }
  uint64_t write_errors() const { return werr_.load(); }
  uint64_t stale_flushes() const { return flushes_.load(); }

 private:
  void rx_loop();
  int fd_ = -1;
  std::thread rx_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> sent_{0}, werr_{0}, flushes_{0};
  mutable std::mutex mu_;
  LinkStatusView st_;
};

}  // namespace rcws
