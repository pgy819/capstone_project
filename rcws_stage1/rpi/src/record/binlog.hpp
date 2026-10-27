// Binary run log (Sec 12.4). Writer thread drains an SPSC queue so the vision
// thread never blocks on I/O. Layout is mirrored by tools/rcws_log.py.
//
// File:   "RCWSLOG\0" | u32 version | u32 cfg_len | cfg text | records...
// Record: u16 type | u16 payload_len | payload (packed, little-endian)
#pragma once
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "common/spsc_queue.hpp"
#include "common/types.hpp"

namespace rcws {

enum class RecType : uint16_t { FRAME = 1, DETBATCH = 2, MEAS = 3, EST = 4, STATE = 5, NCC = 6, NOTE = 7, TRUTH = 8 };

#pragma pack(push, 1)
struct RecFrame {
  uint32_t seq;
  int64_t t_sof, t_in, t_out;   // t_in/t_out: processing start/end (CLOCK_MONOTONIC)
  int64_t boot_minus_mono;
  int32_t exposure_us;
  float gain, lens;
  uint8_t state, in_flight;
  uint16_t dropped;             // frames lost by the source since the previous one
};
struct RecDetHead {
  uint32_t seq, consumed_seq;   // frame of the request / frame during which it was consumed
  int64_t t_sof, t_submit, t_done;
  float crop[4];
  uint8_t kind, n;
  uint16_t pad;
};
struct RecDet {
  float box[4];
  float score;
  uint8_t cls, pad[3];
};
struct RecMeas {
  int64_t t_cap;
  uint32_t seq;
  float u, v, sigma, conf, w, h, d2;
  uint8_t src, accepted, pad[2];
};
struct RecEst {
  int64_t t;
  double psi, psi_d, psi_dd, theta, theta_d, theta_dd, sig_psi, sig_theta;
  float mu_ca, range_m;
};
struct RecState {
  int64_t t;
  uint8_t from, to, reason, pad;
};
struct RecTruth {               // reference trajectory (synthetic scene / monitor test bench)
  int64_t t;
  uint32_t seq;
  float u, v, w, h;
  uint8_t visible, pad[3];
};
struct RecNcc {
  int64_t t;
  uint32_t seq;
  float u, v, rho, second, radius;
  int16_t tmpl;
  uint8_t ok, pad;
};
#pragma pack(pop)

class BinLog {
 public:
  BinLog() : q_(1 << 14) {}
  ~BinLog() { close(); }
  bool open(const std::string& path, const std::string& cfg_text, int cpu = -1);
  void close();

  template <typename T>
  void write(RecType t, const T& rec) { write_raw(t, &rec, sizeof(T)); }
  void write_raw(RecType t, const void* p, size_t n);
  void write_det(const DetectionBatch& b, uint32_t consumed_seq);
  void note(const std::string& s) { write_raw(RecType::NOTE, s.data(), std::min<size_t>(s.size(), 60000)); }

  uint64_t dropped() const { return dropped_.load(); }
  bool is_open() const { return f_ != nullptr; }

 private:
  void run(int cpu);
  SpscQueue<std::vector<uint8_t>> q_;
  std::FILE* f_ = nullptr;
  std::thread th_;
  std::atomic<bool> stop_{false};
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace rcws
