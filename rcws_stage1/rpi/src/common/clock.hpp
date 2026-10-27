// Time base helpers (Sec 5.1). The whole runtime uses CLOCK_MONOTONIC.
#pragma once
#include <cstdint>
#include <ctime>

namespace rcws {

inline int64_t now_ns(clockid_t id = CLOCK_MONOTONIC) {
  timespec ts{};
  clock_gettime(id, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// CLOCK_BOOTTIME - CLOCK_MONOTONIC = accumulated suspend time. libcamera specifies
// SensorTimestamp in CLOCK_BOOTTIME; this offset converts it. Sampled per frame so a
// non-zero value (system was suspended) is visible in the log.
inline int64_t boottime_minus_monotonic_ns() {
  const int64_t a = now_ns(CLOCK_MONOTONIC);
  const int64_t b = now_ns(CLOCK_BOOTTIME);
  const int64_t c = now_ns(CLOCK_MONOTONIC);
  return b - (a + (c - a) / 2);
}

}  // namespace rcws
