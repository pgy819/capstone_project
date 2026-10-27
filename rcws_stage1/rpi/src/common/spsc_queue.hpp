// Bounded lock-free single-producer / single-consumer queue.
// Used between threads whose roles never change (detector completion -> vision
// thread, vision thread -> logger thread). push() fails instead of blocking.
#pragma once
#include <atomic>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace rcws {

template <typename T>
class SpscQueue {
 public:
  explicit SpscQueue(size_t capacity_pow2) : buf_(capacity_pow2), mask_(capacity_pow2 - 1) {
    // capacity must be a power of two
  }
  bool push(T v) {
    const size_t h = head_.load(std::memory_order_relaxed);
    const size_t t = tail_.load(std::memory_order_acquire);
    if (h - t > mask_) return false;  // full
    buf_[h & mask_] = std::move(v);
    head_.store(h + 1, std::memory_order_release);
    return true;
  }
  std::optional<T> pop() {
    const size_t t = tail_.load(std::memory_order_relaxed);
    const size_t h = head_.load(std::memory_order_acquire);
    if (t == h) return std::nullopt;
    T v = std::move(buf_[t & mask_]);
    tail_.store(t + 1, std::memory_order_release);
    return v;
  }
  size_t size() const {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }

 private:
  std::vector<T> buf_;
  const size_t mask_;
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
};

}  // namespace rcws
