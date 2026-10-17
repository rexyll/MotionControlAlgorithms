#pragma once

#include <atomic>
#include <cstdint>

namespace mca::realtime {

class SequenceLock {
 public:
  void begin_write() noexcept { sequence_.fetch_add(1U, std::memory_order_acq_rel); }
  void end_write() noexcept { sequence_.fetch_add(1U, std::memory_order_release); }

  bool read_begin(std::uint32_t& sequence) const noexcept {
    sequence = sequence_.load(std::memory_order_acquire);
    return (sequence & 1U) == 0U;
  }

  bool read_retry(std::uint32_t sequence) const noexcept {
    return sequence_.load(std::memory_order_acquire) != sequence;
  }

 private:
  std::atomic<std::uint32_t> sequence_{0U};
};

class FixedRateTimer {
 public:
  explicit FixedRateTimer(double period_s) noexcept : period_s_(period_s) {}
  void reset(double now_s) noexcept { next_s_ = now_s + period_s_; }
  bool due(double now_s) noexcept {
    if (now_s < next_s_) return false;
    next_s_ += period_s_;
    return true;
  }
  double period() const noexcept { return period_s_; }

 private:
  double period_s_{0.001};
  double next_s_{0.0};
};

}  // namespace mca::realtime
