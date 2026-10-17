#pragma once

#include <cmath>

namespace mca::filters {

class FirstOrderLowPass {
 public:
  explicit FirstOrderLowPass(double cutoff_hz = 0.0) noexcept : cutoff_hz_(cutoff_hz) {}

  void set_cutoff(double cutoff_hz) noexcept { cutoff_hz_ = cutoff_hz; }
  void reset(double value = 0.0) noexcept { state_ = value; initialized_ = false; }

  double update(double input, double dt) noexcept {
    if (dt <= 0.0 || !std::isfinite(dt)) return state_;
    if (!initialized_) {
      state_ = input;
      initialized_ = true;
      return state_;
    }
    if (cutoff_hz_ <= 0.0) {
      state_ = input;
      return state_;
    }
    constexpr double two_pi = 6.283185307179586476925286766559;
    const double rc = 1.0 / (two_pi * cutoff_hz_);
    const double alpha = dt / (rc + dt);
    state_ += alpha * (input - state_);
    return state_;
  }

  double value() const noexcept { return state_; }

 private:
  double cutoff_hz_{0.0};
  double state_{0.0};
  bool initialized_{false};
};

}  // namespace mca::filters
