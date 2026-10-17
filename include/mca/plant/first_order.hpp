#pragma once

#include <cmath>

#include "mca/core/types.hpp"

namespace mca::plant {

// Continuous-time first-order plant:
//   tau * y_dot + y = gain * u
// Exact zero-order-hold discretization is used by step().
class FirstOrderPlant {
 public:
  FirstOrderPlant() = default;
  explicit FirstOrderPlant(double time_constant, double gain = 1.0) {
    configure(time_constant, gain);
  }

  Status configure(double time_constant, double gain = 1.0) noexcept {
    if (!(time_constant > 0.0) || !is_finite(time_constant) || !is_finite(gain)) {
      configured_ = false;
      return {StatusCode::InvalidArgument};
    }
    time_constant_ = time_constant;
    gain_ = gain;
    configured_ = true;
    output_ = 0.0;
    return {};
  }

  Status reset(double output = 0.0) noexcept {
    if (!configured_) return {StatusCode::NotConfigured};
    if (!is_finite(output)) return {StatusCode::InvalidArgument};
    output_ = output;
    return {};
  }

  Status step(double input, double dt) noexcept {
    if (!configured_) return {StatusCode::NotConfigured};
    if (!is_finite(input) || !(dt > 0.0) || !is_finite(dt)) {
      return {StatusCode::InvalidArgument};
    }
    const double a = std::exp(-dt / time_constant_);
    output_ = a * output_ + gain_ * (1.0 - a) * input;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double output() const noexcept { return output_; }
  [[nodiscard]] double time_constant() const noexcept { return time_constant_; }
  [[nodiscard]] double gain() const noexcept { return gain_; }

 private:
  double time_constant_{1.0};
  double gain_{1.0};
  double output_{0.0};
  bool configured_{false};
};

}  // namespace mca::plant
