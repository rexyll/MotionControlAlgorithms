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

  core::Status configure(double time_constant, double gain = 1.0) noexcept {
    if (!(time_constant > 0.0) || !core::is_finite(time_constant) ||
        !core::is_finite(gain)) {
      return {core::StatusCode::InvalidArgument};
    }
    time_constant_ = time_constant;
    gain_ = gain;
    configured_ = true;
    output_ = 0.0;
    return {};
  }

  core::Status reset(double output = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(output)) return {core::StatusCode::InvalidArgument};
    output_ = output;
    return {};
  }

  core::Status step(double input, double dt) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(input) || !(dt > 0.0) || !core::is_finite(dt)) {
      return {core::StatusCode::InvalidArgument};
    }
    const double a = std::exp(-dt / time_constant_);
    const double retained_output = a * output_;
    const double input_gain = gain_ * (1.0 - a);
    if (!core::is_finite(retained_output) || !core::is_finite(input_gain)) {
      return {core::StatusCode::NumericalError};
    }
    const double input_contribution = input_gain * input;
    if (!core::is_finite(input_contribution)) {
      return {core::StatusCode::NumericalError};
    }
    const double candidate = retained_output + input_contribution;
    if (!core::is_finite(candidate)) return {core::StatusCode::NumericalError};
    output_ = candidate;
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
