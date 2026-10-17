#pragma once

#include "mca/core/types.hpp"

namespace mca::control {

struct PidFeedforwardConfig {
  double velocity_gain{0.0};
  double acceleration_gain{0.0};
  double constant_bias{0.0};
};

// Model-based scalar feedforward:
//   u_ff = kv * v_ref + ka * a_ref + bias
class PidFeedforward {
 public:
  explicit PidFeedforward(PidFeedforwardConfig config = {}) : config_(config) {}

  core::Status configure(const PidFeedforwardConfig& config) noexcept {
    if (!core::is_finite(config.velocity_gain) ||
        !core::is_finite(config.acceleration_gain) ||
        !core::is_finite(config.constant_bias)) {
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    return {};
  }

  [[nodiscard]] double compute(double velocity_reference,
                               double acceleration_reference) const noexcept {
    return config_.velocity_gain * velocity_reference +
           config_.acceleration_gain * acceleration_reference +
           config_.constant_bias;
  }

 private:
  PidFeedforwardConfig config_{};
};

}  // namespace mca::control
