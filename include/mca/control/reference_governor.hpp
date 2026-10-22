#pragma once

#include <algorithm>
#include <cmath>

#include "mca/core/types.hpp"

namespace mca::control {

struct ReferenceGovernorConfig {
  double position_min{-1.0e9};
  double position_max{1.0e9};
  double velocity_max{1.0};
  double acceleration_max{2.0};
};

struct ReferenceGovernorOutput {
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

class ReferenceGovernor {
 public:
  core::Status configure(const ReferenceGovernorConfig& config) noexcept {
    if (!valid_config(config)) return {core::StatusCode::InvalidArgument};
    config_ = config;
    configured_ = true;
    reset_required_ = true;
    return {};
  }

  core::Status reset(double position, double velocity = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(position) || !core::is_finite(velocity) ||
        position < config_.position_min || position > config_.position_max ||
        std::abs(velocity) > config_.velocity_max) {
      return {core::StatusCode::InvalidArgument};
    }
    position_ = position;
    velocity_ = velocity;
    reset_required_ = false;
    return {};
  }

  core::Status update(double target_position, double dt,
                      ReferenceGovernorOutput& output) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (reset_required_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(target_position) || !core::is_finite(dt) || dt <= 0.0 ||
        target_position < config_.position_min || target_position > config_.position_max) {
      return {core::StatusCode::InvalidArgument};
    }

    const double error = target_position - position_;
    const double stopping_speed =
        std::sqrt(2.0 * config_.acceleration_max * std::abs(error));
    double desired_velocity = std::copysign(
        std::min(config_.velocity_max, stopping_speed), error);
    if (std::abs(error) <= 1.0e-14) desired_velocity = 0.0;

    const double max_delta_velocity = config_.acceleration_max * dt;
    const double next_velocity = std::clamp(
        desired_velocity, velocity_ - max_delta_velocity, velocity_ + max_delta_velocity);
    double next_position = position_ + next_velocity * dt;
    next_position = std::clamp(next_position, config_.position_min, config_.position_max);
    const double actual_velocity = (next_position - position_) / dt;
    const double actual_acceleration = (actual_velocity - velocity_) / dt;
    if (!core::is_finite(next_position) || !core::is_finite(actual_velocity) ||
        !core::is_finite(actual_acceleration)) {
      return {core::StatusCode::NumericalError};
    }

    ReferenceGovernorOutput candidate;
    candidate.position = next_position;
    candidate.velocity = actual_velocity;
    candidate.acceleration = actual_acceleration;
    position_ = candidate.position;
    velocity_ = candidate.velocity;
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] bool reset_required() const noexcept { return reset_required_; }

 private:
  static bool valid_config(const ReferenceGovernorConfig& c) noexcept {
    return core::is_finite(c.position_min) && core::is_finite(c.position_max) &&
           c.position_min < c.position_max && core::is_finite(c.velocity_max) &&
           c.velocity_max > 0.0 && core::is_finite(c.acceleration_max) &&
           c.acceleration_max > 0.0;
  }

  ReferenceGovernorConfig config_{};
  double position_{0.0};
  double velocity_{0.0};
  bool configured_{false};
  bool reset_required_{true};
};

}  // namespace mca::control
