#pragma once

#include "mca/core/math.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct PidConfig {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double output_min{-1.0e9};
  double output_max{1.0e9};
  double integral_min{-1.0e9};
  double integral_max{1.0e9};
};

// Discrete PID controller with derivative-on-measurement and conditional
// integration anti-windup. The runtime path performs no dynamic allocation.
class PidController {
 public:
  PidController() = default;
  explicit PidController(const PidConfig& config) { configure(config); }

  core::Status configure(const PidConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    reset();
    return {};
  }

  core::Status reset(double integral = 0.0, double measurement = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(integral) || !core::is_finite(measurement)) {
      return {core::StatusCode::InvalidArgument};
    }
    integral_ = math::clamp(integral, config_.integral_min, config_.integral_max);
    previous_measurement_ = measurement;
    output_ = 0.0;
    initialized_ = true;
    return {};
  }

  core::Status update(double setpoint, double measurement, double dt,
                      double& output) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(setpoint) || !core::is_finite(measurement) ||
        !(dt > 0.0) || !core::is_finite(dt)) {
      return {core::StatusCode::InvalidArgument};
    }
    if (!initialized_) reset(0.0, measurement);

    const double error = setpoint - measurement;
    const double derivative = (measurement - previous_measurement_) / dt;
    const double p = config_.kp * error;
    const double d = -config_.kd * derivative;
    const double candidate_integral = math::clamp(
        integral_ + config_.ki * error * dt,
        config_.integral_min, config_.integral_max);
    const double unsaturated = p + candidate_integral + d;
    const double saturated = math::clamp(
        unsaturated, config_.output_min, config_.output_max);

    const bool pushing_high = saturated >= config_.output_max && error > 0.0;
    const bool pushing_low = saturated <= config_.output_min && error < 0.0;
    if (!(pushing_high || pushing_low)) integral_ = candidate_integral;

    output_ = math::clamp(p + integral_ + d,
                          config_.output_min, config_.output_max);
    previous_measurement_ = measurement;
    output = output_;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double integral() const noexcept { return integral_; }
  [[nodiscard]] double output() const noexcept { return output_; }

 private:
  static bool valid(const PidConfig& c) noexcept {
    return core::is_finite(c.kp) && core::is_finite(c.ki) && core::is_finite(c.kd) &&
           core::is_finite(c.output_min) && core::is_finite(c.output_max) &&
           core::is_finite(c.integral_min) && core::is_finite(c.integral_max) &&
           c.output_min <= c.output_max && c.integral_min <= c.integral_max;
  }

  PidConfig config_{};
  double integral_{0.0};
  double previous_measurement_{0.0};
  double output_{0.0};
  bool configured_{false};
  bool initialized_{false};
};

}  // namespace mca::control
