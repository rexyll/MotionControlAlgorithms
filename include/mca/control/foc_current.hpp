#pragma once

#include "mca/core/math.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct FocCurrentControllerConfig {
  double kp_d{0.0};
  double ki_d{0.0};
  double kp_q{0.0};
  double ki_q{0.0};
  double current_min{-1.0e9};
  double current_max{1.0e9};
  double voltage_d_min{-1.0e9};
  double voltage_d_max{1.0e9};
  double voltage_q_min{-1.0e9};
  double voltage_q_max{1.0e9};
  double integral_d_min{-1.0e9};
  double integral_d_max{1.0e9};
  double integral_q_min{-1.0e9};
  double integral_q_max{1.0e9};
};

struct FocCurrentControllerOutput {
  double d_reference{0.0};
  double q_reference{0.0};
  double d_voltage{0.0};
  double q_voltage{0.0};
};

// Discrete dq-axis PI current controller for field-oriented control.
// The two axes are independently saturated; conditional integration prevents
// further windup when the requested voltage is pushing against its limit.
class FocCurrentController {
 public:
  FocCurrentController() = default;
  explicit FocCurrentController(const FocCurrentControllerConfig& config) {
    configure(config);
  }

  core::Status configure(const FocCurrentControllerConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    reset();
    return {};
  }

  core::Status reset(double integral_d = 0.0, double integral_q = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(integral_d) || !core::is_finite(integral_q)) {
      return {core::StatusCode::InvalidArgument};
    }
    integral_d_ = math::clamp(integral_d, config_.integral_d_min, config_.integral_d_max);
    integral_q_ = math::clamp(integral_q, config_.integral_q_min, config_.integral_q_max);
    output_ = {};
    initialized_ = true;
    return {};
  }

  core::Status update(double d_reference, double q_reference,
                      double d_current, double q_current, double dt,
                      FocCurrentControllerOutput& output) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(d_reference) || !core::is_finite(q_reference) ||
        !core::is_finite(d_current) || !core::is_finite(q_current) ||
        !(dt > 0.0) || !core::is_finite(dt)) {
      return {core::StatusCode::InvalidArgument};
    }
    if (!initialized_) reset();

    const double d_ref = math::clamp(d_reference, config_.current_min, config_.current_max);
    const double q_ref = math::clamp(q_reference, config_.current_min, config_.current_max);
    const double d_error = d_ref - d_current;
    const double q_error = q_ref - q_current;

    const double candidate_d_integral = math::clamp(
        integral_d_ + config_.ki_d * d_error * dt,
        config_.integral_d_min, config_.integral_d_max);
    const double candidate_q_integral = math::clamp(
        integral_q_ + config_.ki_q * q_error * dt,
        config_.integral_q_min, config_.integral_q_max);

    const double d_unsaturated = config_.kp_d * d_error + candidate_d_integral;
    const double q_unsaturated = config_.kp_q * q_error + candidate_q_integral;
    const double d_voltage = math::clamp(d_unsaturated, config_.voltage_d_min, config_.voltage_d_max);
    const double q_voltage = math::clamp(q_unsaturated, config_.voltage_q_min, config_.voltage_q_max);

    const bool d_pushes_high = d_voltage >= config_.voltage_d_max && d_error > 0.0;
    const bool d_pushes_low = d_voltage <= config_.voltage_d_min && d_error < 0.0;
    const bool q_pushes_high = q_voltage >= config_.voltage_q_max && q_error > 0.0;
    const bool q_pushes_low = q_voltage <= config_.voltage_q_min && q_error < 0.0;
    if (!(d_pushes_high || d_pushes_low)) integral_d_ = candidate_d_integral;
    if (!(q_pushes_high || q_pushes_low)) integral_q_ = candidate_q_integral;

    output_.d_reference = d_ref;
    output_.q_reference = q_ref;
    output_.d_voltage = d_voltage;
    output_.q_voltage = q_voltage;
    output = output_;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double d_integral() const noexcept { return integral_d_; }
  [[nodiscard]] double q_integral() const noexcept { return integral_q_; }
  [[nodiscard]] const FocCurrentControllerOutput& output() const noexcept { return output_; }

 private:
  static bool valid(const FocCurrentControllerConfig& c) noexcept {
    return core::is_finite(c.kp_d) && core::is_finite(c.ki_d) &&
           core::is_finite(c.kp_q) && core::is_finite(c.ki_q) &&
           core::is_finite(c.current_min) && core::is_finite(c.current_max) &&
           core::is_finite(c.voltage_d_min) && core::is_finite(c.voltage_d_max) &&
           core::is_finite(c.voltage_q_min) && core::is_finite(c.voltage_q_max) &&
           core::is_finite(c.integral_d_min) && core::is_finite(c.integral_d_max) &&
           core::is_finite(c.integral_q_min) && core::is_finite(c.integral_q_max) &&
           c.current_min <= c.current_max &&
           c.voltage_d_min <= c.voltage_d_max && c.voltage_q_min <= c.voltage_q_max &&
           c.integral_d_min <= c.integral_d_max && c.integral_q_min <= c.integral_q_max;
  }

  FocCurrentControllerConfig config_{};
  double integral_d_{0.0};
  double integral_q_{0.0};
  FocCurrentControllerOutput output_{};
  bool configured_{false};
  bool initialized_{false};
};

}  // namespace mca::control
