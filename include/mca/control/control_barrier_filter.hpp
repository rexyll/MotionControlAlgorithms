#pragma once

#include <algorithm>
#include <cmath>

#include "mca/core/types.hpp"

namespace mca::control {

struct ControlBarrierFilterConfig {
  double alpha{1.0};
  double control_min{-1.0e9};
  double control_max{1.0e9};
  double coefficient_tolerance{1.0e-9};
};

struct ControlBarrierFilterInput {
  double barrier_value{0.0};
  double drift_derivative{0.0};
  double control_derivative{0.0};
  double nominal_control{0.0};
};

struct ControlBarrierFilterOutput {
  double control{0.0};
  double safety_margin{0.0};
  bool modified{false};
};

// Safety filter for a scalar control-affine system h_dot = L_f h + L_g h * u.
// It minimally changes the nominal command while enforcing
// L_f h + L_g h * u + alpha*h >= 0 and the actuator bounds.
class ControlBarrierFilter {
 public:
  core::Status configure(const ControlBarrierFilterConfig& config) {
    if (!valid_config(config)) return {core::StatusCode::InvalidArgument};
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status filter(const ControlBarrierFilterInput& input,
                      ControlBarrierFilterOutput& output) const {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(input.barrier_value) ||
        !core::is_finite(input.drift_derivative) ||
        !core::is_finite(input.control_derivative) ||
        !core::is_finite(input.nominal_control)) {
      return {core::StatusCode::InvalidArgument};
    }

    const double base = input.drift_derivative + config_.alpha * input.barrier_value;
    if (!core::is_finite(base)) return {core::StatusCode::NumericalError};

    double lower = config_.control_min;
    double upper = config_.control_max;
    const double g = input.control_derivative;
    if (std::abs(g) <= config_.coefficient_tolerance) {
      if (base < -config_.coefficient_tolerance) {
        return {core::StatusCode::ConstraintViolation};
      }
    } else {
      const double boundary = -base / g;
      if (!core::is_finite(boundary)) return {core::StatusCode::NumericalError};
      if (g > 0.0) {
        lower = std::max(lower, boundary);
      } else {
        upper = std::min(upper, boundary);
      }
    }

    if (lower > upper + config_.coefficient_tolerance) {
      return {core::StatusCode::ConstraintViolation};
    }
    // Collapse tiny round-off-only interval inversions to their midpoint.
    if (lower > upper) {
      const double midpoint = lower + (upper - lower) * 0.5;
      lower = midpoint;
      upper = midpoint;
    }

    const double control = std::max(lower, std::min(upper, input.nominal_control));
    const double margin = base + g * control;
    if (!core::is_finite(control) || !core::is_finite(margin)) {
      return {core::StatusCode::NumericalError};
    }
    if (margin < -10.0 * config_.coefficient_tolerance) {
      return {core::StatusCode::ConstraintViolation};
    }

    ControlBarrierFilterOutput candidate;
    candidate.control = control;
    candidate.safety_margin = margin;
    candidate.modified = std::abs(control - input.nominal_control) >
                         config_.coefficient_tolerance;
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  static bool valid_config(const ControlBarrierFilterConfig& c) noexcept {
    return core::is_finite(c.alpha) && c.alpha > 0.0 &&
           core::is_finite(c.control_min) && core::is_finite(c.control_max) &&
           c.control_min <= c.control_max &&
           core::is_finite(c.coefficient_tolerance) &&
           c.coefficient_tolerance > 0.0;
  }

  ControlBarrierFilterConfig config_{};
  bool configured_{false};
};

}  // namespace mca::control
