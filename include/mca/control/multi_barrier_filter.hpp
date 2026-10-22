#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "mca/core/types.hpp"

namespace mca::control {

struct MultiBarrierFilterConfig {
  double alpha{1.0};
  double control_min{-1.0e9};
  double control_max{1.0e9};
  double coefficient_tolerance{1.0e-9};
};

struct BarrierConstraint {
  double barrier_value{0.0};
  double drift_derivative{0.0};
  double control_derivative{0.0};
};

struct MultiBarrierFilterInput {
  std::vector<BarrierConstraint> constraints;
  double nominal_control{0.0};
};

struct MultiBarrierFilterOutput {
  double control{0.0};
  double minimum_safety_margin{0.0};
  std::size_t active_constraints{0};
  bool modified{false};
};

// Intersects first-order CBF inequalities for a scalar actuator:
// L_f h_i + L_g h_i * u + alpha*h_i >= 0 for every constraint i.
class MultiBarrierFilter {
 public:
  core::Status configure(const MultiBarrierFilterConfig& config) {
    if (!valid_config(config)) return {core::StatusCode::InvalidArgument};
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status filter(const MultiBarrierFilterInput& input,
                      MultiBarrierFilterOutput& output) const {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (input.constraints.empty() || !core::is_finite(input.nominal_control)) {
      return {core::StatusCode::InvalidArgument};
    }

    double lower = config_.control_min;
    double upper = config_.control_max;
    for (const auto& constraint : input.constraints) {
      if (!core::is_finite(constraint.barrier_value) ||
          !core::is_finite(constraint.drift_derivative) ||
          !core::is_finite(constraint.control_derivative)) {
        return {core::StatusCode::InvalidArgument};
      }
      const double base = constraint.drift_derivative +
                          config_.alpha * constraint.barrier_value;
      if (!core::is_finite(base)) return {core::StatusCode::NumericalError};

      const double g = constraint.control_derivative;
      if (std::abs(g) <= config_.coefficient_tolerance) {
        if (base < -config_.coefficient_tolerance) {
          return {core::StatusCode::ConstraintViolation};
        }
        continue;
      }

      const double boundary = -base / g;
      if (!core::is_finite(boundary)) return {core::StatusCode::NumericalError};
      if (g > 0.0) {
        lower = std::max(lower, boundary);
      } else {
        upper = std::min(upper, boundary);
      }
      if (lower > upper + config_.coefficient_tolerance) {
        return {core::StatusCode::ConstraintViolation};
      }
    }

    if (lower > upper) {
      const double midpoint = lower + (upper - lower) * 0.5;
      lower = midpoint;
      upper = midpoint;
    }
    const double control = std::max(lower, std::min(upper, input.nominal_control));
    if (!core::is_finite(control)) return {core::StatusCode::NumericalError};

    double minimum_margin = 0.0;
    std::size_t active_constraints = 0;
    bool first = true;
    for (const auto& constraint : input.constraints) {
      const double margin = constraint.drift_derivative +
                            config_.alpha * constraint.barrier_value +
                            constraint.control_derivative * control;
      if (!core::is_finite(margin)) return {core::StatusCode::NumericalError};
      if (margin < -10.0 * config_.coefficient_tolerance) {
        return {core::StatusCode::ConstraintViolation};
      }
      if (first || margin < minimum_margin) minimum_margin = margin;
      first = false;
      if (std::abs(margin) <= 10.0 * config_.coefficient_tolerance) {
        ++active_constraints;
      }
    }

    MultiBarrierFilterOutput candidate;
    candidate.control = control;
    candidate.minimum_safety_margin = minimum_margin;
    candidate.active_constraints = active_constraints;
    candidate.modified = std::abs(control - input.nominal_control) >
                         config_.coefficient_tolerance;
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  static bool valid_config(const MultiBarrierFilterConfig& c) noexcept {
    return core::is_finite(c.alpha) && c.alpha > 0.0 &&
           core::is_finite(c.control_min) && core::is_finite(c.control_max) &&
           c.control_min <= c.control_max &&
           core::is_finite(c.coefficient_tolerance) &&
           c.coefficient_tolerance > 0.0;
  }

  MultiBarrierFilterConfig config_{};
  bool configured_{false};
};

}  // namespace mca::control
