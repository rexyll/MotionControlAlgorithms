#pragma once

#include <cstddef>
#include <Eigen/Dense>

#include "mca/control/multi_input_cbf_filter.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct DiscreteTimeCbfConfig {
  // Enforce h(x[k+1]) >= (1 - gamma) * h(x[k]) + safety_margin.
  double gamma{0.2};
  Eigen::VectorXd control_min;
  Eigen::VectorXd control_max;
  double tolerance{1.0e-8};
  double safety_margin{0.0};
  std::size_t max_total_constraints{16U};
};

struct DiscreteTimeCbfInput {
  Eigen::VectorXd barrier_values;
  // Control-independent part of the one-step prediction of each barrier.
  // h_predicted(u) = predicted_drift + control_derivatives * u.
  Eigen::VectorXd predicted_drift;
  Eigen::MatrixXd control_derivatives;
  Eigen::VectorXd nominal_control;
};

struct DiscreteTimeCbfOutput {
  Eigen::VectorXd control;
  double minimum_safety_margin{0.0};
  std::size_t active_constraint_count{0U};
  bool modified{false};
};

// Projects a nominal control onto discrete-time CBF constraints and actuator
// bounds. The underlying active-set enumeration is intended for small inputs.
class DiscreteTimeCbfFilter {
 public:
  core::Status configure(const DiscreteTimeCbfConfig& config) {
    if (!valid_config(config)) return {core::StatusCode::InvalidArgument};
    MultiInputCbfConfig inner;
    inner.control_min = config.control_min;
    inner.control_max = config.control_max;
    inner.tolerance = config.tolerance;
    inner.max_total_constraints = config.max_total_constraints;
    const auto status = inner_.configure(inner);
    if (!status.ok()) return status;
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status filter(const DiscreteTimeCbfInput& input,
                      DiscreteTimeCbfOutput& output) const {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const Eigen::Index barriers = input.barrier_values.size();
    const Eigen::Index controls = input.nominal_control.size();
    if (barriers <= 0 || controls != config_.control_min.size() ||
        input.predicted_drift.size() != barriers ||
        input.control_derivatives.rows() != barriers ||
        input.control_derivatives.cols() != controls) {
      return {core::StatusCode::InvalidArgument};
    }
    if (!input.barrier_values.allFinite() || !input.predicted_drift.allFinite() ||
        !input.control_derivatives.allFinite() || !input.nominal_control.allFinite()) {
      return {core::StatusCode::InvalidArgument};
    }

    MultiInputCbfInput transformed;
    transformed.barrier_values = Eigen::VectorXd::Zero(barriers);
    transformed.drift_derivatives =
        input.predicted_drift - (1.0 - config_.gamma) * input.barrier_values;
    transformed.drift_derivatives.array() -= config_.safety_margin;
    transformed.control_derivatives = input.control_derivatives;
    transformed.nominal_control = input.nominal_control;
    if (!transformed.drift_derivatives.allFinite())
      return {core::StatusCode::NumericalError};

    MultiInputCbfOutput candidate;
    const auto status = inner_.filter(transformed, candidate);
    if (!status.ok()) return status;
    DiscreteTimeCbfOutput result;
    result.control = candidate.control;
    result.minimum_safety_margin =
        candidate.minimum_safety_margin + config_.safety_margin;
    result.active_constraint_count = candidate.active_constraints;
    result.modified = candidate.modified;
    output = result;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  static bool valid_config(const DiscreteTimeCbfConfig& config) noexcept {
    return core::is_finite(config.gamma) && config.gamma > 0.0 &&
           config.gamma <= 1.0 && config.control_min.size() > 0 &&
           config.control_min.size() == config.control_max.size() &&
           config.control_min.allFinite() && config.control_max.allFinite() &&
           (config.control_min.array() <= config.control_max.array()).all() &&
           core::is_finite(config.tolerance) && config.tolerance > 0.0 &&
           core::is_finite(config.safety_margin) && config.safety_margin >= 0.0 &&
           config.max_total_constraints > 0U;
  }

  DiscreteTimeCbfConfig config_{};
  MultiInputCbfFilter inner_{};
  bool configured_{false};
};

}  // namespace mca::control
