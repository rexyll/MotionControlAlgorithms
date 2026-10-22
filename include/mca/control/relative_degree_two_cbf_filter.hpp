#pragma once

#include <Eigen/Dense>

#include "mca/control/multi_input_cbf_filter.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct RelativeDegreeTwoCbfConfig {
  double alpha1{1.0};
  double alpha2{1.0};
  Eigen::VectorXd control_min;
  Eigen::VectorXd control_max;
  double coefficient_tolerance{1.0e-9};
  double safety_margin{0.0};
  std::size_t max_total_constraints{16U};
};

struct RelativeDegreeTwoCbfInput {
  Eigen::VectorXd barrier_values;
  Eigen::VectorXd barrier_derivatives;
  Eigen::VectorXd second_drift_derivatives;
  Eigen::MatrixXd control_derivatives;
  Eigen::VectorXd nominal_control;
};

struct RelativeDegreeTwoCbfOutput {
  Eigen::VectorXd control;
  double minimum_safety_margin{0.0};
  std::size_t active_constraint_count{0U};
  bool modified{false};
};

// Multi-input exponential CBF filter for barriers with relative degree two.
// It enforces, for each barrier,
//
//   Lf2 h + LgLf h * u + alpha1 * h_dot + alpha1*alpha2 * h >= margin.
//
// The resulting affine inequalities are delegated to the deterministic
// small-dimensional multi-input CBF projection filter.
class RelativeDegreeTwoCbfFilter {
 public:
  core::Status configure(const RelativeDegreeTwoCbfConfig& config) {
    if (!valid_config(config)) return {core::StatusCode::InvalidArgument};

    MultiInputCbfConfig inner;
    inner.control_min = config.control_min;
    inner.control_max = config.control_max;
    inner.tolerance = config.coefficient_tolerance;
    inner.max_total_constraints = config.max_total_constraints;
    const auto status = inner_.configure(inner);
    if (!status.ok()) return status;

    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status filter(const RelativeDegreeTwoCbfInput& input,
                      RelativeDegreeTwoCbfOutput& output) const {
    if (!configured_) return {core::StatusCode::NotConfigured};

    const Eigen::Index barriers = input.barrier_values.size();
    const Eigen::Index controls = input.nominal_control.size();
    if (barriers <= 0 || controls <= 0 ||
        input.barrier_derivatives.size() != barriers ||
        input.second_drift_derivatives.size() != barriers ||
        input.control_derivatives.rows() != barriers ||
        input.control_derivatives.cols() != controls) {
      return {core::StatusCode::InvalidArgument};
    }

    if (!input.barrier_values.allFinite() ||
        !input.barrier_derivatives.allFinite() ||
        !input.second_drift_derivatives.allFinite() ||
        !input.control_derivatives.allFinite() ||
        !input.nominal_control.allFinite()) {
      return {core::StatusCode::InvalidArgument};
    }

    MultiInputCbfInput transformed;
    transformed.barrier_values = Eigen::VectorXd::Zero(barriers);
    transformed.drift_derivatives = Eigen::VectorXd::Zero(barriers);
    transformed.control_derivatives = input.control_derivatives;
    transformed.nominal_control = input.nominal_control;

    for (Eigen::Index i = 0; i < barriers; ++i) {
      transformed.drift_derivatives(i) =
          input.second_drift_derivatives(i) +
          config_.alpha1 * input.barrier_derivatives(i) +
          config_.alpha1 * config_.alpha2 * input.barrier_values(i) -
          config_.safety_margin;
    }

    MultiInputCbfOutput candidate;
    const auto status = inner_.filter(transformed, candidate);
    if (!status.ok()) return status;

    RelativeDegreeTwoCbfOutput result;
    result.control = candidate.control;
    result.modified = candidate.modified;
    result.active_constraint_count = candidate.active_constraints;
    result.minimum_safety_margin =
        candidate.minimum_safety_margin + config_.safety_margin;
    output = result;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  static bool valid_config(const RelativeDegreeTwoCbfConfig& config) noexcept {
    return core::is_finite(config.alpha1) && config.alpha1 > 0.0 &&
           core::is_finite(config.alpha2) && config.alpha2 > 0.0 &&
           config.control_min.size() > 0 &&
           config.control_min.size() == config.control_max.size() &&
           config.control_min.allFinite() && config.control_max.allFinite() &&
           (config.control_min.array() <= config.control_max.array()).all() &&
           core::is_finite(config.coefficient_tolerance) &&
           config.coefficient_tolerance > 0.0 &&
           core::is_finite(config.safety_margin) &&
           config.safety_margin >= 0.0 &&
           config.max_total_constraints > 0U;
  }

  RelativeDegreeTwoCbfConfig config_{};
  MultiInputCbfFilter inner_{};
  bool configured_{false};
};

}  // namespace mca::control
