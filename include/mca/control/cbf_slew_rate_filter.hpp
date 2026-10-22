#pragma once

#include <Eigen/Dense>

#include "mca/control/discrete_time_cbf_filter.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct CbfSlewRateInput {
  DiscreteTimeCbfInput prediction;
  // Last applied control, not merely the last requested command.
  Eigen::VectorXd previous_control;
  // Per-axis maximum absolute change allowed for this control update.
  Eigen::VectorXd max_control_delta;
};

// Discrete-time CBF filter with actuator magnitude and slew-rate limits.
class CbfSlewRateFilter {
 public:
  core::Status configure(const DiscreteTimeCbfConfig& config) {
    DiscreteTimeCbfFilter candidate;
    const auto status = candidate.configure(config);
    if (!status.ok()) return status;
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status filter(const CbfSlewRateInput& input,
                      DiscreteTimeCbfOutput& output) const {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const Eigen::Index controls = config_.control_min.size();
    if (input.previous_control.size() != controls ||
        input.max_control_delta.size() != controls ||
        input.prediction.nominal_control.size() != controls) {
      return {core::StatusCode::InvalidArgument};
    }
    if (!input.previous_control.allFinite() ||
        !input.max_control_delta.allFinite() ||
        (input.max_control_delta.array() < 0.0).any()) {
      return {core::StatusCode::InvalidArgument};
    }

    DiscreteTimeCbfConfig bounded = config_;
    bounded.control_min =
        config_.control_min.cwiseMax(input.previous_control -
                                     input.max_control_delta);
    bounded.control_max =
        config_.control_max.cwiseMin(input.previous_control +
                                     input.max_control_delta);
    if ((bounded.control_min.array() > bounded.control_max.array()).any())
      return {core::StatusCode::ConstraintViolation};

    DiscreteTimeCbfFilter filter;
    const auto configure_status = filter.configure(bounded);
    if (!configure_status.ok()) return configure_status;
    DiscreteTimeCbfOutput candidate;
    const auto status = filter.filter(input.prediction, candidate);
    if (!status.ok()) return status;
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  DiscreteTimeCbfConfig config_{};
  bool configured_{false};
};

}  // namespace mca::control
