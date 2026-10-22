#pragma once

#include <Eigen/Dense>

#include "mca/control/discrete_time_cbf_filter.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct RobustDiscreteTimeCbfInput {
  DiscreteTimeCbfInput prediction;
  // Per-barrier absolute bound on additive one-step prediction error.
  // The robust constraint uses predicted_drift - prediction_error_bounds.
  Eigen::VectorXd prediction_error_bounds;
};

class RobustDiscreteTimeCbfFilter {
 public:
  core::Status configure(const DiscreteTimeCbfConfig& config) {
    const auto status = inner_.configure(config);
    if (!status.ok()) return status;
    configured_ = true;
    return {};
  }

  core::Status filter(const RobustDiscreteTimeCbfInput& input,
                      DiscreteTimeCbfOutput& output) const {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const Eigen::Index barriers = input.prediction.barrier_values.size();
    if (barriers <= 0 || input.prediction.predicted_drift.size() != barriers ||
        input.prediction_error_bounds.size() != barriers)
      return {core::StatusCode::InvalidArgument};
    if (!input.prediction_error_bounds.allFinite() ||
        (input.prediction_error_bounds.array() < 0.0).any())
      return {core::StatusCode::InvalidArgument};

    DiscreteTimeCbfInput robust = input.prediction;
    robust.predicted_drift =
        input.prediction.predicted_drift - input.prediction_error_bounds;
    if (!robust.predicted_drift.allFinite())
      return {core::StatusCode::NumericalError};

    DiscreteTimeCbfOutput candidate;
    const auto status = inner_.filter(robust, candidate);
    if (!status.ok()) return status;
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  DiscreteTimeCbfFilter inner_{};
  bool configured_{false};
};

}  // namespace mca::control
