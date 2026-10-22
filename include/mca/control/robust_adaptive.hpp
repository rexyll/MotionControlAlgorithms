#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <cstddef>
#include <Eigen/Core>

namespace mca::control {

struct RobustAdaptiveConfig {
  double adaptation_gain{1.0};
  double leakage_gain{0.01};
  double robust_gain{0.5};
  double boundary_layer{0.02};
  double control_limit{100.0};
  double parameter_limit{100.0};
};

struct RobustAdaptiveInput {
  Eigen::VectorXd regressor;
  double sliding_error{0.0};
  double nominal_control{0.0};
  double dt{0.0};
};

struct RobustAdaptiveOutput {
  double control{0.0};
  double adaptive_term{0.0};
  double robust_term{0.0};
  Eigen::VectorXd parameters;
};

// Scalar-control adaptive robust compensation. The adaptive law uses projection
// and sigma leakage; the robust term is a continuous boundary-layer approximation
// to sign(s), avoiding an ideal discontinuous switching command.
class RobustAdaptiveController {
 public:
  bool configure(const RobustAdaptiveConfig& config, Eigen::Index parameter_count) {
    if (!valid_config(config) || parameter_count <= 0) return false;
    config_ = config;
    parameters_ = Eigen::VectorXd::Zero(parameter_count);
    configured_ = true;
    return true;
  }

  bool reset(const Eigen::VectorXd& parameters) {
    if (!configured_ || parameters.size() != parameters_.size() ||
        !parameters.allFinite()) return false;
    parameters_ = parameters.cwiseMax(-config_.parameter_limit)
                             .cwiseMin(config_.parameter_limit);
    return true;
  }

  bool update(const RobustAdaptiveInput& input, RobustAdaptiveOutput& output) {
    if (!configured_ || input.regressor.size() != parameters_.size() ||
        !input.regressor.allFinite() || !std::isfinite(input.sliding_error) ||
        !std::isfinite(input.nominal_control) || !std::isfinite(input.dt) ||
        input.dt <= 0.0) return false;

    const double adaptive = parameters_.dot(input.regressor);
    const double scaled_error = input.sliding_error / config_.boundary_layer;
    const double robust = -config_.robust_gain *
        std::max(-1.0, std::min(1.0, scaled_error));
    const double raw_control = input.nominal_control + adaptive + robust;
    if (!std::isfinite(raw_control)) return false;

    Eigen::VectorXd candidate = parameters_;
    // Projected gradient update with sigma-modification to limit parameter drift.
    candidate.array() += input.dt *
        (config_.adaptation_gain * input.sliding_error * input.regressor.array() -
         config_.leakage_gain * candidate.array());
    candidate = candidate.cwiseMax(-config_.parameter_limit)
                         .cwiseMin(config_.parameter_limit);
    if (!candidate.allFinite()) return false;

    RobustAdaptiveOutput result;
    result.control = std::max(-config_.control_limit,
                              std::min(config_.control_limit, raw_control));
    result.adaptive_term = adaptive;
    result.robust_term = robust;
    result.parameters = candidate;
    parameters_ = std::move(candidate);
    output = std::move(result);
    return true;
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] const Eigen::VectorXd& parameters() const noexcept { return parameters_; }

 private:
  static bool valid_config(const RobustAdaptiveConfig& c) {
    return std::isfinite(c.adaptation_gain) && c.adaptation_gain > 0.0 &&
           std::isfinite(c.leakage_gain) && c.leakage_gain >= 0.0 &&
           std::isfinite(c.robust_gain) && c.robust_gain >= 0.0 &&
           std::isfinite(c.boundary_layer) && c.boundary_layer > 0.0 &&
           std::isfinite(c.control_limit) && c.control_limit > 0.0 &&
           std::isfinite(c.parameter_limit) && c.parameter_limit > 0.0;
  }

  RobustAdaptiveConfig config_{};
  Eigen::VectorXd parameters_;
  bool configured_{false};
};

}  // namespace mca::control
