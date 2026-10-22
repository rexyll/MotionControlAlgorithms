#pragma once

#include <Eigen/Core>

#include "mca/core/types.hpp"

namespace mca::control {

struct IterativeLearningConfig {
  double learning_gain{0.5};
  double retention_factor{1.0};
  double control_min{-1.0e9};
  double control_max{1.0e9};
};

// Stores a sample-indexed feedforward sequence across repeated executions.
// Update rule: u_(k+1)[i] = clip(rho*u_k[i] + L*e_k[i], u_min, u_max).
// The caller is responsible for aligning error samples with the same trajectory.
class IterativeLearningController {
 public:
  IterativeLearningController() = default;

  core::Status configure(const IterativeLearningConfig& config,
                         Eigen::Index sample_count) {
    if (!valid(config) || sample_count <= 0) {
      configured_ = false;
      feedforward_.resize(0);
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    feedforward_ = Eigen::VectorXd::Zero(sample_count);
    configured_ = true;
    return {};
  }

  core::Status reset(const Eigen::Ref<const Eigen::VectorXd>& initial_sequence) {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (initial_sequence.size() != feedforward_.size() ||
        !initial_sequence.allFinite()) return {core::StatusCode::InvalidArgument};
    Eigen::VectorXd candidate = initial_sequence.cwiseMax(config_.control_min)
                                                   .cwiseMin(config_.control_max);
    feedforward_ = candidate;
    return {};
  }

  core::Status update(const Eigen::Ref<const Eigen::VectorXd>& tracking_error,
                      Eigen::VectorXd& next_sequence) {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (tracking_error.size() != feedforward_.size() ||
        !tracking_error.allFinite()) return {core::StatusCode::InvalidArgument};

    Eigen::VectorXd candidate = config_.retention_factor * feedforward_ +
                                config_.learning_gain * tracking_error;
    if (!candidate.allFinite()) return {core::StatusCode::NumericalError};
    candidate = candidate.cwiseMax(config_.control_min).cwiseMin(config_.control_max);
    feedforward_ = candidate;
    next_sequence = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] const Eigen::VectorXd& feedforward_sequence() const noexcept {
    return feedforward_;
  }

 private:
  static bool valid(const IterativeLearningConfig& c) noexcept {
    return core::is_finite(c.learning_gain) && c.learning_gain >= 0.0 &&
           core::is_finite(c.retention_factor) && c.retention_factor >= 0.0 &&
           c.retention_factor <= 1.0 && core::is_finite(c.control_min) &&
           core::is_finite(c.control_max) && c.control_min <= c.control_max;
  }

  IterativeLearningConfig config_{};
  Eigen::VectorXd feedforward_{};
  bool configured_{false};
};

}  // namespace mca::control
