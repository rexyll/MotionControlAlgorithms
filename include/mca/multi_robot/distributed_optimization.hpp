#pragma once

#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace mca::multi_robot {

struct DistributedOptimizationInput {
  // Each row is one agent's local estimate in a shared D-dimensional space.
  Eigen::MatrixXd estimates;
  // Current local objective gradients evaluated at each row of estimates.
  Eigen::MatrixXd gradients;
  // Local gradients evaluated at the caller-supplied next_estimates candidate.
  // These must be computed by the caller at the estimates produced by this step.
  Eigen::MatrixXd gradients_at_next_estimates;
  // Nonnegative, doubly-stochastic mixing matrix (row and column sums equal 1).
  Eigen::MatrixXd mixing;
  Eigen::MatrixXd gradient_trackers;
  double step_size{0.01};
  double stochastic_tolerance{1e-9};
};

struct DistributedOptimizationOutput {
  Eigen::MatrixXd estimates;
  Eigen::MatrixXd gradient_trackers;
};

// One gradient-tracking iteration:
// x+ = W x - alpha y
// y+ = W y + grad f_i(x_i+) - grad f_i(x_i).
// The caller evaluates each local objective gradient at x+ and supplies it as
// gradients_at_next_estimates on the same step. W must be doubly stochastic.
inline bool distributed_optimization_step(
    const DistributedOptimizationInput& input,
    DistributedOptimizationOutput& output,
    std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const Eigen::Index n = input.estimates.rows();
  const Eigen::Index d = input.estimates.cols();
  if (n < 2 || d <= 0)
    return fail("estimates must contain at least two agents and one coordinate");
  if (input.gradients.rows() != n || input.gradients.cols() != d ||
      input.gradients_at_next_estimates.rows() != n ||
      input.gradients_at_next_estimates.cols() != d ||
      input.gradient_trackers.rows() != n ||
      input.gradient_trackers.cols() != d)
    return fail("gradient and tracker matrices must match estimates dimensions");
  if (input.mixing.rows() != n || input.mixing.cols() != n)
    return fail("mixing must be square with one row per agent");
  if (!input.estimates.allFinite() || !input.gradients.allFinite() ||
      !input.gradients_at_next_estimates.allFinite() ||
      !input.gradient_trackers.allFinite() || !input.mixing.allFinite())
    return fail("all matrices must contain finite values");
  if ((input.mixing.array() < 0.0).any())
    return fail("mixing weights must be nonnegative");
  if (!std::isfinite(input.step_size) || input.step_size <= 0.0)
    return fail("step_size must be finite and positive");
  if (!std::isfinite(input.stochastic_tolerance) ||
      input.stochastic_tolerance <= 0.0)
    return fail("stochastic_tolerance must be finite and positive");
  const Eigen::VectorXd row_sums = input.mixing.rowwise().sum();
  const Eigen::RowVectorXd column_sums = input.mixing.colwise().sum();
  if ((row_sums.array() - 1.0).abs().maxCoeff() >
          input.stochastic_tolerance ||
      (column_sums.array() - 1.0).abs().maxCoeff() >
          input.stochastic_tolerance)
    return fail("mixing must be doubly stochastic within tolerance");

  DistributedOptimizationOutput candidate;
  candidate.estimates =
      input.mixing * input.estimates - input.step_size * input.gradient_trackers;
  if (!candidate.estimates.allFinite())
    return fail("next estimates contain non-finite values");
  candidate.gradient_trackers =
      input.mixing * input.gradient_trackers +
      input.gradients_at_next_estimates - input.gradients;
  if (!candidate.gradient_trackers.allFinite())
    return fail("next gradient trackers contain non-finite values");
  output = std::move(candidate);
  if (error) error->clear();
  return true;
}

}  // namespace mca::multi_robot
