#pragma once

#include "mca/legged/whole_body_qp.hpp"

#include <cmath>
#include <string>

namespace mca::legged {

// Coordinates a locomotion/stability task and an end-effector manipulation
// task in one bounded generalized-velocity/acceleration solve.
struct WalkingManipulationInput {
  Eigen::Index variables{0};
  Eigen::MatrixXd locomotion_jacobian;
  Eigen::VectorXd locomotion_target;
  Eigen::MatrixXd manipulation_jacobian;
  Eigen::VectorXd manipulation_target;
  double locomotion_weight{10.0};
  double manipulation_weight{1.0};
  double regularization{1e-6};
  Eigen::VectorXd lower_bounds;
  Eigen::VectorXd upper_bounds;
  Eigen::VectorXd reference;
  int iterations{120};
  double step_size{0.0};
};

struct WalkingManipulationOutput {
  Eigen::VectorXd generalized_command;
  double locomotion_residual_norm{0.0};
  double manipulation_residual_norm{0.0};
  double objective{0.0};
  int iterations{0};
};

class WalkingManipulationController {
 public:
  static bool solve(const WalkingManipulationInput& input,
                    WalkingManipulationOutput& output,
                    std::string* error = nullptr) {
    const auto fail = [error](const char* message) {
      if (error) *error = message;
      return false;
    };
    const Eigen::Index n = input.variables;
    if (n <= 0) return fail("walking-manipulation variable count must be positive");
    if (input.locomotion_jacobian.rows() <= 0 ||
        input.locomotion_jacobian.cols() != n ||
        input.locomotion_target.size() != input.locomotion_jacobian.rows())
      return fail("locomotion task dimensions are inconsistent");
    if (input.manipulation_jacobian.rows() <= 0 ||
        input.manipulation_jacobian.cols() != n ||
        input.manipulation_target.size() != input.manipulation_jacobian.rows())
      return fail("manipulation task dimensions are inconsistent");
    if (!input.locomotion_jacobian.allFinite() ||
        !input.locomotion_target.allFinite() ||
        !input.manipulation_jacobian.allFinite() ||
        !input.manipulation_target.allFinite())
      return fail("walking-manipulation task values must be finite");
    if (!std::isfinite(input.locomotion_weight) || input.locomotion_weight <= 0.0 ||
        !std::isfinite(input.manipulation_weight) || input.manipulation_weight <= 0.0)
      return fail("task weights must be positive and finite");

    WholeBodyQpInput qp;
    qp.variables = n;
    qp.regularization = input.regularization;
    qp.iterations = input.iterations;
    qp.step_size = input.step_size;
    qp.lower_bounds = input.lower_bounds;
    qp.upper_bounds = input.upper_bounds;
    qp.reference = input.reference;
    qp.tasks.push_back({input.locomotion_jacobian, input.locomotion_target,
                        input.locomotion_weight});
    qp.tasks.push_back({input.manipulation_jacobian, input.manipulation_target,
                        input.manipulation_weight});

    WholeBodyQpOutput solved;
    if (!WholeBodyQp::solve(qp, solved, error)) return false;
    if (solved.task_residual_norms.size() != 2)
      return fail("walking-manipulation solver returned unexpected task metrics");

    WalkingManipulationOutput candidate;
    candidate.generalized_command = solved.solution;
    candidate.locomotion_residual_norm = solved.task_residual_norms[0];
    candidate.manipulation_residual_norm = solved.task_residual_norms[1];
    candidate.objective = solved.objective;
    candidate.iterations = solved.iterations;
    if (!candidate.generalized_command.allFinite() ||
        !std::isfinite(candidate.locomotion_residual_norm) ||
        !std::isfinite(candidate.manipulation_residual_norm) ||
        !std::isfinite(candidate.objective))
      return fail("walking-manipulation output is non-finite");
    output = std::move(candidate);
    return true;
  }
};

}  // namespace mca::legged
