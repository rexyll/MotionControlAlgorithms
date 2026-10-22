#pragma once

#include <cmath>
#include <string>

#include <Eigen/Dense>

namespace mca::multi_robot {

struct ConsensusInput {
  // One row per robot/agent; columns are independent state coordinates.
  Eigen::MatrixXd states;
  // Nonnegative directed edge weights. adjacency(i,j) weights agent j in i's update.
  Eigen::MatrixXd adjacency;
  double gain{1.0};
  double time_step{0.01};
};

struct ConsensusOutput {
  Eigen::MatrixXd states;
};

// Performs one explicit Euler consensus step:
// x_next = x + gain*dt*(A*x - diag(A*1)*x).
// For a convex-combination update, gain*dt*max_i(sum_j A_ij) must be <= 1.
// The function validates all inputs and only commits output on success.
inline bool consensus_step(const ConsensusInput& input,
                           ConsensusOutput& output,
                           std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const Eigen::Index n = input.states.rows();
  if (n <= 0 || input.states.cols() <= 0)
    return fail("states must have at least one agent and one coordinate");
  if (input.adjacency.rows() != n || input.adjacency.cols() != n)
    return fail("adjacency must be square with one row per agent");
  if (!input.states.allFinite() || !input.adjacency.allFinite())
    return fail("states and adjacency must be finite");
  if (!std::isfinite(input.gain) || input.gain <= 0.0 ||
      !std::isfinite(input.time_step) || input.time_step <= 0.0)
    return fail("gain and time step must be finite and positive");
  if ((input.adjacency.array() < 0.0).any())
    return fail("adjacency weights must be nonnegative");

  Eigen::VectorXd degree = input.adjacency.rowwise().sum();
  const double alpha = input.gain * input.time_step;
  if (!std::isfinite(alpha) || alpha * degree.maxCoeff() > 1.0 + 1e-12)
    return fail("gain*time_step*maximum_degree must not exceed one");

  ConsensusOutput candidate;
  candidate.states = input.states + alpha *
      (input.adjacency * input.states -
       degree.asDiagonal() * input.states);
  if (!candidate.states.allFinite())
    return fail("consensus update produced non-finite states");
  output = std::move(candidate);
  if (error) error->clear();
  return true;
}

}  // namespace mca::multi_robot
