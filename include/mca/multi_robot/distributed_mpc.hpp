#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace mca::multi_robot {

struct DistributedMpcInput {
  // One row per robot; columns contain a flattened horizon of controls.
  Eigen::MatrixXd control_sequences;
  // Gradient of each robot's local finite-horizon objective at its sequence.
  Eigen::MatrixXd local_gradients;
  // Nonnegative symmetric graph weights; zero entries mean no communication edge.
  Eigen::MatrixXd adjacency;
  Eigen::Index horizon{0};
  Eigen::Index control_dimensions{0};
  double gradient_step{0.01};
  double consensus_gain{0.1};
  double max_abs_control{1.0};
};

struct DistributedMpcOutput {
  // One projected gradient/consensus update of each complete control sequence.
  Eigen::MatrixXd control_sequences;
  // First control vector per robot; apply now and replan at the next sample.
  Eigen::MatrixXd first_controls;
};

// Performs one distributed receding-horizon coordination iteration. The local
// gradient is supplied by the caller's model/prediction cost; a graph Laplacian
// penalty discourages disagreement between connected robots' control plans.
// This is an optimization primitive, not a dynamics model or communication stack.
inline bool distributed_mpc_step(const DistributedMpcInput& input,
                                 DistributedMpcOutput& output,
                                 std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const Eigen::Index agents = input.control_sequences.rows();
  const Eigen::Index columns = input.control_sequences.cols();
  if (agents < 2 || input.horizon <= 0 || input.control_dimensions <= 0)
    return fail("at least two agents and positive horizon/control dimensions are required");
  if (input.horizon > std::numeric_limits<Eigen::Index>::max() /
                          input.control_dimensions ||
      columns != input.horizon * input.control_dimensions)
    return fail("control sequence columns must equal horizon times control dimensions");
  if (input.local_gradients.rows() != agents ||
      input.local_gradients.cols() != columns)
    return fail("local_gradients must match control_sequences dimensions");
  if (input.adjacency.rows() != agents || input.adjacency.cols() != agents)
    return fail("adjacency must be square with one row per agent");
  if (!input.control_sequences.allFinite() || !input.local_gradients.allFinite() ||
      !input.adjacency.allFinite())
    return fail("control sequences, gradients and adjacency must be finite");
  if ((input.adjacency.array() < 0.0).any())
    return fail("adjacency weights must be nonnegative");
  if ((input.adjacency - input.adjacency.transpose()).cwiseAbs().maxCoeff() > 1e-9)
    return fail("adjacency must be symmetric");
  if (input.adjacency.diagonal().cwiseAbs().maxCoeff() > 1e-9)
    return fail("adjacency diagonal must be zero");
  if (!std::isfinite(input.gradient_step) || input.gradient_step <= 0.0 ||
      !std::isfinite(input.consensus_gain) || input.consensus_gain < 0.0 ||
      !std::isfinite(input.max_abs_control) || input.max_abs_control <= 0.0)
    return fail("gradient_step and max_abs_control must be positive; consensus_gain must be nonnegative");

  const Eigen::MatrixXd degree = input.adjacency.rowwise().sum().asDiagonal();
  const Eigen::MatrixXd consensus_gradient =
      (degree - input.adjacency) * input.control_sequences;
  DistributedMpcOutput candidate;
  candidate.control_sequences = input.control_sequences -
      input.gradient_step *
          (input.local_gradients + input.consensus_gain * consensus_gradient);
  candidate.control_sequences =
      candidate.control_sequences.cwiseMax(-input.max_abs_control)
                                  .cwiseMin(input.max_abs_control);
  if (!candidate.control_sequences.allFinite())
    return fail("updated control sequences contain non-finite values");
  candidate.first_controls = candidate.control_sequences.leftCols(input.control_dimensions);
  if (!candidate.first_controls.allFinite())
    return fail("first controls contain non-finite values");
  output = std::move(candidate);
  if (error) error->clear();
  return true;
}

}  // namespace mca::multi_robot
