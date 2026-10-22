#pragma once

#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace mca::multi_robot {

struct FormationInput {
  // One row per robot; columns are planar or spatial coordinates.
  Eigen::MatrixXd positions;
  // Desired shape offsets. A common translation of all rows is irrelevant.
  Eigen::MatrixXd desired_offsets;
  // Nonnegative directed edge weights; adjacency(i,j) weights robot j in i's control.
  Eigen::MatrixXd adjacency;
  double gain{1.0};
  double max_speed{1.0};
};

struct FormationOutput {
  // Per-agent velocity command, with each row independently speed-limited.
  Eigen::MatrixXd velocities;
};

// Relative-formation velocity law:
// v_i = gain * sum_j a_ij * ((p_j-p_i) - (r_j-r_i)).
// For a connected undirected graph and consistent offsets, this drives relative
// position errors toward zero. It does not regulate the formation's global pose.
inline bool formation_control(const FormationInput& input,
                              FormationOutput& output,
                              std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const Eigen::Index n = input.positions.rows();
  const Eigen::Index dimensions = input.positions.cols();
  if (n < 2 || dimensions <= 0)
    return fail("positions must contain at least two agents and one coordinate");
  if (input.desired_offsets.rows() != n ||
      input.desired_offsets.cols() != dimensions)
    return fail("desired_offsets must match positions dimensions");
  if (input.adjacency.rows() != n || input.adjacency.cols() != n)
    return fail("adjacency must be square with one row per agent");
  if (!input.positions.allFinite() || !input.desired_offsets.allFinite() ||
      !input.adjacency.allFinite())
    return fail("positions, offsets and adjacency must be finite");
  if ((input.adjacency.array() < 0.0).any())
    return fail("adjacency weights must be nonnegative");
  if (!std::isfinite(input.gain) || input.gain <= 0.0 ||
      !std::isfinite(input.max_speed) || input.max_speed <= 0.0)
    return fail("gain and max_speed must be finite and positive");

  FormationOutput candidate;
  candidate.velocities = Eigen::MatrixXd::Zero(n, dimensions);
  for (Eigen::Index i = 0; i < n; ++i) {
    for (Eigen::Index j = 0; j < n; ++j) {
      const double weight = input.adjacency(i, j);
      if (weight == 0.0 || i == j) continue;
      const Eigen::RowVectorXd relative_position =
          input.positions.row(j) - input.positions.row(i);
      const Eigen::RowVectorXd relative_offset =
          input.desired_offsets.row(j) - input.desired_offsets.row(i);
      candidate.velocities.row(i) +=
          input.gain * weight * (relative_position - relative_offset);
    }
    const double speed = candidate.velocities.row(i).norm();
    if (!std::isfinite(speed))
      return fail("formation command produced a non-finite speed");
    if (speed > input.max_speed)
      candidate.velocities.row(i) *= input.max_speed / speed;
  }
  if (!candidate.velocities.allFinite())
    return fail("formation command contains non-finite values");
  output = std::move(candidate);
  if (error) error->clear();
  return true;
}

}  // namespace mca::multi_robot
