#pragma once

#include <cmath>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace mca::multi_robot {

struct LeaderFollowerInput {
  // One row per agent; the leader is included in positions.
  Eigen::MatrixXd positions;
  // Desired follower positions expressed as offsets from the leader.
  // The leader row is ignored.
  Eigen::MatrixXd desired_offsets;
  Eigen::Index leader_index{0};
  // Externally planned leader velocity; it is passed through unchanged.
  Eigen::VectorXd leader_velocity;
  double gain{1.0};
  // Applies to follower commands; leader command remains externally owned.
  double max_follower_speed{1.0};
};

struct LeaderFollowerOutput {
  // One velocity row per agent. The leader row equals leader_velocity.
  Eigen::MatrixXd velocities;
};

// Leader-follower kinematic tracking:
// v_i = v_leader + gain * ((p_leader + offset_i) - p_i), i != leader.
// Each follower command is independently saturated. This primitive does not
// plan the leader trajectory, prevent collisions, or model communication faults.
inline bool leader_follower_control(const LeaderFollowerInput& input,
                                    LeaderFollowerOutput& output,
                                    std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const Eigen::Index n = input.positions.rows();
  const Eigen::Index dimensions = input.positions.cols();
  if (n < 2 || dimensions <= 0)
    return fail("positions must contain at least two agents and one coordinate");
  if (input.leader_index < 0 || input.leader_index >= n)
    return fail("leader_index must identify an agent");
  if (input.desired_offsets.rows() != n ||
      input.desired_offsets.cols() != dimensions)
    return fail("desired_offsets must match positions dimensions");
  if (input.leader_velocity.size() != dimensions)
    return fail("leader_velocity must match the coordinate dimension");
  if (!input.positions.allFinite() || !input.desired_offsets.allFinite() ||
      !input.leader_velocity.allFinite())
    return fail("positions, offsets and leader velocity must be finite");
  if (!std::isfinite(input.gain) || input.gain <= 0.0 ||
      !std::isfinite(input.max_follower_speed) ||
      input.max_follower_speed <= 0.0)
    return fail("gain and max_follower_speed must be finite and positive");

  LeaderFollowerOutput candidate;
  candidate.velocities = Eigen::MatrixXd::Zero(n, dimensions);
  candidate.velocities.row(input.leader_index) = input.leader_velocity.transpose();
  const Eigen::RowVectorXd leader_position =
      input.positions.row(input.leader_index);
  for (Eigen::Index i = 0; i < n; ++i) {
    if (i == input.leader_index) continue;
    const Eigen::RowVectorXd desired_position =
        leader_position + input.desired_offsets.row(i);
    Eigen::RowVectorXd command =
        input.leader_velocity.transpose() +
        input.gain * (desired_position - input.positions.row(i));
    const double speed = command.norm();
    if (!std::isfinite(speed))
      return fail("leader-follower command produced a non-finite speed");
    if (speed > input.max_follower_speed)
      command *= input.max_follower_speed / speed;
    candidate.velocities.row(i) = command;
  }
  if (!candidate.velocities.allFinite())
    return fail("leader-follower command contains non-finite values");
  output = std::move(candidate);
  if (error) error->clear();
  return true;
}

}  // namespace mca::multi_robot
