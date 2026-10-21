#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "mca/geometry/se3_so3.hpp"

namespace mca::legged {

// Floating-base generalized state. Base position/linear velocity are world-frame;
// orientation maps body vectors to world; angular velocity is body-frame.
// Joint positions and velocities use the same ordered joint-coordinate convention.
struct FloatingBaseState {
  geometry::Vec3 base_position_world{};
  geometry::Quaternion base_orientation_body_to_world{};
  geometry::Vec3 base_linear_velocity_world{};
  geometry::Vec3 base_angular_velocity_body{};
  std::vector<double> joint_positions{};
  std::vector<double> joint_velocities{};
};

inline bool validate_floating_base_state(const FloatingBaseState& state,
                                         std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const auto finite_vec = [](geometry::Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  };
  const auto q = state.base_orientation_body_to_world;
  if (!finite_vec(state.base_position_world) ||
      !finite_vec(state.base_linear_velocity_world) ||
      !finite_vec(state.base_angular_velocity_body)) {
    return fail("base position and velocities must be finite");
  }
  if (!std::isfinite(q.w) || !std::isfinite(q.x) ||
      !std::isfinite(q.y) || !std::isfinite(q.z) ||
      geometry::quaternion_norm(q) <= 1e-12) {
    return fail("base orientation quaternion must have non-zero finite norm");
  }
  if (state.joint_positions.size() != state.joint_velocities.size()) {
    return fail("joint position and velocity sizes must match");
  }
  for (double value : state.joint_positions)
    if (!std::isfinite(value)) return fail("joint positions must be finite");
  for (double value : state.joint_velocities)
    if (!std::isfinite(value)) return fail("joint velocities must be finite");
  return true;
}

}  // namespace mca::legged
