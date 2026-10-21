#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "mca/geometry/se3_so3.hpp"

namespace mca::legged {

// Point contact against a stationary plane. The plane normal points toward the
// admissible half-space. Gap and velocities are measured along/in that frame.
struct ContactPointState {
  double gap{0.0};  // Positive when separated, negative when penetrating.
  geometry::Vec3 velocity_world{};
  geometry::Vec3 plane_normal_world{0.0, 0.0, 1.0};
};

struct ContactModelParameters {
  double normal_stiffness{10000.0};
  double normal_damping{100.0};
  double friction_coefficient{0.7};
  double tangential_damping{100.0};
};

struct ContactWrench {
  geometry::Vec3 force_world{};
  double normal_force{0.0};
  bool active{false};
};

// Penalty contact with unilateral normal force and regularized Coulomb friction.
// The result is written only on success.
inline bool evaluate_contact(const ContactModelParameters& parameters,
                             const ContactPointState& state,
                             ContactWrench& output,
                             std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const auto finite_vec = [](geometry::Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  };
  if (!std::isfinite(parameters.normal_stiffness) ||
      !std::isfinite(parameters.normal_damping) ||
      !std::isfinite(parameters.friction_coefficient) ||
      !std::isfinite(parameters.tangential_damping) ||
      parameters.normal_stiffness < 0.0 || parameters.normal_damping < 0.0 ||
      parameters.friction_coefficient < 0.0 ||
      parameters.tangential_damping < 0.0) {
    return fail("contact parameters must be finite and non-negative");
  }
  if (!std::isfinite(state.gap) || !finite_vec(state.velocity_world) ||
      !finite_vec(state.plane_normal_world)) {
    return fail("contact state must be finite");
  }
  const double normal_norm = geometry::norm(state.plane_normal_world);
  if (normal_norm <= 1e-12 || !std::isfinite(normal_norm))
    return fail("plane normal must have non-zero finite norm");

  const geometry::Vec3 n = (1.0 / normal_norm) * state.plane_normal_world;
  const double normal_velocity = geometry::dot(state.velocity_world, n);
  const double penetration = std::max(0.0, -state.gap);
  const double normal_force =
      std::max(0.0, parameters.normal_stiffness * penetration -
                        parameters.normal_damping * normal_velocity);
  const geometry::Vec3 tangential_velocity =
      state.velocity_world - normal_velocity * n;
  const double tangential_speed = geometry::norm(tangential_velocity);
  double tangential_force_magnitude = 0.0;
  if (normal_force > 0.0 && tangential_speed > 1e-12) {
    tangential_force_magnitude =
        std::min(parameters.friction_coefficient * normal_force,
                 parameters.tangential_damping * tangential_speed);
  }
  geometry::Vec3 force = normal_force * n;
  if (tangential_force_magnitude > 0.0)
    force = force + (-tangential_force_magnitude / tangential_speed) *
                        tangential_velocity;
  if (!finite_vec(force) || !std::isfinite(normal_force))
    return fail("computed contact force is non-finite");

  ContactWrench candidate;
  candidate.force_world = force;
  candidate.normal_force = normal_force;
  candidate.active = normal_force > 0.0;
  output = candidate;
  return true;
}

}  // namespace mca::legged
