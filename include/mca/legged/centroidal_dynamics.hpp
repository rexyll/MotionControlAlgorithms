#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "mca/geometry/se3_so3.hpp"

namespace mca::legged {

// A force expressed in world coordinates and applied at a world-frame point.
struct CentroidalAppliedForce {
  geometry::Vec3 point_world{};
  geometry::Vec3 force_world{};
};

struct CentroidalDynamicsInput {
  double total_mass{1.0};
  geometry::Vec3 center_of_mass_world{};
  geometry::Vec3 gravity_world{0.0, 0.0, -9.81};
  // Contact and other non-gravitational forces. Gravity is applied at the COM.
  std::vector<CentroidalAppliedForce> applied_forces{};
  // Additional free moment about the COM, expressed in world coordinates.
  geometry::Vec3 external_torque_world{};
};

struct CentroidalDynamicsOutput {
  geometry::Vec3 net_force_world{};
  geometry::Vec3 linear_momentum_rate_world{};
  geometry::Vec3 center_of_mass_acceleration_world{};
  geometry::Vec3 angular_momentum_rate_world{};
};

namespace detail {
inline bool centroidal_finite(geometry::Vec3 v) noexcept {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}  // namespace detail

// Computes Newton-Euler centroidal rates in world coordinates:
// F_net = m*g + sum(F_i), c_dotdot = F_net/m, L_dot = tau_ext +
// sum((p_i - c) x F_i). Angular momentum is taken about the current COM.
// This is a force aggregation model, not a full articulated-body dynamics solver.
inline bool evaluate_centroidal_dynamics(
    const CentroidalDynamicsInput& input,
    CentroidalDynamicsOutput& output,
    std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  if (!std::isfinite(input.total_mass) || input.total_mass <= 0.0)
    return fail("total mass must be finite and positive");
  if (!detail::centroidal_finite(input.center_of_mass_world) ||
      !detail::centroidal_finite(input.gravity_world) ||
      !detail::centroidal_finite(input.external_torque_world))
    return fail("COM, gravity and external torque must be finite");
  for (const auto& applied : input.applied_forces) {
    if (!detail::centroidal_finite(applied.point_world) ||
        !detail::centroidal_finite(applied.force_world))
      return fail("applied force points and vectors must be finite");
  }

  CentroidalDynamicsOutput candidate;
  candidate.net_force_world = input.total_mass * input.gravity_world;
  candidate.angular_momentum_rate_world = input.external_torque_world;
  for (const auto& applied : input.applied_forces) {
    candidate.net_force_world =
        candidate.net_force_world + applied.force_world;
    candidate.angular_momentum_rate_world =
        candidate.angular_momentum_rate_world +
        geometry::cross(applied.point_world - input.center_of_mass_world,
                        applied.force_world);
  }
  candidate.linear_momentum_rate_world = candidate.net_force_world;
  candidate.center_of_mass_acceleration_world =
      (1.0 / input.total_mass) * candidate.net_force_world;
  if (!detail::centroidal_finite(candidate.net_force_world) ||
      !detail::centroidal_finite(candidate.linear_momentum_rate_world) ||
      !detail::centroidal_finite(candidate.center_of_mass_acceleration_world) ||
      !detail::centroidal_finite(candidate.angular_momentum_rate_world))
    return fail("computed centroidal dynamics output is non-finite");

  output = std::move(candidate);
  return true;
}

}  // namespace mca::legged
