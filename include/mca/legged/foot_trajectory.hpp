#pragma once

#include <cmath>
#include <string>

#include "mca/geometry/se3_so3.hpp"

namespace mca::legged {

struct FootTrajectoryParameters {
  geometry::Vec3 start_position_world{};
  geometry::Vec3 finish_position_world{};
  double swing_height{0.05};
  double duration_s{1.0};
};

struct FootTrajectorySample {
  geometry::Vec3 position_world{};
  geometry::Vec3 velocity_world{};
  geometry::Vec3 acceleration_world{};
};

// Quintic smoothstep interpolation between foot endpoints, with a smooth
// non-negative clearance arc along world Z. Endpoint velocity and acceleration
// are zero; samples outside the swing interval clamp to the endpoint.
inline bool evaluate_foot_trajectory(const FootTrajectoryParameters& parameters,
                                     double time_s,
                                     FootTrajectorySample& output,
                                     std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  const auto finite_vec = [](geometry::Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  };
  if (!finite_vec(parameters.start_position_world) ||
      !finite_vec(parameters.finish_position_world) ||
      !std::isfinite(parameters.swing_height) ||
      !std::isfinite(parameters.duration_s) || !std::isfinite(time_s)) {
    return fail("foot trajectory inputs must be finite");
  }
  if (parameters.swing_height < 0.0 || parameters.duration_s <= 0.0) {
    return fail("swing height must be non-negative and duration positive");
  }

  FootTrajectorySample candidate;
  if (time_s <= 0.0) {
    candidate.position_world = parameters.start_position_world;
    output = candidate;
    return true;
  }
  if (time_s >= parameters.duration_s) {
    candidate.position_world = parameters.finish_position_world;
    output = candidate;
    return true;
  }

  const double u = time_s / parameters.duration_s;
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  const double blend_du = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
  const double blend_ddu = 60.0 * u - 180.0 * u2 + 120.0 * u3;

  // 64*u^3*(1-u)^3 has unit peak at u=0.5 and zero first/second
  // derivatives at both endpoints.
  const double one_minus_u = 1.0 - u;
  const double arc = 64.0 * u3 * one_minus_u * one_minus_u * one_minus_u;
  const double arc_du = 192.0 * u2 * one_minus_u * one_minus_u *
                        (1.0 - 2.0 * u);
  const double arc_ddu = 384.0 * u * one_minus_u *
                         (1.0 - 5.0 * u + 5.0 * u2);
  const geometry::Vec3 delta = parameters.finish_position_world -
                               parameters.start_position_world;
  candidate.position_world = parameters.start_position_world + blend * delta;
  candidate.position_world.z += parameters.swing_height * arc;
  candidate.velocity_world = (blend_du / parameters.duration_s) * delta;
  candidate.velocity_world.z +=
      parameters.swing_height * arc_du / parameters.duration_s;
  const double inv_duration_sq = 1.0 / (parameters.duration_s * parameters.duration_s);
  candidate.acceleration_world =
      (blend_ddu * inv_duration_sq) * delta;
  candidate.acceleration_world.z +=
      parameters.swing_height * arc_ddu * inv_duration_sq;
  if (!finite_vec(candidate.position_world) || !finite_vec(candidate.velocity_world) ||
      !finite_vec(candidate.acceleration_world)) {
    return fail("computed foot trajectory sample is non-finite");
  }
  output = candidate;
  return true;
}

}  // namespace mca::legged
