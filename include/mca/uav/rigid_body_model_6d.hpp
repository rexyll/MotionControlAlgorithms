#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <string>

#include "mca/geometry/se3_so3.hpp"

namespace mca::uav {

// Rigid-body state for a six-degree-of-freedom vehicle.
// Position and linear velocity are expressed in the world frame. Attitude is
// the body-to-world quaternion (w,x,y,z); angular velocity is body-frame.
// Applied force and torque are expressed in the body frame. SI units are used.
struct RigidBodyState6D {
  geometry::Vec3 position_world{};
  geometry::Vec3 velocity_world{};
  geometry::Quaternion attitude_body_to_world{};
  geometry::Vec3 angular_velocity_body{};
};

struct RigidBodyInput6D {
  geometry::Vec3 force_body{};
  geometry::Vec3 torque_body{};
};

struct RigidBodyParameters6D {
  double mass{1.0};
  geometry::Mat3 inertia_body{};
  geometry::Vec3 gravity_world{0.0, 0.0, -9.81};
};

struct RigidBodyDerivative6D {
  geometry::Vec3 position_rate_world{};
  geometry::Vec3 velocity_rate_world{};
  geometry::Quaternion attitude_rate{};
  geometry::Vec3 angular_acceleration_body{};
};

class RigidBodyModel6D {
 public:
  // Evaluates Newton-Euler rigid-body equations without integrating time.
  // Returns false on invalid/non-finite input and leaves derivative unchanged.
  static bool derivative(const RigidBodyParameters6D& parameters,
                         const RigidBodyState6D& state,
                         const RigidBodyInput6D& input,
                         RigidBodyDerivative6D& derivative,
                         std::string* error = nullptr) {
    Eigen::Matrix3d inertia;
    if (!valid_parameters(parameters, inertia, error)) return false;
    if (!finite(state.position_world) || !finite(state.velocity_world) ||
        !finite(state.angular_velocity_body) ||
        !finite(state.attitude_body_to_world) ||
        !finite(input.force_body) || !finite(input.torque_body)) {
      return fail("state and input values must be finite", error);
    }

    geometry::Quaternion attitude = state.attitude_body_to_world;
    if (!geometry::normalize(attitude))
      return fail("attitude quaternion must have non-zero finite norm", error);

    const Eigen::Vector3d force(input.force_body.x, input.force_body.y, input.force_body.z);
    const Eigen::Vector3d gravity(parameters.gravity_world.x, parameters.gravity_world.y,
                                  parameters.gravity_world.z);
    const Eigen::Vector3d omega(state.angular_velocity_body.x,
                                state.angular_velocity_body.y,
                                state.angular_velocity_body.z);
    const Eigen::Vector3d torque(input.torque_body.x, input.torque_body.y,
                                 input.torque_body.z);
    const geometry::Vec3 force_world = geometry::rotate(attitude, input.force_body);
    const Eigen::Vector3d acceleration =
        Eigen::Vector3d(force_world.x, force_world.y, force_world.z) /
            parameters.mass + gravity;
    const Eigen::Vector3d angular_momentum = inertia * omega;
    const Eigen::Vector3d angular_acceleration =
        inertia.ldlt().solve(torque - omega.cross(angular_momentum));

    const geometry::Quaternion omega_quaternion{0.0, omega.x(), omega.y(), omega.z()};
    const geometry::Quaternion attitude_rate =
        geometry::multiply(attitude, omega_quaternion);
    RigidBodyDerivative6D candidate;
    candidate.position_rate_world = state.velocity_world;
    candidate.velocity_rate_world = {acceleration.x(), acceleration.y(), acceleration.z()};
    candidate.attitude_rate = {0.5 * attitude_rate.w, 0.5 * attitude_rate.x,
                               0.5 * attitude_rate.y, 0.5 * attitude_rate.z};
    candidate.angular_acceleration_body = {angular_acceleration.x(),
                                           angular_acceleration.y(),
                                           angular_acceleration.z()};
    if (!finite(candidate.position_rate_world) || !finite(candidate.velocity_rate_world) ||
        !finite(candidate.attitude_rate) || !finite(candidate.angular_acceleration_body)) {
      return fail("rigid-body derivative is non-finite", error);
    }
    derivative = candidate;
    return true;
  }

 private:
  static bool finite(geometry::Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }
  static bool finite(geometry::Quaternion q) {
    return std::isfinite(q.w) && std::isfinite(q.x) &&
           std::isfinite(q.y) && std::isfinite(q.z);
  }
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
  static bool valid_parameters(const RigidBodyParameters6D& parameters,
                               Eigen::Matrix3d& inertia, std::string* error) {
    if (!std::isfinite(parameters.mass) || parameters.mass <= 0.0 ||
        !finite(parameters.gravity_world) ||
        !std::all_of(parameters.inertia_body.v.begin(), parameters.inertia_body.v.end(),
                     [](double value) { return std::isfinite(value); })) {
      return fail("mass, inertia, and gravity must be finite and mass positive", error);
    }
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        inertia(r, c) = parameters.inertia_body(r, c);
    if (!inertia.isApprox(inertia.transpose(), 1e-12))
      return fail("body inertia tensor must be symmetric", error);
    Eigen::LLT<Eigen::Matrix3d> factor(inertia);
    if (factor.info() != Eigen::Success)
      return fail("body inertia tensor must be positive definite", error);
    return true;
  }
};

}  // namespace mca::uav
