#pragma once

#include <algorithm>
#include <cmath>

#include "mca/core/types.hpp"
#include "mca/geometry/se3_so3.hpp"

namespace mca::uav {

struct GeometricAttitudeControllerConfig {
  // Diagonal body inertia in kg*m^2; this baseline assumes principal axes.
  geometry::Vec3 inertia_diagonal{0.03, 0.03, 0.05};
  geometry::Vec3 attitude_gain{4.0, 4.0, 2.0};
  geometry::Vec3 angular_rate_gain{0.2, 0.2, 0.15};
  geometry::Vec3 torque_limit{10.0, 10.0, 5.0};
};

struct GeometricAttitudeControllerInput {
  geometry::Quaternion attitude_body_to_world{};
  geometry::Quaternion desired_attitude_body_to_world{};
  geometry::Vec3 angular_velocity_body{};
  geometry::Vec3 desired_angular_velocity_body{};
  geometry::Vec3 feedforward_torque_body{};
};

struct GeometricAttitudeControllerOutput {
  geometry::Vec3 attitude_error_body{};
  geometry::Vec3 torque_body{};
};

// Quaternion-geometric attitude PD with diagonal-inertia gyroscopic
// compensation. No Euler angles are used, avoiding Euler-coordinate
// singularities. The caller owns attitude estimation and actuator allocation.
class GeometricAttitudeController {
 public:
  GeometricAttitudeController() = default;

  core::Status configure(const GeometricAttitudeControllerConfig& config) noexcept {
    if (!finite(config.inertia_diagonal) ||
        config.inertia_diagonal.x <= 0.0 ||
        config.inertia_diagonal.y <= 0.0 ||
        config.inertia_diagonal.z <= 0.0 ||
        !finite_nonnegative(config.attitude_gain) ||
        !finite_nonnegative(config.angular_rate_gain) ||
        !finite_nonnegative(config.torque_limit)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status update(const GeometricAttitudeControllerInput& input,
                      GeometricAttitudeControllerOutput& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!finite(input.attitude_body_to_world) ||
        !finite(input.desired_attitude_body_to_world) ||
        !finite(input.angular_velocity_body) ||
        !finite(input.desired_angular_velocity_body) ||
        !finite(input.feedforward_torque_body)) {
      return {core::StatusCode::InvalidArgument};
    }

    geometry::Quaternion current = input.attitude_body_to_world;
    geometry::Quaternion desired = input.desired_attitude_body_to_world;
    if (!geometry::normalize(current) || !geometry::normalize(desired))
      return {core::StatusCode::InvalidArgument};

    // q_error = conjugate(q_current) * q_desired is the relative rotation
    // expressed in the current body frame. Flip antipodal representations to
    // select the shortest quaternion arc.
    geometry::Quaternion error_q = geometry::normalized(
        geometry::multiply(geometry::conjugate(current), desired));
    if (error_q.w < 0.0) {
      error_q.w = -error_q.w;
      error_q.x = -error_q.x;
      error_q.y = -error_q.y;
      error_q.z = -error_q.z;
    }

    GeometricAttitudeControllerOutput candidate;
    candidate.attitude_error_body = {2.0 * error_q.x, 2.0 * error_q.y,
                                     2.0 * error_q.z};
    const geometry::Vec3 omega = input.angular_velocity_body;
    const geometry::Vec3 inertia_omega{
        config_.inertia_diagonal.x * omega.x,
        config_.inertia_diagonal.y * omega.y,
        config_.inertia_diagonal.z * omega.z};
    const geometry::Vec3 gyro = geometry::cross(omega, inertia_omega);
    const geometry::Vec3 error = candidate.attitude_error_body;
    const geometry::Vec3 rate_error =
        input.desired_angular_velocity_body - input.angular_velocity_body;
    geometry::Vec3 torque{
        config_.attitude_gain.x * error.x +
            config_.angular_rate_gain.x * rate_error.x + gyro.x +
            input.feedforward_torque_body.x,
        config_.attitude_gain.y * error.y +
            config_.angular_rate_gain.y * rate_error.y + gyro.y +
            input.feedforward_torque_body.y,
        config_.attitude_gain.z * error.z +
            config_.angular_rate_gain.z * rate_error.z + gyro.z +
            input.feedforward_torque_body.z};
    torque.x = std::clamp(torque.x, -config_.torque_limit.x, config_.torque_limit.x);
    torque.y = std::clamp(torque.y, -config_.torque_limit.y, config_.torque_limit.y);
    torque.z = std::clamp(torque.z, -config_.torque_limit.z, config_.torque_limit.z);
    if (!finite(candidate.attitude_error_body) || !finite(torque))
      return {core::StatusCode::NumericalError};
    candidate.torque_body = torque;
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  static bool finite(geometry::Vec3 v) noexcept {
    return core::is_finite(v.x) && core::is_finite(v.y) && core::is_finite(v.z);
  }
  static bool finite(geometry::Quaternion q) noexcept {
    return core::is_finite(q.w) && core::is_finite(q.x) &&
           core::is_finite(q.y) && core::is_finite(q.z);
  }
  static bool finite_nonnegative(geometry::Vec3 v) noexcept {
    return finite(v) && v.x >= 0.0 && v.y >= 0.0 && v.z >= 0.0;
  }

  GeometricAttitudeControllerConfig config_{};
  bool configured_{false};
};

}  // namespace mca::uav
