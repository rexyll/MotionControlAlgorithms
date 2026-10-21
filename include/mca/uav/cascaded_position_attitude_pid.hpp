#pragma once

#include <array>
#include <algorithm>
#include <cmath>

#include "mca/control/pid.hpp"
#include "mca/geometry/se3_so3.hpp"

namespace mca::uav {

// Independent axis PID loops: world-frame position produces desired linear
// acceleration; body-frame attitude error produces body torque commands.
// The caller is responsible for motor/thrust allocation and actuator limits.
struct CascadedPositionAttitudePidConfig {
  double mass{1.0};
  geometry::Vec3 gravity_world{0.0, 0.0, -9.81};
  std::array<control::PidConfig, 3> position_pid{};
  std::array<control::PidConfig, 3> attitude_pid{};
};

struct CascadedPositionAttitudePidInput {
  geometry::Vec3 position_setpoint_world{};
  geometry::Quaternion attitude_setpoint_body_to_world{};
  geometry::Vec3 position_world{};
  geometry::Quaternion attitude_body_to_world{};
  double dt{0.001};
};

struct CascadedPositionAttitudePidOutput {
  geometry::Vec3 desired_acceleration_world{};
  geometry::Vec3 force_body{};
  geometry::Vec3 torque_body{};
};

class CascadedPositionAttitudePid {
 public:
  CascadedPositionAttitudePid() = default;

  core::Status configure(const CascadedPositionAttitudePidConfig& config) noexcept {
    if (!core::is_finite(config.mass) || config.mass <= 0.0 ||
        !finite(config.gravity_world)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    auto position = position_pid_;
    auto attitude = attitude_pid_;
    for (std::size_t i = 0; i < 3; ++i) {
      if (!position[i].configure(config.position_pid[i]).ok() ||
          !attitude[i].configure(config.attitude_pid[i]).ok()) {
        configured_ = false;
        return {core::StatusCode::InvalidArgument};
      }
    }
    config_ = config;
    position_pid_ = position;
    attitude_pid_ = attitude;
    configured_ = true;
    return {};
  }

  core::Status reset() noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    for (auto& pid : position_pid_)
      if (!pid.reset().ok()) return {core::StatusCode::RuntimeError};
    for (auto& pid : attitude_pid_)
      if (!pid.reset().ok()) return {core::StatusCode::RuntimeError};
    return {};
  }

  core::Status update(const CascadedPositionAttitudePidInput& input,
                      CascadedPositionAttitudePidOutput& output) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!finite(input.position_setpoint_world) || !finite(input.position_world) ||
        !finite(input.attitude_setpoint_body_to_world) ||
        !finite(input.attitude_body_to_world) || !core::is_finite(input.dt) ||
        input.dt <= 0.0) {
      return {core::StatusCode::InvalidArgument};
    }

    geometry::Quaternion attitude = input.attitude_body_to_world;
    geometry::Quaternion desired_attitude = input.attitude_setpoint_body_to_world;
    if (!geometry::normalize(attitude) || !geometry::normalize(desired_attitude))
      return {core::StatusCode::InvalidArgument};

    const geometry::Vec3 position_setpoint = input.position_setpoint_world;
    const geometry::Vec3 position = input.position_world;
    const auto current_euler = quaternion_to_euler(attitude);
    const auto desired_euler = quaternion_to_euler(desired_attitude);

    CascadedPositionAttitudePidOutput candidate;
    double acceleration[3]{};
    const double setpoint_position[3]{position_setpoint.x, position_setpoint.y,
                                      position_setpoint.z};
    const double measured_position[3]{position.x, position.y, position.z};
    double measured_angle[3]{current_euler[0], current_euler[1], current_euler[2]};
    const double desired_angle[3]{desired_euler[0], desired_euler[1], desired_euler[2]};
    double torque[3]{};
    for (std::size_t i = 0; i < 3; ++i) {
      if (!position_pid_[i].update(setpoint_position[i], measured_position[i],
                                   input.dt, acceleration[i]).ok())
        return {core::StatusCode::InvalidArgument};
      const double wrapped_setpoint =
          measured_angle[i] + wrap_angle(desired_angle[i] - measured_angle[i]);
      if (!attitude_pid_[i].update(wrapped_setpoint, measured_angle[i],
                                   input.dt, torque[i]).ok())
        return {core::StatusCode::InvalidArgument};
    }

    candidate.desired_acceleration_world = {acceleration[0], acceleration[1], acceleration[2]};
    const geometry::Vec3 force_world{
        config_.mass * (acceleration[0] - config_.gravity_world.x),
        config_.mass * (acceleration[1] - config_.gravity_world.y),
        config_.mass * (acceleration[2] - config_.gravity_world.z)};
    candidate.force_body = geometry::rotate(geometry::conjugate(attitude), force_world);
    candidate.torque_body = {torque[0], torque[1], torque[2]};
    if (!finite(candidate.desired_acceleration_world) || !finite(candidate.force_body) ||
        !finite(candidate.torque_body))
      return {core::StatusCode::NumericalError};
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
  static double wrap_angle(double angle) noexcept {
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi) angle -= 2.0 * pi;
    while (angle < -pi) angle += 2.0 * pi;
    return angle;
  }
  static std::array<double, 3> quaternion_to_euler(geometry::Quaternion q) noexcept {
    const double sin_pitch = std::max(-1.0, std::min(
        1.0, 2.0 * (q.w * q.y - q.z * q.x)));
    return {
      std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                 1.0 - 2.0 * (q.x * q.x + q.y * q.y)),
      std::asin(sin_pitch),
      std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    };
  }

  CascadedPositionAttitudePidConfig config_{};
  std::array<control::PidController, 3> position_pid_{};
  std::array<control::PidController, 3> attitude_pid_{};
  bool configured_{false};
};

}  // namespace mca::uav
