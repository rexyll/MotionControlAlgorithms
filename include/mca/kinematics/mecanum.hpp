#pragma once

#include <cmath>

namespace mca::kinematics {

struct MecanumWheelSpeeds {
  double front_left{0.0};
  double front_right{0.0};
  double rear_left{0.0};
  double rear_right{0.0};
};

struct MecanumKinematics {
  MecanumKinematics() = default;

  MecanumKinematics(double wheel_radius, double half_length, double half_width) {
    configure(wheel_radius, half_length, half_width);
  }

  bool configure(double wheel_radius, double half_length, double half_width) noexcept {
    if (!std::isfinite(wheel_radius) || !std::isfinite(half_length) ||
        !std::isfinite(half_width) || wheel_radius <= 0.0 ||
        half_length <= 0.0 || half_width <= 0.0) {
      configured_ = false;
      return false;
    }
    wheel_radius_ = wheel_radius;
    half_length_ = half_length;
    half_width_ = half_width;
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  double wheel_radius() const noexcept { return wheel_radius_; }
  double half_length() const noexcept { return half_length_; }
  double half_width() const noexcept { return half_width_; }

  // Standard X-roller mecanum convention:
  // x forward, y left, positive yaw counter-clockwise.
  // half_length and half_width are measured from the robot center.
  bool inverse(double linear_x, double linear_y, double angular_z,
               MecanumWheelSpeeds& wheels) const noexcept {
    if (!configured_ || !std::isfinite(linear_x) || !std::isfinite(linear_y) ||
        !std::isfinite(angular_z)) {
      return false;
    }
    const double geometry = half_length_ + half_width_;
    const double rotational = geometry * angular_z;
    const double inv_radius = 1.0 / wheel_radius_;
    wheels.front_left = (linear_x - linear_y - rotational) * inv_radius;
    wheels.front_right = (linear_x + linear_y + rotational) * inv_radius;
    wheels.rear_left = (linear_x + linear_y - rotational) * inv_radius;
    wheels.rear_right = (linear_x - linear_y + rotational) * inv_radius;
    return finite(wheels);
  }

  bool forward(const MecanumWheelSpeeds& wheels, double& linear_x,
               double& linear_y, double& angular_z) const noexcept {
    if (!configured_ || !finite(wheels)) return false;
    const double radius = wheel_radius_;
    const double geometry = half_length_ + half_width_;
    linear_x = 0.25 * radius * (wheels.front_left + wheels.front_right +
                                wheels.rear_left + wheels.rear_right);
    linear_y = 0.25 * radius * (-wheels.front_left + wheels.front_right +
                                 wheels.rear_left - wheels.rear_right);
    angular_z = radius * (-wheels.front_left + wheels.front_right -
                          wheels.rear_left + wheels.rear_right) /
                (4.0 * geometry);
    return std::isfinite(linear_x) && std::isfinite(linear_y) &&
           std::isfinite(angular_z);
  }

 private:
  static bool finite(const MecanumWheelSpeeds& wheels) noexcept {
    return std::isfinite(wheels.front_left) &&
           std::isfinite(wheels.front_right) && std::isfinite(wheels.rear_left) &&
           std::isfinite(wheels.rear_right);
  }

  double wheel_radius_{0.0};
  double half_length_{0.0};
  double half_width_{0.0};
  bool configured_{false};
};

}  // namespace mca::kinematics
