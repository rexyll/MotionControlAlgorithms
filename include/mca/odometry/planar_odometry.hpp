#pragma once

#include <cmath>

namespace mca::odometry {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct BodyTwist2D {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

// Integrates a constant body-frame planar twist over one sample interval.
// For nonzero angular velocity this uses the exact SE(2) constant-twist update.
class PlanarOdometry {
 public:
  PlanarOdometry() = default;

  bool reset(const Pose2D& pose = {}) noexcept {
    if (!finite(pose)) return false;
    pose_ = pose;
    return true;
  }

  const Pose2D& pose() const noexcept { return pose_; }

  bool update(const BodyTwist2D& twist, double dt) noexcept {
    if (!finite(twist) || !std::isfinite(dt) || dt <= 0.0) return false;

    const double wz = twist.angular_z;
    const double theta = wz * dt;
    const double c = std::cos(pose_.yaw);
    const double s = std::sin(pose_.yaw);

    double dx_body = twist.linear_x * dt;
    double dy_body = twist.linear_y * dt;
    if (std::abs(wz) > 1e-12) {
      const double sin_theta = std::sin(theta);
      const double one_minus_cos = 1.0 - std::cos(theta);
      dx_body = (twist.linear_x * sin_theta +
                 twist.linear_y * one_minus_cos) / wz;
      dy_body = (twist.linear_x * one_minus_cos +
                 twist.linear_y * sin_theta) / wz;
    }

    Pose2D next = pose_;
    next.x += c * dx_body - s * dy_body;
    next.y += s * dx_body + c * dy_body;
    next.yaw += theta;
    if (!finite(next)) return false;
    pose_ = next;
    return true;
  }

 private:
  static bool finite(const Pose2D& pose) noexcept {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.yaw);
  }

  static bool finite(const BodyTwist2D& twist) noexcept {
    return std::isfinite(twist.linear_x) && std::isfinite(twist.linear_y) &&
           std::isfinite(twist.angular_z);
  }

  Pose2D pose_{};
};

}  // namespace mca::odometry
