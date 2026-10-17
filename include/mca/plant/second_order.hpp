#pragma once

#include <cmath>

#include "mca/core/types.hpp"

namespace mca::plant {

// Normalized second-order mechanical plant:
//   J*qdd + B*qd + K*q = input
// State is integrated with semi-implicit Euler. This model is intended for
// controller experiments where inertia, damping, and stiffness are explicit.
class SecondOrderPlant {
 public:
  SecondOrderPlant() = default;
  SecondOrderPlant(double inertia, double damping, double stiffness = 0.0) {
    configure(inertia, damping, stiffness);
  }

  core::Status configure(double inertia, double damping, double stiffness = 0.0) noexcept {
    if (!(inertia > 0.0) || damping < 0.0 || stiffness < 0.0 ||
        !core::is_finite(inertia) || !core::is_finite(damping) ||
        !core::is_finite(stiffness)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    inertia_ = inertia;
    damping_ = damping;
    stiffness_ = stiffness;
    configured_ = true;
    position_ = 0.0;
    velocity_ = 0.0;
    return {};
  }

  core::Status reset(double position = 0.0, double velocity = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(position) || !core::is_finite(velocity)) {
      return {core::StatusCode::InvalidArgument};
    }
    position_ = position;
    velocity_ = velocity;
    return {};
  }

  core::Status step(double effort, double dt) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(effort) || !(dt > 0.0) || !core::is_finite(dt)) {
      return {core::StatusCode::InvalidArgument};
    }
    const double acceleration =
        (effort - damping_ * velocity_ - stiffness_ * position_) / inertia_;
    velocity_ += acceleration * dt;
    position_ += velocity_ * dt;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double position() const noexcept { return position_; }
  [[nodiscard]] double velocity() const noexcept { return velocity_; }
  [[nodiscard]] double acceleration(double effort) const noexcept {
    return (effort - damping_ * velocity_ - stiffness_ * position_) / inertia_;
  }
  [[nodiscard]] double inertia() const noexcept { return inertia_; }
  [[nodiscard]] double damping() const noexcept { return damping_; }
  [[nodiscard]] double stiffness() const noexcept { return stiffness_; }

 private:
  double inertia_{1.0};
  double damping_{0.0};
  double stiffness_{0.0};
  double position_{0.0};
  double velocity_{0.0};
  bool configured_{false};
};

}  // namespace mca::plant
