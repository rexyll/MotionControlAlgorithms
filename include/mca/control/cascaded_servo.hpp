#pragma once

#include "mca/control/pid.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct CascadedServoConfig {
  PidConfig position{};
  PidConfig velocity{};
  PidConfig torque{};
};

// Scalar cascaded position -> velocity -> torque controller.
// The innermost controller output is interpreted as an actuator voltage/input.
// All three loops execute at the caller-provided sample period. Runtime state
// is fixed-size and performs no dynamic allocation.
class CascadedServo {
 public:
  CascadedServo() = default;
  explicit CascadedServo(const CascadedServoConfig& config) { configure(config); }

  core::Status configure(const CascadedServoConfig& config) noexcept {
    if (auto status = position_.configure(config.position); !status.ok()) {
      configured_ = false;
      return status;
    }
    if (auto status = velocity_.configure(config.velocity); !status.ok()) {
      configured_ = false;
      return status;
    }
    if (auto status = torque_.configure(config.torque); !status.ok()) {
      configured_ = false;
      return status;
    }
    configured_ = true;
    reset();
    return {};
  }

  core::Status reset(double position = 0.0, double velocity = 0.0,
                     double torque = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(position) || !core::is_finite(velocity) ||
        !core::is_finite(torque)) {
      return {core::StatusCode::InvalidArgument};
    }
    if (auto status = position_.reset(0.0, position); !status.ok()) return status;
    if (auto status = velocity_.reset(0.0, velocity); !status.ok()) return status;
    if (auto status = torque_.reset(0.0, torque); !status.ok()) return status;
    velocity_reference_ = 0.0;
    torque_reference_ = 0.0;
    actuator_command_ = 0.0;
    return {};
  }

  core::Status update(double position_reference, double position,
                      double velocity, double torque, double dt,
                      double& actuator_command) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(position_reference) || !core::is_finite(position) ||
        !core::is_finite(velocity) || !core::is_finite(torque) ||
        !(dt > 0.0) || !core::is_finite(dt)) {
      return {core::StatusCode::InvalidArgument};
    }

    if (auto status = position_.update(position_reference, position, dt,
                                       velocity_reference_);
        !status.ok()) {
      return status;
    }
    if (auto status = velocity_.update(velocity_reference_, velocity, dt,
                                       torque_reference_);
        !status.ok()) {
      return status;
    }
    if (auto status = torque_.update(torque_reference_, torque, dt,
                                     actuator_command_);
        !status.ok()) {
      return status;
    }

    actuator_command = actuator_command_;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double velocity_reference() const noexcept {
    return velocity_reference_;
  }
  [[nodiscard]] double torque_reference() const noexcept { return torque_reference_; }
  [[nodiscard]] double actuator_command() const noexcept { return actuator_command_; }

 private:
  PidController position_{};
  PidController velocity_{};
  PidController torque_{};
  double velocity_reference_{0.0};
  double torque_reference_{0.0};
  double actuator_command_{0.0};
  bool configured_{false};
};

}  // namespace mca::control
