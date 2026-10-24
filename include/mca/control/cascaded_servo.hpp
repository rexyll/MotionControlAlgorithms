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
    // Prepare all loops on local copies so a bad inner-loop configuration
    // cannot leave a previously configured cascade partially reconfigured.
    PidController position_trial = position_;
    PidController velocity_trial = velocity_;
    PidController torque_trial = torque_;

    if (auto status = position_trial.configure(config.position); !status.ok()) {
      return status;
    }
    if (auto status = velocity_trial.configure(config.velocity); !status.ok()) {
      return status;
    }
    if (auto status = torque_trial.configure(config.torque); !status.ok()) {
      return status;
    }

    position_ = position_trial;
    velocity_ = velocity_trial;
    torque_ = torque_trial;
    configured_ = true;
    velocity_reference_ = 0.0;
    torque_reference_ = 0.0;
    actuator_command_ = 0.0;
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

    // Execute the complete cascade on local copies. A failure in an inner
    // loop must not leave an outer loop, intermediate reference, or actuator
    // command partially advanced. Commit only after every loop succeeds.
    PidController position_trial = position_;
    PidController velocity_trial = velocity_;
    PidController torque_trial = torque_;
    double velocity_reference_trial = velocity_reference_;
    double torque_reference_trial = torque_reference_;
    double actuator_command_trial = actuator_command_;

    if (auto status = position_trial.update(position_reference, position, dt,
                                            velocity_reference_trial);
        !status.ok()) {
      return status;
    }
    if (auto status = velocity_trial.update(velocity_reference_trial, velocity, dt,
                                            torque_reference_trial);
        !status.ok()) {
      return status;
    }
    if (auto status = torque_trial.update(torque_reference_trial, torque, dt,
                                          actuator_command_trial);
        !status.ok()) {
      return status;
    }

    position_ = position_trial;
    velocity_ = velocity_trial;
    torque_ = torque_trial;
    velocity_reference_ = velocity_reference_trial;
    torque_reference_ = torque_reference_trial;
    actuator_command_ = actuator_command_trial;
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
