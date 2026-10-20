#pragma once

#include "mca/core/types.hpp"

namespace mca::control {

struct StateObserverConfig {
  double inertia_kg_m2{1.0};
  double viscous_damping_nm_per_rad_s{0.0};
  double stiffness_nm_per_rad{0.0};
  double position_gain_per_s{20.0};
  double velocity_gain_per_s2{100.0};
};

struct StateObserverOutput {
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

// Continuous-time Luenberger observer for a second-order mechanical axis.
// Plant model:
//   J*qdd + B*qd + K*q = tau
// Measured output is position q. The observer estimates position and velocity:
//   qhat_dot  = vhat + L1*(q - qhat)
//   vhat_dot  = (tau - B*vhat - K*qhat)/J + L2*(q - qhat)
// Gains L1 [1/s] and L2 [1/s^2] are applied with forward Euler integration.
class StateObserver {
 public:
  StateObserver() = default;
  explicit StateObserver(const StateObserverConfig& config) { configure(config); }

  core::Status configure(const StateObserverConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    reset();
    return {};
  }

  core::Status reset(double position = 0.0, double velocity = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(position) || !core::is_finite(velocity)) {
      return {core::StatusCode::InvalidArgument};
    }
    position_hat_ = position;
    velocity_hat_ = velocity;
    return {};
  }

  core::Status update(double torque, double measured_position, double dt,
                      StateObserverOutput& output) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(torque) || !core::is_finite(measured_position) ||
        !(dt > 0.0) || !core::is_finite(dt)) {
      return {core::StatusCode::InvalidArgument};
    }

    const double position_error = measured_position - position_hat_;
    const double model_acceleration =
        (torque - config_.viscous_damping_nm_per_rad_s * velocity_hat_ -
         config_.stiffness_nm_per_rad * position_hat_) /
        config_.inertia_kg_m2;
    const double corrected_velocity_derivative =
        model_acceleration + config_.velocity_gain_per_s2 * position_error;
    const double corrected_position_derivative =
        velocity_hat_ + config_.position_gain_per_s * position_error;

    position_hat_ += corrected_position_derivative * dt;
    velocity_hat_ += corrected_velocity_derivative * dt;

    output.position = position_hat_;
    output.velocity = velocity_hat_;
    output.acceleration = corrected_velocity_derivative;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double position() const noexcept { return position_hat_; }
  [[nodiscard]] double velocity() const noexcept { return velocity_hat_; }
  [[nodiscard]] const StateObserverConfig& config() const noexcept { return config_; }

 private:
  static bool valid(const StateObserverConfig& c) noexcept {
    return core::is_finite(c.inertia_kg_m2) &&
           core::is_finite(c.viscous_damping_nm_per_rad_s) &&
           core::is_finite(c.stiffness_nm_per_rad) &&
           core::is_finite(c.position_gain_per_s) &&
           core::is_finite(c.velocity_gain_per_s2) && c.inertia_kg_m2 > 0.0 &&
           c.viscous_damping_nm_per_rad_s >= 0.0 && c.stiffness_nm_per_rad >= 0.0 &&
           c.position_gain_per_s >= 0.0 && c.velocity_gain_per_s2 >= 0.0;
  }

  StateObserverConfig config_{};
  double position_hat_{0.0};
  double velocity_hat_{0.0};
  bool configured_{false};
};

}  // namespace mca::control
