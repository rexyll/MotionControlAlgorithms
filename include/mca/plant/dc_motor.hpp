#pragma once

#include "mca/core/types.hpp"

namespace mca::plant {

struct DcMotorConfig {
  double resistance_ohm{1.0};
  double inductance_h{0.001};
  double torque_constant_nm_per_a{0.1};
  double back_emf_constant_v_per_rad_s{0.1};
  double inertia_kg_m2{0.001};
  double viscous_damping_nm_per_rad_s{0.0001};
};

// Lumped DC motor model using SI units:
//   L di/dt = V - R i - Ke w
//   J dw/dt = Kt i - B w - tau_load
//   dtheta/dt = w
// The state is integrated with semi-implicit Euler. This model is intended
// for controller/servo experiments, not electrical switching simulation.
class DcMotor {
 public:
  DcMotor() = default;
  explicit DcMotor(const DcMotorConfig& config) { configure(config); }

  core::Status configure(const DcMotorConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return reset();
  }

  core::Status reset(double position = 0.0, double velocity = 0.0,
                     double current = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(position) || !core::is_finite(velocity) ||
        !core::is_finite(current)) {
      return {core::StatusCode::InvalidArgument};
    }
    position_rad_ = position;
    velocity_rad_s_ = velocity;
    current_a_ = current;
    return {};
  }

  core::Status step(double voltage_v, double load_torque_nm,
                    double dt) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(voltage_v) || !core::is_finite(load_torque_nm) ||
        !(dt > 0.0) || !core::is_finite(dt)) {
      return {core::StatusCode::InvalidArgument};
    }

    const double current_dot =
        (voltage_v - config_.resistance_ohm * current_a_ -
         config_.back_emf_constant_v_per_rad_s * velocity_rad_s_) /
        config_.inductance_h;
    current_a_ += current_dot * dt;

    const double torque_nm = config_.torque_constant_nm_per_a * current_a_;
    const double acceleration =
        (torque_nm - config_.viscous_damping_nm_per_rad_s * velocity_rad_s_ -
         load_torque_nm) /
        config_.inertia_kg_m2;
    velocity_rad_s_ += acceleration * dt;
    position_rad_ += velocity_rad_s_ * dt;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double position() const noexcept { return position_rad_; }
  [[nodiscard]] double velocity() const noexcept { return velocity_rad_s_; }
  [[nodiscard]] double current() const noexcept { return current_a_; }
  [[nodiscard]] double torque() const noexcept {
    return config_.torque_constant_nm_per_a * current_a_;
  }
  [[nodiscard]] const DcMotorConfig& config() const noexcept { return config_; }

 private:
  static bool valid(const DcMotorConfig& c) noexcept {
    return core::is_finite(c.resistance_ohm) &&
           core::is_finite(c.inductance_h) &&
           core::is_finite(c.torque_constant_nm_per_a) &&
           core::is_finite(c.back_emf_constant_v_per_rad_s) &&
           core::is_finite(c.inertia_kg_m2) &&
           core::is_finite(c.viscous_damping_nm_per_rad_s) &&
           c.resistance_ohm > 0.0 && c.inductance_h > 0.0 &&
           c.torque_constant_nm_per_a > 0.0 &&
           c.back_emf_constant_v_per_rad_s > 0.0 &&
           c.inertia_kg_m2 > 0.0 && c.viscous_damping_nm_per_rad_s >= 0.0;
  }

  DcMotorConfig config_{};
  double position_rad_{0.0};
  double velocity_rad_s_{0.0};
  double current_a_{0.0};
  bool configured_{false};
};

}  // namespace mca::plant
