#pragma once

#include "mca/core/types.hpp"

namespace mca::plant {

struct PmsmDqConfig {
  double resistance_ohm{0.5};
  double d_axis_inductance_h{0.001};
  double q_axis_inductance_h{0.001};
  double permanent_magnet_flux_wb{0.05};
  double inertia_kg_m2{0.001};
  double viscous_damping_nm_per_rad_s{0.0001};
  int pole_pairs{4};
};

// Rotor-frame sinusoidal PMSM model using the P1-06 Park convention:
//   vd = R id + Ld did/dt - we Lq iq
//   vq = R iq + Lq diq/dt + we (Ld id + psi_f)
//   Te = 3/2 p [psi_f iq + (Ld-Lq) id iq]
//   J dw/dt = Te - B w - tau_load
//   dtheta/dt = w
//   theta_e = p theta
// Semi-implicit Euler is used for integration. This is a plant model for
// controller experiments, not a switching-inverter or magnetic saturation model.
class PmsmDq {
 public:
  PmsmDq() = default;
  explicit PmsmDq(const PmsmDqConfig& config) { configure(config); }

  core::Status configure(const PmsmDqConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return reset();
  }

  core::Status reset(double position_rad = 0.0, double velocity_rad_s = 0.0,
                     double d_current_a = 0.0,
                     double q_current_a = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(position_rad) || !core::is_finite(velocity_rad_s) ||
        !core::is_finite(d_current_a) || !core::is_finite(q_current_a)) {
      return {core::StatusCode::InvalidArgument};
    }
    position_rad_ = position_rad;
    velocity_rad_s_ = velocity_rad_s;
    d_current_a_ = d_current_a;
    q_current_a_ = q_current_a;
    return {};
  }

  core::Status step(double d_voltage_v, double q_voltage_v,
                    double load_torque_nm, double dt) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(d_voltage_v) || !core::is_finite(q_voltage_v) ||
        !core::is_finite(load_torque_nm) || !(dt > 0.0) ||
        !core::is_finite(dt)) {
      return {core::StatusCode::InvalidArgument};
    }

    const double electrical_speed = electrical_angular_velocity();
    const double d_current_dot =
        (d_voltage_v - config_.resistance_ohm * d_current_a_ +
         electrical_speed * config_.q_axis_inductance_h * q_current_a_) /
        config_.d_axis_inductance_h;
    const double q_current_dot =
        (q_voltage_v - config_.resistance_ohm * q_current_a_ -
         electrical_speed *
             (config_.d_axis_inductance_h * d_current_a_ +
              config_.permanent_magnet_flux_wb)) /
        config_.q_axis_inductance_h;

    d_current_a_ += d_current_dot * dt;
    q_current_a_ += q_current_dot * dt;

    const double acceleration =
        (electromagnetic_torque() -
         config_.viscous_damping_nm_per_rad_s * velocity_rad_s_ -
         load_torque_nm) /
        config_.inertia_kg_m2;
    velocity_rad_s_ += acceleration * dt;
    position_rad_ += velocity_rad_s_ * dt;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double position() const noexcept { return position_rad_; }
  [[nodiscard]] double velocity() const noexcept { return velocity_rad_s_; }
  [[nodiscard]] double electrical_angle() const noexcept {
    return static_cast<double>(config_.pole_pairs) * position_rad_;
  }
  [[nodiscard]] double electrical_angular_velocity() const noexcept {
    return static_cast<double>(config_.pole_pairs) * velocity_rad_s_;
  }
  [[nodiscard]] double d_current() const noexcept { return d_current_a_; }
  [[nodiscard]] double q_current() const noexcept { return q_current_a_; }
  [[nodiscard]] double electromagnetic_torque() const noexcept {
    const double p = static_cast<double>(config_.pole_pairs);
    return 1.5 * p *
           (config_.permanent_magnet_flux_wb * q_current_a_ +
            (config_.d_axis_inductance_h - config_.q_axis_inductance_h) *
                d_current_a_ * q_current_a_);
  }
  [[nodiscard]] const PmsmDqConfig& config() const noexcept { return config_; }

 private:
  static bool valid(const PmsmDqConfig& c) noexcept {
    return core::is_finite(c.resistance_ohm) &&
           core::is_finite(c.d_axis_inductance_h) &&
           core::is_finite(c.q_axis_inductance_h) &&
           core::is_finite(c.permanent_magnet_flux_wb) &&
           core::is_finite(c.inertia_kg_m2) &&
           core::is_finite(c.viscous_damping_nm_per_rad_s) &&
           c.resistance_ohm > 0.0 && c.d_axis_inductance_h > 0.0 &&
           c.q_axis_inductance_h > 0.0 && c.permanent_magnet_flux_wb > 0.0 &&
           c.inertia_kg_m2 > 0.0 && c.viscous_damping_nm_per_rad_s >= 0.0 &&
           c.pole_pairs > 0;
  }

  PmsmDqConfig config_{};
  double position_rad_{0.0};
  double velocity_rad_s_{0.0};
  double d_current_a_{0.0};
  double q_current_a_{0.0};
  bool configured_{false};
};

}  // namespace mca::plant
