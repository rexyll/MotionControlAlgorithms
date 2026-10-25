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
    if (!valid(config)) return {core::StatusCode::InvalidArgument};
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
    const double p = static_cast<double>(config_.pole_pairs);
    const double electrical_angle = p * position_rad;
    const double electrical_speed = p * velocity_rad_s;
    const double saliency =
        config_.d_axis_inductance_h - config_.q_axis_inductance_h;
    const double flux_torque =
        config_.permanent_magnet_flux_wb * q_current_a;
    const double saliency_torque =
        saliency * d_current_a * q_current_a;
    const double torque = 1.5 * p * (flux_torque + saliency_torque);
    if (!core::is_finite(electrical_angle) ||
        !core::is_finite(electrical_speed) ||
        !core::is_finite(saliency) || !core::is_finite(flux_torque) ||
        !core::is_finite(saliency_torque) || !core::is_finite(torque)) {
      return {core::StatusCode::NumericalError};
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

    const double p = static_cast<double>(config_.pole_pairs);
    const double electrical_speed = p * velocity_rad_s_;
    const double old_d_current = d_current_a_;
    const double old_q_current = q_current_a_;
    const double d_resistive = config_.resistance_ohm * old_d_current;
    const double d_coupling =
        electrical_speed * config_.q_axis_inductance_h * old_q_current;
    const double q_resistive = config_.resistance_ohm * old_q_current;
    const double q_flux =
        config_.d_axis_inductance_h * old_d_current +
        config_.permanent_magnet_flux_wb;
    const double q_coupling = electrical_speed * q_flux;
    const double d_current_dot =
        (d_voltage_v - d_resistive + d_coupling) /
        config_.d_axis_inductance_h;
    const double q_current_dot =
        (q_voltage_v - q_resistive - q_coupling) /
        config_.q_axis_inductance_h;
    const double next_d_current = old_d_current + d_current_dot * dt;
    const double next_q_current = old_q_current + q_current_dot * dt;
    if (!core::is_finite(electrical_speed) ||
        !core::is_finite(d_resistive) || !core::is_finite(d_coupling) ||
        !core::is_finite(q_resistive) || !core::is_finite(q_flux) ||
        !core::is_finite(q_coupling) || !core::is_finite(d_current_dot) ||
        !core::is_finite(q_current_dot) ||
        !core::is_finite(next_d_current) ||
        !core::is_finite(next_q_current)) {
      return {core::StatusCode::NumericalError};
    }

    const double saliency = config_.d_axis_inductance_h -
                            config_.q_axis_inductance_h;
    const double torque =
        1.5 * p *
        (config_.permanent_magnet_flux_wb * next_q_current +
         saliency * next_d_current * next_q_current);
    const double damping_torque =
        config_.viscous_damping_nm_per_rad_s * velocity_rad_s_;
    const double acceleration =
        (torque - damping_torque - load_torque_nm) /
        config_.inertia_kg_m2;
    const double next_velocity = velocity_rad_s_ + acceleration * dt;
    const double next_position = position_rad_ + next_velocity * dt;
    if (!core::is_finite(saliency) || !core::is_finite(torque) ||
        !core::is_finite(damping_torque) || !core::is_finite(acceleration) ||
        !core::is_finite(next_velocity) || !core::is_finite(next_position) ||
        !core::is_finite(p * next_position)) {
      return {core::StatusCode::NumericalError};
    }

    d_current_a_ = next_d_current;
    q_current_a_ = next_q_current;
    velocity_rad_s_ = next_velocity;
    position_rad_ = next_position;
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

  // Checked diagnostic variant. Output is updated only after the electrical
  // angle derivation has been validated as finite.
  core::Status electrical_angle(double& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const double candidate =
        static_cast<double>(config_.pole_pairs) * position_rad_;
    if (!core::is_finite(candidate)) {
      return {core::StatusCode::NumericalError};
    }
    output = candidate;
    return {};
  }

  // Checked diagnostic variant. Output is updated only after the electrical
  // angular-velocity derivation has been validated as finite.
  core::Status electrical_angular_velocity(double& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const double candidate =
        static_cast<double>(config_.pole_pairs) * velocity_rad_s_;
    if (!core::is_finite(candidate)) {
      return {core::StatusCode::NumericalError};
    }
    output = candidate;
    return {};
  }

  // Checked diagnostic variant. Output is updated only after all derived
  // arithmetic has been validated as finite.
  core::Status electromagnetic_torque(double& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const double p = static_cast<double>(config_.pole_pairs);
    if (!core::is_finite(p)) return {core::StatusCode::NumericalError};
    const double saliency =
        config_.d_axis_inductance_h - config_.q_axis_inductance_h;
    if (!core::is_finite(saliency)) {
      return {core::StatusCode::NumericalError};
    }
    const double flux_torque =
        config_.permanent_magnet_flux_wb * q_current_a_;
    if (!core::is_finite(flux_torque)) {
      return {core::StatusCode::NumericalError};
    }
    const double dq_product = d_current_a_ * q_current_a_;
    if (!core::is_finite(dq_product)) {
      return {core::StatusCode::NumericalError};
    }
    const double saliency_torque = saliency * dq_product;
    if (!core::is_finite(saliency_torque)) {
      return {core::StatusCode::NumericalError};
    }
    const double torque_terms = flux_torque + saliency_torque;
    if (!core::is_finite(torque_terms)) {
      return {core::StatusCode::NumericalError};
    }
    const double scale = 1.5 * p;
    if (!core::is_finite(scale)) return {core::StatusCode::NumericalError};
    const double candidate = scale * torque_terms;
    if (!core::is_finite(candidate)) {
      return {core::StatusCode::NumericalError};
    }
    output = candidate;
    return {};
  }

  // Checked mechanical acceleration diagnostic; this query never mutates state.
  core::Status acceleration(double load_torque_nm, double& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(load_torque_nm)) {
      return {core::StatusCode::InvalidArgument};
    }
    double torque_candidate = 0.0;
    const auto torque_status = electromagnetic_torque(torque_candidate);
    if (!torque_status.ok()) return torque_status;
    const double damping_torque =
        config_.viscous_damping_nm_per_rad_s * velocity_rad_s_;
    if (!core::is_finite(damping_torque)) {
      return {core::StatusCode::NumericalError};
    }
    const double numerator = torque_candidate - damping_torque - load_torque_nm;
    if (!core::is_finite(numerator)) return {core::StatusCode::NumericalError};
    const double candidate = numerator / config_.inertia_kg_m2;
    if (!core::is_finite(candidate)) return {core::StatusCode::NumericalError};
    output = candidate;
    return {};
  }

  // Checked stator electrical input power under this model's dq convention:
  // P = 3/2 * (vd * id + vq * iq). Output/state are transactional on failure.
  core::Status electrical_power(double vd, double vq,
                                double& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(vd) || !core::is_finite(vq)) {
      return {core::StatusCode::InvalidArgument};
    }
    const double d_power = vd * d_current_a_;
    const double q_power = vq * q_current_a_;
    if (!core::is_finite(d_power) || !core::is_finite(q_power)) {
      return {core::StatusCode::NumericalError};
    }
    const double sum = d_power + q_power;
    if (!core::is_finite(sum)) return {core::StatusCode::NumericalError};
    const double candidate = 1.5 * sum;
    if (!core::is_finite(candidate)) {
      return {core::StatusCode::NumericalError};
    }
    output = candidate;
    return {};
  }

  // Checked mechanical shaft-power diagnostic; this query never mutates state.
  core::Status mechanical_power(double& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    double torque_candidate = 0.0;
    const auto torque_status = electromagnetic_torque(torque_candidate);
    if (!torque_status.ok()) return torque_status;
    const double candidate = torque_candidate * velocity_rad_s_;
    if (!core::is_finite(candidate)) {
      return {core::StatusCode::NumericalError};
    }
    output = candidate;
    return {};
  }

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
