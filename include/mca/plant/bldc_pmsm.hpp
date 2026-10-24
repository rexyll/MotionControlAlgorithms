#pragma once

#include <algorithm>
#include <cmath>

#include "mca/core/types.hpp"

namespace mca::plant {

enum class ThreePhaseMotorKind { BldcTrapezoidal, PmsmSinusoidal };

struct BldcPmsmConfig {
  ThreePhaseMotorKind kind{ThreePhaseMotorKind::PmsmSinusoidal};
  double phase_resistance_ohm{0.2};
  double phase_inductance_h{0.001};
  double torque_constant_nm_per_a{0.05};
  double back_emf_constant_v_per_rad_s{0.05};
  double inertia_kg_m2{0.001};
  double viscous_damping_nm_per_rad_s{0.0001};
  unsigned pole_pairs{4};
};

// Averaged three-phase motor model. Phase inductance is uncoupled and the
// neutral point is not modeled. This is a plant-level model, not a switching
// inverter or a substitute for the PMSM dq model introduced in P1-07.
class BldcPmsm {
 public:
  BldcPmsm() = default;
  explicit BldcPmsm(const BldcPmsmConfig& config) { configure(config); }

  core::Status configure(const BldcPmsmConfig& config) noexcept {
    if (!valid(config)) return {core::StatusCode::InvalidArgument};
    config_ = config;
    configured_ = true;
    return reset();
  }

  core::Status reset(double mechanical_angle_rad = 0.0,
                     double mechanical_speed_rad_s = 0.0,
                     double ia = 0.0, double ib = 0.0,
                     double ic = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(mechanical_angle_rad) ||
        !core::is_finite(mechanical_speed_rad_s) || !core::is_finite(ia) ||
        !core::is_finite(ib) || !core::is_finite(ic)) {
      return {core::StatusCode::InvalidArgument};
    }
    const double pole_pairs = static_cast<double>(config_.pole_pairs);
    const double electrical_angle = mechanical_angle_rad * pole_pairs;
    const double electrical_speed = mechanical_speed_rad_s * pole_pairs;
    const double base_back_emf =
        config_.back_emf_constant_v_per_rad_s * electrical_speed;
    const double theta = wrap_angle(electrical_angle);
    const double offset = 2.0 * kPi / 3.0;
    const double torque =
        config_.torque_constant_nm_per_a *
        (emf_shape(theta) * ia + emf_shape(theta - offset) * ib +
         emf_shape(theta + offset) * ic);
    if (!core::is_finite(electrical_angle) ||
        !core::is_finite(electrical_speed) ||
        !core::is_finite(base_back_emf) || !core::is_finite(torque)) {
      return {core::StatusCode::NumericalError};
    }

    angle_rad_ = mechanical_angle_rad;
    speed_rad_s_ = mechanical_speed_rad_s;
    currents_[0] = ia;
    currents_[1] = ib;
    currents_[2] = ic;
    return {};
  }

  core::Status step(double va, double vb, double vc, double load_torque_nm,
                    double dt) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(va) || !core::is_finite(vb) ||
        !core::is_finite(vc) || !core::is_finite(load_torque_nm) ||
        !core::is_finite(dt) || !(dt > 0.0)) {
      return {core::StatusCode::InvalidArgument};
    }

    const double theta = electrical_angle();
    const double two_pi_over_three = 2.0 * kPi / 3.0;
    if (!core::is_finite(theta)) return {core::StatusCode::NumericalError};
    const double ea = config_.back_emf_constant_v_per_rad_s *
                      static_cast<double>(config_.pole_pairs) * speed_rad_s_ *
                      emf_shape(theta);
    const double eb = config_.back_emf_constant_v_per_rad_s *
                      static_cast<double>(config_.pole_pairs) * speed_rad_s_ *
                      emf_shape(theta - two_pi_over_three);
    const double ec = config_.back_emf_constant_v_per_rad_s *
                      static_cast<double>(config_.pole_pairs) * speed_rad_s_ *
                      emf_shape(theta + two_pi_over_three);
    if (!core::is_finite(ea) || !core::is_finite(eb) ||
        !core::is_finite(ec)) {
      return {core::StatusCode::NumericalError};
    }

    const double old_a = currents_[0];
    const double old_b = currents_[1];
    const double old_c = currents_[2];
    const double derivative_a =
        (va - config_.phase_resistance_ohm * old_a - ea) /
        config_.phase_inductance_h;
    const double derivative_b =
        (vb - config_.phase_resistance_ohm * old_b - eb) /
        config_.phase_inductance_h;
    const double derivative_c =
        (vc - config_.phase_resistance_ohm * old_c - ec) /
        config_.phase_inductance_h;
    const double next_a = old_a + derivative_a * dt;
    const double next_b = old_b + derivative_b * dt;
    const double next_c = old_c + derivative_c * dt;
    if (!core::is_finite(derivative_a) || !core::is_finite(derivative_b) ||
        !core::is_finite(derivative_c) || !core::is_finite(next_a) ||
        !core::is_finite(next_b) || !core::is_finite(next_c)) {
      return {core::StatusCode::NumericalError};
    }

    const double offset = two_pi_over_three;
    const double torque =
        config_.torque_constant_nm_per_a *
        (emf_shape(theta) * next_a +
         emf_shape(theta - offset) * next_b +
         emf_shape(theta + offset) * next_c);
    const double damping_torque =
        config_.viscous_damping_nm_per_rad_s * speed_rad_s_;
    const double acceleration =
        (torque - damping_torque - load_torque_nm) /
        config_.inertia_kg_m2;
    const double next_speed = speed_rad_s_ + acceleration * dt;
    const double next_angle = angle_rad_ + next_speed * dt;
    if (!core::is_finite(torque) || !core::is_finite(damping_torque) ||
        !core::is_finite(acceleration) || !core::is_finite(next_speed) ||
        !core::is_finite(next_angle) ||
        !core::is_finite(next_angle * static_cast<double>(config_.pole_pairs))) {
      return {core::StatusCode::NumericalError};
    }

    currents_[0] = next_a;
    currents_[1] = next_b;
    currents_[2] = next_c;
    speed_rad_s_ = next_speed;
    angle_rad_ = next_angle;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double mechanical_angle() const noexcept { return angle_rad_; }
  [[nodiscard]] double electrical_angle() const noexcept {
    return wrap_angle(angle_rad_ * static_cast<double>(config_.pole_pairs));
  }
  [[nodiscard]] double speed() const noexcept { return speed_rad_s_; }
  [[nodiscard]] double current_a() const noexcept { return currents_[0]; }
  [[nodiscard]] double current_b() const noexcept { return currents_[1]; }
  [[nodiscard]] double current_c() const noexcept { return currents_[2]; }

  // Checked diagnostic variant. Output is updated only after the electrical
  // angle derivation has been validated as finite; the wrapped angle is then
  // returned using the same convention as the legacy accessor.
  core::Status electrical_angle(double& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const double candidate =
        angle_rad_ * static_cast<double>(config_.pole_pairs);
    if (!core::is_finite(candidate)) {
      return {core::StatusCode::NumericalError};
    }
    const double wrapped = wrap_angle(candidate);
    if (!core::is_finite(wrapped)) {
      return {core::StatusCode::NumericalError};
    }
    output = wrapped;
    return {};
  }

  // Checked diagnostic variant for the mechanical-to-electrical speed
  // conversion used by back-EMF and FOC calculations.
  core::Status electrical_angular_velocity(double& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    const double candidate =
        speed_rad_s_ * static_cast<double>(config_.pole_pairs);
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
    const double electrical_angle_candidate =
        angle_rad_ * static_cast<double>(config_.pole_pairs);
    if (!core::is_finite(electrical_angle_candidate)) {
      return {core::StatusCode::NumericalError};
    }
    const double theta = wrap_angle(electrical_angle_candidate);
    const double offset = 2.0 * kPi / 3.0;
    const double phase_a = emf_shape(theta) * currents_[0];
    const double phase_b = emf_shape(theta - offset) * currents_[1];
    const double phase_c = emf_shape(theta + offset) * currents_[2];
    if (!core::is_finite(phase_a) || !core::is_finite(phase_b) ||
        !core::is_finite(phase_c)) {
      return {core::StatusCode::NumericalError};
    }
    const double phase_sum = phase_a + phase_b + phase_c;
    if (!core::is_finite(phase_sum)) return {core::StatusCode::NumericalError};
    const double candidate = config_.torque_constant_nm_per_a * phase_sum;
    if (!core::is_finite(candidate)) return {core::StatusCode::NumericalError};
    output = candidate;
    return {};
  }
  // Legacy unchecked accessor retained for source compatibility. Prefer the
  // status-returning overload when callers need to handle invalid state.
  [[nodiscard]] double electromagnetic_torque() const noexcept {
    const double theta = electrical_angle();
    const double offset = 2.0 * kPi / 3.0;
    return config_.torque_constant_nm_per_a *
           (emf_shape(theta) * currents_[0] +
            emf_shape(theta - offset) * currents_[1] +
            emf_shape(theta + offset) * currents_[2]);
  }

  [[nodiscard]] const BldcPmsmConfig& config() const noexcept { return config_; }

 private:
  static constexpr double kPi = 3.14159265358979323846;

  static double wrap_angle(double angle) noexcept {
    angle = std::fmod(angle, 2.0 * kPi);
    return angle < 0.0 ? angle + 2.0 * kPi : angle;
  }

  double emf_shape(double angle) const noexcept {
    const double theta = wrap_angle(angle);
    if (config_.kind == ThreePhaseMotorKind::PmsmSinusoidal) {
      return std::sin(theta);
    }
    // 120-degree flat top with 60-degree linear commutation intervals.
    const double pi_over_six = kPi / 6.0;
    const double five_pi_over_six = 5.0 * kPi / 6.0;
    const double seven_pi_over_six = 7.0 * kPi / 6.0;
    const double eleven_pi_over_six = 11.0 * kPi / 6.0;
    if (theta < pi_over_six) return theta / pi_over_six;
    if (theta < five_pi_over_six) return 1.0;
    if (theta < seven_pi_over_six) return 1.0 - (theta - five_pi_over_six) / pi_over_six;
    if (theta < eleven_pi_over_six) return -1.0;
    return -1.0 + (theta - eleven_pi_over_six) / pi_over_six;
  }

  static bool valid(const BldcPmsmConfig& c) noexcept {
    return core::is_finite(c.phase_resistance_ohm) &&
           core::is_finite(c.phase_inductance_h) &&
           core::is_finite(c.torque_constant_nm_per_a) &&
           core::is_finite(c.back_emf_constant_v_per_rad_s) &&
           core::is_finite(c.inertia_kg_m2) &&
           core::is_finite(c.viscous_damping_nm_per_rad_s) &&
           c.phase_resistance_ohm > 0.0 && c.phase_inductance_h > 0.0 &&
           c.torque_constant_nm_per_a > 0.0 &&
           c.back_emf_constant_v_per_rad_s > 0.0 &&
           c.inertia_kg_m2 > 0.0 && c.viscous_damping_nm_per_rad_s >= 0.0 &&
           c.pole_pairs > 0;
  }

  BldcPmsmConfig config_{};
  double angle_rad_{0.0};
  double speed_rad_s_{0.0};
  double currents_[3]{0.0, 0.0, 0.0};
  bool configured_{false};
};

}  // namespace mca::plant
