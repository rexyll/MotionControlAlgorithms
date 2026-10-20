#pragma once

#include "mca/core/math.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct DisturbanceObserverConfig {
  double inertia_kg_m2{0.01};
  double viscous_damping_nm_per_rad_s{0.0};
  double filter_time_constant_s{0.001};
  double disturbance_min_nm{-1.0e9};
  double disturbance_max_nm{1.0e9};
};

struct DisturbanceObserverOutput {
  double estimated_disturbance_nm{0.0};
  double compensated_torque_nm{0.0};
};

// Model-residual disturbance observer for a single mechanical axis.
// d = J*a - tau_command + B*omega, followed by a first-order low-pass filter.
class DisturbanceObserver {
 public:
  DisturbanceObserver() = default;
  explicit DisturbanceObserver(const DisturbanceObserverConfig& config) { configure(config); }

  core::Status configure(const DisturbanceObserverConfig& config) noexcept {
    if (!valid(config)) { configured_ = false; return {core::StatusCode::InvalidArgument}; }
    config_ = config; configured_ = true; reset(); return {};
  }

  core::Status reset(double estimated_disturbance_nm = 0.0,
                     double previous_velocity_rad_s = 0.0) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(estimated_disturbance_nm) || !core::is_finite(previous_velocity_rad_s))
      return {core::StatusCode::InvalidArgument};
    estimated_disturbance_nm_ = math::clamp(estimated_disturbance_nm,
                                             config_.disturbance_min_nm,
                                             config_.disturbance_max_nm);
    previous_velocity_rad_s_ = previous_velocity_rad_s;
    initialized_ = true;
    return {};
  }

  core::Status update(double torque_command_nm, double velocity_rad_s, double dt,
                      DisturbanceObserverOutput& output) noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(torque_command_nm) || !core::is_finite(velocity_rad_s) ||
        !(dt > 0.0) || !core::is_finite(dt)) return {core::StatusCode::InvalidArgument};
    if (!initialized_) reset(0.0, velocity_rad_s);

    const double acceleration = (velocity_rad_s - previous_velocity_rad_s_) / dt;
    const double residual = config_.inertia_kg_m2 * acceleration - torque_command_nm +
                            config_.viscous_damping_nm_per_rad_s * velocity_rad_s;
    const double alpha = dt / (config_.filter_time_constant_s + dt);
    estimated_disturbance_nm_ += alpha * (residual - estimated_disturbance_nm_);
    estimated_disturbance_nm_ = math::clamp(estimated_disturbance_nm_,
                                             config_.disturbance_min_nm,
                                             config_.disturbance_max_nm);
    previous_velocity_rad_s_ = velocity_rad_s;
    output.estimated_disturbance_nm = estimated_disturbance_nm_;
    output.compensated_torque_nm = torque_command_nm - estimated_disturbance_nm_;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] double estimated_disturbance_nm() const noexcept { return estimated_disturbance_nm_; }
  [[nodiscard]] const DisturbanceObserverConfig& config() const noexcept { return config_; }

 private:
  static bool valid(const DisturbanceObserverConfig& c) noexcept {
    return core::is_finite(c.inertia_kg_m2) &&
           core::is_finite(c.viscous_damping_nm_per_rad_s) &&
           core::is_finite(c.filter_time_constant_s) &&
           core::is_finite(c.disturbance_min_nm) && core::is_finite(c.disturbance_max_nm) &&
           c.inertia_kg_m2 > 0.0 && c.viscous_damping_nm_per_rad_s >= 0.0 &&
           c.filter_time_constant_s > 0.0 && c.disturbance_min_nm <= c.disturbance_max_nm;
  }

  DisturbanceObserverConfig config_{};
  double estimated_disturbance_nm_{0.0};
  double previous_velocity_rad_s_{0.0};
  bool configured_{false};
  bool initialized_{false};
};

}  // namespace mca::control
