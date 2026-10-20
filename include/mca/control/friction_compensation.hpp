#pragma once

#include <cmath>

#include "mca/core/math.hpp"
#include "mca/core/types.hpp"

namespace mca::control {

struct FrictionCompensationConfig {
  double static_friction_nm{0.0};
  double coulomb_friction_nm{0.0};
  double viscous_friction_nm_per_rad_s{0.0};
  double velocity_deadband_rad_s{1.0e-3};
  double compensation_min_nm{-1.0e9};
  double compensation_max_nm{1.0e9};
};

// Computes the torque command that compensates a simple static/Coulomb/viscous
// friction model. The requested torque selects the compensation direction
// while the axis is inside the velocity deadband, avoiding sign ambiguity at
// zero speed. The runtime path performs no dynamic allocation.
class FrictionCompensator {
 public:
  FrictionCompensator() = default;
  explicit FrictionCompensator(const FrictionCompensationConfig& config) {
    configure(config);
  }

  core::Status configure(const FrictionCompensationConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status compute(double velocity_rad_s, double requested_torque_nm,
                       double& compensation_nm) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(velocity_rad_s) ||
        !core::is_finite(requested_torque_nm)) {
      return {core::StatusCode::InvalidArgument};
    }

    const double abs_velocity = std::abs(velocity_rad_s);
    double compensation =
        config_.viscous_friction_nm_per_rad_s * velocity_rad_s;

    if (abs_velocity > config_.velocity_deadband_rad_s) {
      compensation += config_.coulomb_friction_nm * sign(velocity_rad_s);
    } else if (std::abs(requested_torque_nm) > 0.0) {
      compensation += config_.static_friction_nm * sign(requested_torque_nm);
    }

    compensation_nm = math::clamp(compensation, config_.compensation_min_nm,
                                   config_.compensation_max_nm);
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] const FrictionCompensationConfig& config() const noexcept {
    return config_;
  }

 private:
  static double sign(double value) noexcept {
    return value > 0.0 ? 1.0 : -1.0;
  }

  static bool valid(const FrictionCompensationConfig& c) noexcept {
    return core::is_finite(c.static_friction_nm) &&
           core::is_finite(c.coulomb_friction_nm) &&
           core::is_finite(c.viscous_friction_nm_per_rad_s) &&
           core::is_finite(c.velocity_deadband_rad_s) &&
           core::is_finite(c.compensation_min_nm) &&
           core::is_finite(c.compensation_max_nm) &&
           c.static_friction_nm >= 0.0 && c.coulomb_friction_nm >= 0.0 &&
           c.viscous_friction_nm_per_rad_s >= 0.0 &&
           c.velocity_deadband_rad_s >= 0.0 &&
           c.compensation_min_nm <= c.compensation_max_nm;
  }

  FrictionCompensationConfig config_{};
  bool configured_{false};
};

}  // namespace mca::control
