#pragma once

#include "mca/core/types.hpp"

namespace mca::control {

struct PmsmDqFeedforwardConfig {
  double d_axis_inductance_h{0.001};
  double q_axis_inductance_h{0.001};
  double permanent_magnet_flux_wb{0.05};
  double voltage_d_min{-1.0e9};
  double voltage_d_max{1.0e9};
  double voltage_q_min{-1.0e9};
  double voltage_q_max{1.0e9};
};

struct PmsmDqFeedforwardOutput {
  double d_voltage{0.0};
  double q_voltage{0.0};
};

// Rotor-frame PMSM voltage decoupling/feedforward terms using the P1-07
// dq convention:
//   vd_ff = -we Lq iq
//   vq_ff =  we (Ld id + psi_f)
// Voltage limits are applied independently to the feedforward terms; PI
// output limiting remains the responsibility of the current controller.
class PmsmDqFeedforward {
 public:
  PmsmDqFeedforward() = default;
  explicit PmsmDqFeedforward(const PmsmDqFeedforwardConfig& config) {
    configure(config);
  }

  core::Status configure(const PmsmDqFeedforwardConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status compute(double electrical_angular_velocity,
                       double d_current, double q_current,
                       PmsmDqFeedforwardOutput& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(electrical_angular_velocity) ||
        !core::is_finite(d_current) || !core::is_finite(q_current)) {
      return {core::StatusCode::InvalidArgument};
    }

    const double d_voltage =
        -electrical_angular_velocity * config_.q_axis_inductance_h * q_current;
    const double q_voltage =
        electrical_angular_velocity *
        (config_.d_axis_inductance_h * d_current +
         config_.permanent_magnet_flux_wb);

    output.d_voltage = clamp(d_voltage, config_.voltage_d_min,
                             config_.voltage_d_max);
    output.q_voltage = clamp(q_voltage, config_.voltage_q_min,
                             config_.voltage_q_max);
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] const PmsmDqFeedforwardConfig& config() const noexcept {
    return config_;
  }

 private:
  static double clamp(double value, double minimum, double maximum) noexcept {
    return value < minimum ? minimum : (value > maximum ? maximum : value);
  }

  static bool valid(const PmsmDqFeedforwardConfig& c) noexcept {
    return core::is_finite(c.d_axis_inductance_h) &&
           core::is_finite(c.q_axis_inductance_h) &&
           core::is_finite(c.permanent_magnet_flux_wb) &&
           core::is_finite(c.voltage_d_min) &&
           core::is_finite(c.voltage_d_max) &&
           core::is_finite(c.voltage_q_min) &&
           core::is_finite(c.voltage_q_max) &&
           c.d_axis_inductance_h > 0.0 && c.q_axis_inductance_h > 0.0 &&
           c.permanent_magnet_flux_wb > 0.0 &&
           c.voltage_d_min <= c.voltage_d_max &&
           c.voltage_q_min <= c.voltage_q_max;
  }

  PmsmDqFeedforwardConfig config_{};
  bool configured_{false};
};

}  // namespace mca::control
