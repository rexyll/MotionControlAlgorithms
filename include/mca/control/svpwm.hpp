#pragma once

#include <algorithm>
#include <cmath>

#include "mca/core/types.hpp"

namespace mca::control {

struct SvpwmConfig {
  double dc_bus_voltage{48.0};
  double pwm_period_s{0.0001};
};

struct SvpwmInput {
  double alpha_voltage{0.0};
  double beta_voltage{0.0};
};

struct SvpwmOutput {
  double duty_a{0.5};
  double duty_b{0.5};
  double duty_c{0.5};
  double t1_s{0.0};
  double t2_s{0.0};
  double t0_s{0.0};
  double requested_magnitude_v{0.0};
  double applied_magnitude_v{0.0};
  bool overmodulated{false};
  int sector{0};
};

class Svpwm {
 public:
  Svpwm() = default;
  explicit Svpwm(const SvpwmConfig& config) { configure(config); }

  core::Status configure(const SvpwmConfig& config) noexcept {
    if (!valid(config)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return {};
  }

  void reset() noexcept {}

  core::Status compute(const SvpwmInput& input,
                       SvpwmOutput& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(input.alpha_voltage) ||
        !core::is_finite(input.beta_voltage)) {
      return {core::StatusCode::InvalidArgument};
    }

    constexpr double kPi = 3.1415926535897932384626433832795;
    constexpr double kTwoPi = 2.0 * kPi;
    constexpr double kSectorAngle = kPi / 3.0;
    constexpr double kSqrt3 = 1.7320508075688772935;

    const double requested =
        std::hypot(input.alpha_voltage, input.beta_voltage);
    const double linear_limit = config_.dc_bus_voltage / kSqrt3;
    const double applied = std::min(requested, linear_limit);

    output = {};
    output.requested_magnitude_v = requested;
    output.applied_magnitude_v = applied;
    output.overmodulated = requested > linear_limit;

    if (applied <= 1.0e-15) {
      output.duty_a = output.duty_b = output.duty_c = 0.5;
      output.t0_s = config_.pwm_period_s;
      output.sector = 0;
      return {};
    }

    double angle = std::atan2(input.beta_voltage, input.alpha_voltage);
    if (angle < 0.0) angle += kTwoPi;

    int sector = static_cast<int>(std::floor(angle / kSectorAngle)) + 1;
    if (sector > 6) sector = 6;
    const double sector_angle = angle -
                                static_cast<double>(sector - 1) * kSectorAngle;

    const double scale = kSqrt3 * applied / config_.dc_bus_voltage;
    double t1 = config_.pwm_period_s * scale *
                std::sin(kSectorAngle - sector_angle);
    double t2 = config_.pwm_period_s * scale * std::sin(sector_angle);

    // The requested vector is first projected onto the linear SVPWM hexagon.
    // Numerical rounding near a sector boundary must not create negative dwell.
    t1 = std::max(0.0, t1);
    t2 = std::max(0.0, t2);
    const double t0 = std::max(0.0, config_.pwm_period_s - t1 - t2);

    output.t1_s = t1;
    output.t2_s = t2;
    output.t0_s = t0;
    output.sector = sector;

    const double half_t0 = 0.5 * t0;
    const double total_active = t1 + t2 + half_t0;
    const double mid = half_t0;

    switch (sector) {
      case 1:
        output.duty_a = (total_active) / config_.pwm_period_s;
        output.duty_b = (t2 + mid) / config_.pwm_period_s;
        output.duty_c = mid / config_.pwm_period_s;
        break;
      case 2:
        output.duty_a = (t1 + mid) / config_.pwm_period_s;
        output.duty_b = total_active / config_.pwm_period_s;
        output.duty_c = mid / config_.pwm_period_s;
        break;
      case 3:
        output.duty_a = mid / config_.pwm_period_s;
        output.duty_b = total_active / config_.pwm_period_s;
        output.duty_c = (t2 + mid) / config_.pwm_period_s;
        break;
      case 4:
        output.duty_a = mid / config_.pwm_period_s;
        output.duty_b = (t1 + mid) / config_.pwm_period_s;
        output.duty_c = total_active / config_.pwm_period_s;
        break;
      case 5:
        output.duty_a = (t2 + mid) / config_.pwm_period_s;
        output.duty_b = mid / config_.pwm_period_s;
        output.duty_c = total_active / config_.pwm_period_s;
        break;
      case 6:
        output.duty_a = total_active / config_.pwm_period_s;
        output.duty_b = mid / config_.pwm_period_s;
        output.duty_c = (t1 + mid) / config_.pwm_period_s;
        break;
      default:
        return {core::StatusCode::InternalError};
    }

    output.duty_a = std::clamp(output.duty_a, 0.0, 1.0);
    output.duty_b = std::clamp(output.duty_b, 0.0, 1.0);
    output.duty_c = std::clamp(output.duty_c, 0.0, 1.0);
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] const SvpwmConfig& config() const noexcept { return config_; }

  [[nodiscard]] static double linear_voltage_limit(double dc_bus_voltage) noexcept {
    return dc_bus_voltage / 1.7320508075688772935;
  }

 private:
  static bool valid(const SvpwmConfig& c) noexcept {
    return core::is_finite(c.dc_bus_voltage) &&
           core::is_finite(c.pwm_period_s) && c.dc_bus_voltage > 0.0 &&
           c.pwm_period_s > 0.0;
  }

  SvpwmConfig config_{};
  bool configured_{false};
};

}  // namespace mca::control
