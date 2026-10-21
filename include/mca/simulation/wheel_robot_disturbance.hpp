#pragma once

#include <cmath>

namespace mca::simulation {

struct PlanarTwistCommand {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

struct WheelRobotDisturbanceConfig {
  // Fraction of commanded motion lost to slip, each constrained to [0, 1].
  double longitudinal_slip{0.0};
  double lateral_slip{0.0};
  double yaw_slip{0.0};
  // Additive sinusoidal disturbance amplitudes (m/s, m/s, rad/s).
  double disturbance_x_amplitude{0.0};
  double disturbance_y_amplitude{0.0};
  double disturbance_yaw_amplitude{0.0};
  // Frequencies (Hz) and phases (rad) for each disturbance channel.
  double disturbance_x_frequency{0.0};
  double disturbance_y_frequency{0.0};
  double disturbance_yaw_frequency{0.0};
  double disturbance_x_phase{0.0};
  double disturbance_y_phase{0.0};
  double disturbance_yaw_phase{0.0};
};

// Deterministic planar command-to-motion model for repeatable slip and
// disturbance experiments. It is independent of ROS2, Gazebo and hardware.
class WheelRobotDisturbanceModel {
 public:
  bool configure(const WheelRobotDisturbanceConfig& config) noexcept {
    if (!valid(config)) return false;
    config_ = config;
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  const WheelRobotDisturbanceConfig& config() const noexcept { return config_; }

  bool compute(const PlanarTwistCommand& command, double time_s,
               PlanarTwistCommand& actual) const noexcept {
    if (!configured_ || !finite(command) || !std::isfinite(time_s) || time_s < 0.0) {
      return false;
    }
    constexpr double kTwoPi = 6.283185307179586476925286766559;
    PlanarTwistCommand next;
    next.linear_x = command.linear_x * (1.0 - config_.longitudinal_slip) +
        sinusoid(config_.disturbance_x_amplitude,
                 config_.disturbance_x_frequency, config_.disturbance_x_phase, time_s);
    next.linear_y = command.linear_y * (1.0 - config_.lateral_slip) +
        sinusoid(config_.disturbance_y_amplitude,
                 config_.disturbance_y_frequency, config_.disturbance_y_phase, time_s);
    next.angular_z = command.angular_z * (1.0 - config_.yaw_slip) +
        sinusoid(config_.disturbance_yaw_amplitude,
                 config_.disturbance_yaw_frequency, config_.disturbance_yaw_phase, time_s);
    (void)kTwoPi;
    if (!finite(next)) return false;
    actual = next;
    return true;
  }

 private:
  static double sinusoid(double amplitude, double frequency, double phase,
                         double time_s) noexcept {
    constexpr double kTwoPi = 6.283185307179586476925286766559;
    return amplitude * std::sin(kTwoPi * frequency * time_s + phase);
  }

  static bool finite(const PlanarTwistCommand& value) noexcept {
    return std::isfinite(value.linear_x) && std::isfinite(value.linear_y) &&
           std::isfinite(value.angular_z);
  }

  static bool valid(const WheelRobotDisturbanceConfig& c) noexcept {
    const double values[] = {c.longitudinal_slip, c.lateral_slip, c.yaw_slip,
      c.disturbance_x_amplitude, c.disturbance_y_amplitude,
      c.disturbance_yaw_amplitude, c.disturbance_x_frequency,
      c.disturbance_y_frequency, c.disturbance_yaw_frequency,
      c.disturbance_x_phase, c.disturbance_y_phase, c.disturbance_yaw_phase};
    for (double value : values) if (!std::isfinite(value)) return false;
    return c.longitudinal_slip >= 0.0 && c.longitudinal_slip <= 1.0 &&
      c.lateral_slip >= 0.0 && c.lateral_slip <= 1.0 &&
      c.yaw_slip >= 0.0 && c.yaw_slip <= 1.0 &&
      c.disturbance_x_frequency >= 0.0 && c.disturbance_y_frequency >= 0.0 &&
      c.disturbance_yaw_frequency >= 0.0;
  }

  WheelRobotDisturbanceConfig config_{};
  bool configured_{false};
};

}  // namespace mca::simulation
