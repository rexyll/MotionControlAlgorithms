#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "mca/core/types.hpp"
#include "mca/geometry/se3_so3.hpp"

namespace mca::uav {

struct QuadrotorMixerConfig {
  double arm_length{0.2};
  double yaw_torque_per_thrust{0.02};
  double max_rotor_thrust{10.0};
};

struct QuadrotorWrench {
  double thrust{0.0};
  geometry::Vec3 torque{};
};

struct QuadrotorMixerOutput {
  std::array<double, 4> rotor_thrust{};
  QuadrotorWrench achieved{};
  double torque_scale{1.0};
};

// X-configuration quadrotor mixer. Rotor order is front-left, rear-right,
// front-right, rear-left. Positive roll/pitch/yaw follow right-handed body axes.
// Collective thrust is prioritized; requested moments are uniformly desaturated.
class QuadrotorMixer {
 public:
  core::Status configure(const QuadrotorMixerConfig& config) noexcept {
    if (!core::is_finite(config.arm_length) || config.arm_length <= 0.0 ||
        !core::is_finite(config.yaw_torque_per_thrust) ||
        config.yaw_torque_per_thrust <= 0.0 ||
        !core::is_finite(config.max_rotor_thrust) || config.max_rotor_thrust <= 0.0) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return {};
  }

  core::Status allocate(const QuadrotorWrench& requested,
                        QuadrotorMixerOutput& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!core::is_finite(requested.thrust) ||
        !core::is_finite(requested.torque.x) ||
        !core::is_finite(requested.torque.y) ||
        !core::is_finite(requested.torque.z) ||
        requested.thrust < 0.0 ||
        requested.thrust > 4.0 * config_.max_rotor_thrust) {
      return {core::StatusCode::InvalidArgument};
    }

    const double base = requested.thrust / 4.0;
    const double roll = requested.torque.x / (4.0 * config_.arm_length);
    const double pitch = requested.torque.y / (4.0 * config_.arm_length);
    const double yaw = requested.torque.z / (4.0 * config_.yaw_torque_per_thrust);
    auto mix = [&](double scale) noexcept {
      return std::array<double, 4>{
          base - scale * roll + scale * pitch - scale * yaw,
          base + scale * roll - scale * pitch - scale * yaw,
          base - scale * roll - scale * pitch + scale * yaw,
          base + scale * roll + scale * pitch + scale * yaw};
    };

    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < 48; ++i) {
      const double mid = 0.5 * (lo + hi);
      const auto candidate = mix(mid);
      bool feasible = true;
      for (double thrust : candidate) {
        if (thrust < 0.0 || thrust > config_.max_rotor_thrust) {
          feasible = false;
          break;
        }
      }
      if (feasible) lo = mid;
      else hi = mid;
    }

    QuadrotorMixerOutput candidate;
    candidate.rotor_thrust = mix(lo);
    candidate.torque_scale = lo;
    for (double thrust : candidate.rotor_thrust) {
      if (!core::is_finite(thrust) || thrust < -1e-10 ||
          thrust > config_.max_rotor_thrust + 1e-10) {
        return {core::StatusCode::NumericalError};
      }
    }
    candidate.achieved.thrust = candidate.rotor_thrust[0] +
                                candidate.rotor_thrust[1] +
                                candidate.rotor_thrust[2] +
                                candidate.rotor_thrust[3];
    candidate.achieved.torque = {
        config_.arm_length * (-candidate.rotor_thrust[0] + candidate.rotor_thrust[1] -
                              candidate.rotor_thrust[2] + candidate.rotor_thrust[3]),
        config_.arm_length * (candidate.rotor_thrust[0] - candidate.rotor_thrust[1] -
                              candidate.rotor_thrust[2] + candidate.rotor_thrust[3]),
        config_.yaw_torque_per_thrust * (-candidate.rotor_thrust[0] -
                                         candidate.rotor_thrust[1] +
                                         candidate.rotor_thrust[2] +
                                         candidate.rotor_thrust[3])};
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }

 private:
  QuadrotorMixerConfig config_{};
  bool configured_{false};
};

}  // namespace mca::uav
