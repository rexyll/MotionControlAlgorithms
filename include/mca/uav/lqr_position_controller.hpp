#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "mca/core/types.hpp"
#include "mca/geometry/se3_so3.hpp"

namespace mca::uav {

struct LqrPositionControllerConfig {
  double sample_time{0.01};
  geometry::Vec3 position_weight{10.0, 10.0, 12.0};
  geometry::Vec3 velocity_weight{3.0, 3.0, 4.0};
  geometry::Vec3 acceleration_weight{1.0, 1.0, 1.0};
  geometry::Vec3 max_acceleration{8.0, 8.0, 8.0};
};

struct LqrPositionControllerInput {
  geometry::Vec3 position_world{};
  geometry::Vec3 velocity_world{};
  geometry::Vec3 desired_position_world{};
  geometry::Vec3 desired_velocity_world{};
  geometry::Vec3 feedforward_acceleration_world{};
};

struct LqrPositionControllerOutput {
  geometry::Vec3 acceleration_world{};
  geometry::Vec3 position_error_world{};
  geometry::Vec3 velocity_error_world{};
};

// Independent discrete-time LQR controllers for the three translational axes.
// Each axis uses the exact sampled double-integrator model with acceleration
// as input. The discrete algebraic Riccati equation is solved at configure
// time; update() is fixed-size, allocation-free, and deterministic.
class LqrPositionController {
 public:
  LqrPositionController() = default;

  core::Status configure(const LqrPositionControllerConfig& config) noexcept {
    if (!core::is_finite(config.sample_time) || config.sample_time <= 0.0 ||
        !finite_positive(config.position_weight) ||
        !finite_positive(config.velocity_weight) ||
        !finite_positive(config.acceleration_weight) ||
        !finite_positive(config.max_acceleration)) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }

    LqrPositionControllerConfig candidate_config = config;
    std::array<double, 3> kp{};
    std::array<double, 3> kv{};
    const double dt = config.sample_time;
    const double a[2][2]{{1.0, dt}, {0.0, 1.0}};
    const double b[2]{0.5 * dt * dt, dt};
    const double q_position[3]{config.position_weight.x,
                               config.position_weight.y,
                               config.position_weight.z};
    const double q_velocity[3]{config.velocity_weight.x,
                               config.velocity_weight.y,
                               config.velocity_weight.z};
    const double r_values[3]{config.acceleration_weight.x,
                             config.acceleration_weight.y,
                             config.acceleration_weight.z};

    for (std::size_t axis = 0; axis < 3; ++axis) {
      double p[2][2]{{q_position[axis], 0.0}, {0.0, q_velocity[axis]}};
      bool converged = false;
      for (int iteration = 0; iteration < 10000; ++iteration) {
        double pa[2][2]{};
        double atpa[2][2]{};
        double pb[2]{};
        double atpb[2]{};
        for (int i = 0; i < 2; ++i) {
          for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) pa[i][j] += p[i][k] * a[k][j];
          }
          for (int k = 0; k < 2; ++k) pb[i] += p[i][k] * b[k];
        }
        double denominator = r_values[axis];
        for (int i = 0; i < 2; ++i) denominator += b[i] * pb[i];
        if (!core::is_finite(denominator) || denominator <= 0.0) {
          configured_ = false;
          return {core::StatusCode::NumericalError};
        }
        double gain[2]{};
        for (int i = 0; i < 2; ++i) {
          for (int k = 0; k < 2; ++k) atpb[i] += a[k][i] * pb[k];
          gain[i] = atpb[i] / denominator;
          for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) atpa[i][j] += a[k][i] * pa[k][j];
          }
        }
        double next[2][2]{};
        next[0][0] = q_position[axis] + atpa[0][0] - atpb[0] * gain[0];
        next[0][1] = atpa[0][1] - atpb[0] * gain[1];
        next[1][0] = atpa[1][0] - atpb[1] * gain[0];
        next[1][1] = q_velocity[axis] + atpa[1][1] - atpb[1] * gain[1];
        double change = 0.0;
        for (int i = 0; i < 2; ++i) {
          for (int j = 0; j < 2; ++j) {
            if (!core::is_finite(next[i][j])) {
              configured_ = false;
              return {core::StatusCode::NumericalError};
            }
            change = std::max(change, std::abs(next[i][j] - p[i][j]));
            p[i][j] = next[i][j];
          }
        }
        if (change < 1e-11) {
          converged = true;
          break;
        }
      }
      if (!converged) {
        configured_ = false;
        return {core::StatusCode::NumericalError};
      }

      double pb[2]{};
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) pb[i] += p[i][j] * b[j];
      const double denominator = r_values[axis] + b[0] * pb[0] + b[1] * pb[1];
      double atpb[2]{};
      for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) atpb[i] += a[j][i] * pb[j];
      kp[axis] = atpb[0] / denominator;
      kv[axis] = atpb[1] / denominator;
      if (!core::is_finite(kp[axis]) || !core::is_finite(kv[axis])) {
        configured_ = false;
        return {core::StatusCode::NumericalError};
      }
    }

    config_ = candidate_config;
    position_gain_ = {kp[0], kp[1], kp[2]};
    velocity_gain_ = {kv[0], kv[1], kv[2]};
    configured_ = true;
    return {};
  }

  core::Status update(const LqrPositionControllerInput& input,
                      LqrPositionControllerOutput& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!finite(input.position_world) || !finite(input.velocity_world) ||
        !finite(input.desired_position_world) ||
        !finite(input.desired_velocity_world) ||
        !finite(input.feedforward_acceleration_world)) {
      return {core::StatusCode::InvalidArgument};
    }

    LqrPositionControllerOutput candidate;
    candidate.position_error_world = input.position_world - input.desired_position_world;
    candidate.velocity_error_world = input.velocity_world - input.desired_velocity_world;
    candidate.acceleration_world = {
        input.feedforward_acceleration_world.x -
            position_gain_.x * candidate.position_error_world.x -
            velocity_gain_.x * candidate.velocity_error_world.x,
        input.feedforward_acceleration_world.y -
            position_gain_.y * candidate.position_error_world.y -
            velocity_gain_.y * candidate.velocity_error_world.y,
        input.feedforward_acceleration_world.z -
            position_gain_.z * candidate.position_error_world.z -
            velocity_gain_.z * candidate.velocity_error_world.z};
    candidate.acceleration_world.x = std::clamp(
        candidate.acceleration_world.x, -config_.max_acceleration.x,
        config_.max_acceleration.x);
    candidate.acceleration_world.y = std::clamp(
        candidate.acceleration_world.y, -config_.max_acceleration.y,
        config_.max_acceleration.y);
    candidate.acceleration_world.z = std::clamp(
        candidate.acceleration_world.z, -config_.max_acceleration.z,
        config_.max_acceleration.z);
    if (!finite(candidate.acceleration_world) ||
        !finite(candidate.position_error_world) ||
        !finite(candidate.velocity_error_world)) {
      return {core::StatusCode::NumericalError};
    }
    output = candidate;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] geometry::Vec3 position_gain() const noexcept { return position_gain_; }
  [[nodiscard]] geometry::Vec3 velocity_gain() const noexcept { return velocity_gain_; }

 private:
  static bool finite(geometry::Vec3 v) noexcept {
    return core::is_finite(v.x) && core::is_finite(v.y) && core::is_finite(v.z);
  }
  static bool finite_positive(geometry::Vec3 v) noexcept {
    return finite(v) && v.x > 0.0 && v.y > 0.0 && v.z > 0.0;
  }

  LqrPositionControllerConfig config_{};
  geometry::Vec3 position_gain_{};
  geometry::Vec3 velocity_gain_{};
  bool configured_{false};
};

}  // namespace mca::uav
