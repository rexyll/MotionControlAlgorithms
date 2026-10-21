#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "mca/core/types.hpp"
#include "mca/geometry/se3_so3.hpp"

namespace mca::uav {

struct MpcPositionControllerConfig {
  double sample_time{0.02};
  geometry::Vec3 position_weight{12.0, 12.0, 14.0};
  geometry::Vec3 velocity_weight{3.0, 3.0, 4.0};
  geometry::Vec3 acceleration_weight{0.8, 0.8, 1.0};
  geometry::Vec3 terminal_position_weight{20.0, 20.0, 24.0};
  geometry::Vec3 terminal_velocity_weight{5.0, 5.0, 6.0};
  geometry::Vec3 max_acceleration{8.0, 8.0, 8.0};
  std::size_t horizon{20U};
  std::size_t optimization_iterations{80U};
  double gradient_step{0.01};
};

struct MpcPositionControllerInput {
  geometry::Vec3 position_world{};
  geometry::Vec3 velocity_world{};
  geometry::Vec3 desired_position_world{};
  geometry::Vec3 desired_velocity_world{};
  geometry::Vec3 feedforward_acceleration_world{};
};

struct MpcPositionControllerOutput {
  geometry::Vec3 acceleration_world{};
  geometry::Vec3 position_error_world{};
  geometry::Vec3 velocity_error_world{};
  double predicted_cost{0.0};
};

// Fixed-size, receding-horizon translational MPC. Each axis uses a sampled
// double-integrator prediction and projected-gradient optimization. The first
// optimized acceleration correction is applied; feedforward acceleration is
// added and the resulting command is clamped per axis.
class MpcPositionController {
 public:
  static constexpr std::size_t kMaxHorizon = 100U;
  MpcPositionController() = default;

  core::Status configure(const MpcPositionControllerConfig& config) noexcept {
    if (!core::is_finite(config.sample_time) || config.sample_time <= 0.0 ||
        !finite_positive(config.position_weight) ||
        !finite_positive(config.velocity_weight) ||
        !finite_positive(config.acceleration_weight) ||
        !finite_positive(config.terminal_position_weight) ||
        !finite_positive(config.terminal_velocity_weight) ||
        !finite_positive(config.max_acceleration) || config.horizon == 0U ||
        config.horizon > kMaxHorizon || config.optimization_iterations == 0U ||
        config.optimization_iterations > 500U ||
        !core::is_finite(config.gradient_step) || config.gradient_step <= 0.0 ||
        config.gradient_step > 0.1) {
      configured_ = false;
      return {core::StatusCode::InvalidArgument};
    }
    config_ = config;
    configured_ = true;
    return {};
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] const MpcPositionControllerConfig& config() const noexcept {
    return config_;
  }

  core::Status update(const MpcPositionControllerInput& input,
                      MpcPositionControllerOutput& output) const noexcept {
    if (!configured_) return {core::StatusCode::NotConfigured};
    if (!finite(input.position_world) || !finite(input.velocity_world) ||
        !finite(input.desired_position_world) ||
        !finite(input.desired_velocity_world) ||
        !finite(input.feedforward_acceleration_world)) {
      return {core::StatusCode::InvalidArgument};
    }

    const geometry::Vec3 pe = input.position_world - input.desired_position_world;
    const geometry::Vec3 ve = input.velocity_world - input.desired_velocity_world;
    const double p0[3]{pe.x, pe.y, pe.z};
    const double v0[3]{ve.x, ve.y, ve.z};
    const double ff[3]{input.feedforward_acceleration_world.x,
                       input.feedforward_acceleration_world.y,
                       input.feedforward_acceleration_world.z};
    const double q[3]{config_.position_weight.x, config_.position_weight.y,
                      config_.position_weight.z};
    const double qv[3]{config_.velocity_weight.x, config_.velocity_weight.y,
                       config_.velocity_weight.z};
    const double r[3]{config_.acceleration_weight.x, config_.acceleration_weight.y,
                      config_.acceleration_weight.z};
    const double qt[3]{config_.terminal_position_weight.x,
                       config_.terminal_position_weight.y,
                       config_.terminal_position_weight.z};
    const double qvt[3]{config_.terminal_velocity_weight.x,
                        config_.terminal_velocity_weight.y,
                        config_.terminal_velocity_weight.z};
    const double limits[3]{config_.max_acceleration.x, config_.max_acceleration.y,
                           config_.max_acceleration.z};
    double correction[3]{};
    double total_cost = 0.0;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      double cost = 0.0;
      if (!solve_axis(p0[axis], v0[axis], q[axis], qv[axis], r[axis],
                      qt[axis], qvt[axis], limits[axis], correction[axis], cost)) {
        return {core::StatusCode::NumericalError};
      }
      total_cost += cost;
    }

    MpcPositionControllerOutput candidate;
    candidate.position_error_world = pe;
    candidate.velocity_error_world = ve;
    candidate.acceleration_world = {
        clamp(ff[0] + correction[0], -limits[0], limits[0]),
        clamp(ff[1] + correction[1], -limits[1], limits[1]),
        clamp(ff[2] + correction[2], -limits[2], limits[2])};
    candidate.predicted_cost = total_cost;
    if (!finite(candidate.acceleration_world) ||
        !core::is_finite(candidate.predicted_cost)) {
      return {core::StatusCode::NumericalError};
    }
    output = candidate;
    return {};
  }

 private:
  bool solve_axis(double p0, double v0, double q, double qv, double r,
                  double qt, double qvt, double control_limit, double& first_control,
                  double& final_cost) const noexcept {
    const std::size_t n = config_.horizon;
    const double dt = config_.sample_time;
    const double half_dt2 = 0.5 * dt * dt;
    std::array<double, kMaxHorizon> u{};
    std::array<double, kMaxHorizon + 1U> p{};
    std::array<double, kMaxHorizon + 1U> v{};
    std::array<double, kMaxHorizon> grad{};

    const auto evaluate = [&](bool with_gradient) noexcept {
      p[0] = p0;
      v[0] = v0;
      for (std::size_t k = 0; k < n; ++k) {
        p[k + 1U] = p[k] + dt * v[k] + half_dt2 * u[k];
        v[k + 1U] = v[k] + dt * u[k];
      }
      double cost = qt * p[n] * p[n] + qvt * v[n] * v[n];
      for (std::size_t k = 0; k < n; ++k) {
        cost += q * p[k] * p[k] + qv * v[k] * v[k] + r * u[k] * u[k];
      }
      if (with_gradient) {
        double adj_p = 2.0 * qt * p[n];
        double adj_v = 2.0 * qvt * v[n];
        for (std::size_t reverse = n; reverse > 0U; --reverse) {
          const std::size_t k = reverse - 1U;
          grad[k] = 2.0 * r * u[k] + half_dt2 * adj_p + dt * adj_v;
          const double prev_adj_p = 2.0 * q * p[k] + adj_p;
          const double prev_adj_v = 2.0 * qv * v[k] + dt * adj_p + adj_v;
          adj_p = prev_adj_p;
          adj_v = prev_adj_v;
        }
      }
      return cost;
    };

    for (std::size_t iteration = 0; iteration < config_.optimization_iterations;
         ++iteration) {
      (void)evaluate(true);
      for (std::size_t k = 0; k < n; ++k) {
        u[k] = clamp(u[k] - config_.gradient_step * grad[k],
                     -control_limit, control_limit);
        if (!core::is_finite(u[k]) || !core::is_finite(grad[k])) return false;
      }
    }
    final_cost = evaluate(false);
    first_control = u[0];
    return core::is_finite(final_cost) && core::is_finite(first_control);
  }

  static double clamp(double value, double low, double high) noexcept {
    return std::max(low, std::min(value, high));
  }
  static bool finite(geometry::Vec3 v) noexcept {
    return core::is_finite(v.x) && core::is_finite(v.y) && core::is_finite(v.z);
  }
  static bool finite_positive(geometry::Vec3 v) noexcept {
    return finite(v) && v.x > 0.0 && v.y > 0.0 && v.z > 0.0;
  }

  MpcPositionControllerConfig config_{};
  bool configured_{false};
};

}  // namespace mca::uav
