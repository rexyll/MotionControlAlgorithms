#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mca::path_tracking {

struct MpcPose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct MpcPathPoint2D {
  double x{0.0};
  double y{0.0};
};

struct MpcConfig {
  double lateral_error_weight{4.0};
  double heading_error_weight{2.0};
  double steering_weight{0.5};
  double terminal_lateral_weight{8.0};
  double terminal_heading_weight{4.0};
  double sample_time{0.05};
  double linear_speed{1.0};
  double wheelbase{0.5};
  double max_steering_angle{0.6};
  std::size_t horizon{12U};
  std::size_t optimization_iterations{80U};
  double gradient_step{0.01};
};

struct MpcCommand {
  double linear_x{0.0};
  double steering_angle{0.0};
  double angular_z{0.0};
  double lateral_error{0.0};
  double heading_error{0.0};
  std::size_t target_segment{0U};
  double predicted_cost{0.0};
};

// Receding-horizon linear MPC for local path tracking. The optimizer uses a
// small-error kinematic-bicycle model and projected gradient descent.
class MpcPathTracker {
 public:
  MpcPathTracker() = default;
  explicit MpcPathTracker(const MpcConfig& config) { configure(config); }

  bool configure(const MpcConfig& config) noexcept {
    if (!finite(config) || config.lateral_error_weight <= 0.0 ||
        config.heading_error_weight <= 0.0 || config.steering_weight <= 0.0 ||
        config.terminal_lateral_weight <= 0.0 ||
        config.terminal_heading_weight <= 0.0 || config.sample_time <= 0.0 ||
        config.linear_speed <= 0.0 || config.wheelbase <= 0.0 ||
        config.max_steering_angle <= 0.0 ||
        config.max_steering_angle >= 0.5 * kPi || config.horizon == 0U ||
        config.horizon > 200U || config.optimization_iterations == 0U ||
        config.optimization_iterations > 1000U || config.gradient_step <= 0.0 ||
        config.gradient_step > 1.0) {
      configured_ = false;
      return false;
    }
    config_ = config;
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  const MpcConfig& config() const noexcept { return config_; }

  bool compute(const MpcPose2D& pose,
               const std::vector<MpcPathPoint2D>& path,
               MpcCommand& command) const noexcept {
    if (!configured_ || !finite(pose) || path.size() < 2U) return false;
    for (const auto& point : path) if (!finite(point)) return false;

    double best_distance_sq = std::numeric_limits<double>::infinity();
    double lateral_error = 0.0;
    double path_yaw = 0.0;
    std::size_t target_segment = 0U;
    for (std::size_t i = 0U; i + 1U < path.size(); ++i) {
      const double dx = path[i + 1U].x - path[i].x;
      const double dy = path[i + 1U].y - path[i].y;
      const double length_sq = dx * dx + dy * dy;
      if (!std::isfinite(length_sq) || length_sq <= 0.0) continue;
      const double t = clamp(((pose.x - path[i].x) * dx +
                              (pose.y - path[i].y) * dy) / length_sq, 0.0, 1.0);
      const double ex = pose.x - (path[i].x + t * dx);
      const double ey = pose.y - (path[i].y + t * dy);
      const double distance_sq = ex * ex + ey * ey;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        lateral_error = (-dy * ex + dx * ey) / std::sqrt(length_sq);
        path_yaw = std::atan2(dy, dx);
        target_segment = i;
      }
    }
    if (!std::isfinite(best_distance_sq)) return false;

    const double heading_error = normalizeAngle(pose.yaw - path_yaw);
    // Error dynamics: e_y(k+1)=e_y(k)+v*T*e_psi(k);
    // e_psi(k+1)=e_psi(k)+(v*T/L)*delta(k).
    std::vector<double> controls(config_.horizon, 0.0);
    const double max_delta = config_.max_steering_angle;
    const double a = config_.linear_speed * config_.sample_time;
    const double b = a / config_.wheelbase;
    std::vector<double> ey(config_.horizon + 1U, 0.0);
    std::vector<double> epsi(config_.horizon + 1U, 0.0);
    std::vector<double> grad(config_.horizon, 0.0);

    const auto evaluate = [&](const std::vector<double>& u,
                              std::vector<double>* out_ey,
                              std::vector<double>* out_epsi,
                              std::vector<double>* out_grad) noexcept {
      (*out_ey)[0] = lateral_error;
      (*out_epsi)[0] = heading_error;
      for (std::size_t k = 0U; k < config_.horizon; ++k) {
        (*out_ey)[k + 1U] = (*out_ey)[k] + a * (*out_epsi)[k];
        (*out_epsi)[k + 1U] = (*out_epsi)[k] + b * u[k];
      }
      double cost = 0.0;
      for (std::size_t k = 0U; k < config_.horizon; ++k) {
        cost += config_.lateral_error_weight * (*out_ey)[k] * (*out_ey)[k] +
                config_.heading_error_weight * (*out_epsi)[k] * (*out_epsi)[k] +
                config_.steering_weight * u[k] * u[k];
      }
      cost += config_.terminal_lateral_weight *
                  (*out_ey)[config_.horizon] * (*out_ey)[config_.horizon] +
              config_.terminal_heading_weight *
                  (*out_epsi)[config_.horizon] * (*out_epsi)[config_.horizon];

      if (out_grad != nullptr) {
        double adj_ey = 2.0 * config_.terminal_lateral_weight *
                        (*out_ey)[config_.horizon];
        double adj_psi = 2.0 * config_.terminal_heading_weight *
                         (*out_epsi)[config_.horizon];
        for (std::size_t reverse = config_.horizon; reverse > 0U; --reverse) {
          const std::size_t k = reverse - 1U;
          (*out_grad)[k] = 2.0 * config_.steering_weight * u[k] + b * adj_psi;
          const double current_adj_ey = 2.0 * config_.lateral_error_weight *
              (*out_ey)[k] + adj_ey;
          const double current_adj_psi = 2.0 * config_.heading_error_weight *
              (*out_epsi)[k] + a * adj_ey + adj_psi;
          adj_ey = current_adj_ey;
          adj_psi = current_adj_psi;
        }
      }
      return cost;
    };

    for (std::size_t iteration = 0U;
         iteration < config_.optimization_iterations; ++iteration) {
      evaluate(controls, &ey, &epsi, &grad);
      for (std::size_t k = 0U; k < controls.size(); ++k) {
        controls[k] = clamp(controls[k] - config_.gradient_step * grad[k],
                            -max_delta, max_delta);
      }
    }
    const double final_cost = evaluate(controls, &ey, &epsi, nullptr);
    const double steering = controls.front();
    MpcCommand next;
    next.linear_x = config_.linear_speed;
    next.steering_angle = steering;
    next.angular_z = next.linear_x * std::tan(steering) / config_.wheelbase;
    next.lateral_error = lateral_error;
    next.heading_error = heading_error;
    next.target_segment = target_segment;
    next.predicted_cost = final_cost;
    if (!finite(next)) return false;
    command = next;
    return true;
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  static double clamp(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
  }
  static double normalizeAngle(double a) noexcept {
    return std::atan2(std::sin(a), std::cos(a));
  }
  static bool finite(const MpcConfig& c) noexcept {
    return std::isfinite(c.lateral_error_weight) &&
      std::isfinite(c.heading_error_weight) && std::isfinite(c.steering_weight) &&
      std::isfinite(c.terminal_lateral_weight) &&
      std::isfinite(c.terminal_heading_weight) && std::isfinite(c.sample_time) &&
      std::isfinite(c.linear_speed) && std::isfinite(c.wheelbase) &&
      std::isfinite(c.max_steering_angle) && std::isfinite(c.gradient_step);
  }
  static bool finite(const MpcPose2D& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
  }
  static bool finite(const MpcPathPoint2D& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y);
  }
  static bool finite(const MpcCommand& c) noexcept {
    return std::isfinite(c.linear_x) && std::isfinite(c.steering_angle) &&
      std::isfinite(c.angular_z) && std::isfinite(c.lateral_error) &&
      std::isfinite(c.heading_error) && std::isfinite(c.predicted_cost);
  }

  MpcConfig config_{};
  bool configured_{false};
};

}  // namespace mca::path_tracking
