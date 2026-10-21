#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mca::path_tracking {

struct LqrPose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct LqrPathPoint2D {
  double x{0.0};
  double y{0.0};
};

struct LqrConfig {
  double lateral_error_weight{2.0};
  double heading_error_weight{1.0};
  double steering_weight{1.0};
  double sample_time{0.05};
  double linear_speed{1.0};
  double wheelbase{0.5};
  double max_steering_angle{0.6};
};

struct LqrCommand {
  double linear_x{0.0};
  double steering_angle{0.0};
  double angular_z{0.0};
  double lateral_error{0.0};
  double heading_error{0.0};
  std::size_t target_segment{0};
};

// Discrete-time, kinematic-bicycle LQR path tracker. Uses the local tangent
// of the nearest non-degenerate path segment and keeps middleware out of core.
class LqrPathTracker {
 public:
  LqrPathTracker() = default;
  explicit LqrPathTracker(const LqrConfig& config) { configure(config); }

  bool configure(const LqrConfig& config) noexcept {
    if (!finite(config) || config.lateral_error_weight <= 0.0 ||
        config.heading_error_weight <= 0.0 || config.steering_weight <= 0.0 ||
        config.sample_time <= 0.0 || config.linear_speed <= 0.0 ||
        config.wheelbase <= 0.0 || config.max_steering_angle <= 0.0 ||
        config.max_steering_angle >= 0.5 * kPi) {
      configured_ = false;
      return false;
    }

    // Linearized error model: [e_y(k+1), e_psi(k+1)]' = A*x + B*delta.
    const double a01 = config.linear_speed * config.sample_time;
    const double b1 = config.linear_speed * config.sample_time /
                      config.wheelbase;
    // Solve the 2x2 discrete algebraic Riccati equation by fixed-point
    // iteration. P is symmetric: [[p00,p01],[p01,p11]].
    double p00 = config.lateral_error_weight;
    double p01 = 0.0;
    double p11 = config.heading_error_weight;
    bool converged = false;
    for (int iteration = 0; iteration < 1000; ++iteration) {
      const double pb0 = p01 * b1;
      const double pb1 = p11 * b1;
      const double denominator = config.steering_weight + b1 * pb1;
      if (!std::isfinite(denominator) || denominator <= 0.0) {
        configured_ = false;
        return false;
      }
      // A' P A for A=[[1,a01],[0,1]], and A' P B.
      const double atpa00 = p00;
      const double atpa01 = p00 * a01 + p01;
      const double atpa11 = p00 * a01 * a01 + 2.0 * p01 * a01 + p11;
      const double atpb0 = pb0;
      const double atpb1 = pb0 * a01 + pb1;
      const double gain0 = atpb0 / denominator;
      const double gain1 = atpb1 / denominator;
      const double next00 = config.lateral_error_weight + atpa00 -
                            gain0 * atpb0;
      const double next01 = atpa01 - gain0 * atpb1;
      const double next11 = config.heading_error_weight + atpa11 -
                            gain1 * atpb1;
      if (!std::isfinite(next00) || !std::isfinite(next01) ||
          !std::isfinite(next11)) {
        configured_ = false;
        return false;
      }
      const double change = std::max({std::abs(next00 - p00),
                                      std::abs(next01 - p01),
                                      std::abs(next11 - p11)});
      p00 = next00;
      p01 = next01;
      p11 = next11;
      if (change < 1e-10) {
        converged = true;
        break;
      }
    }
    if (!converged) {
      configured_ = false;
      return false;
    }

    const double b1p1 = b1 * p11;
    const double denominator = config.steering_weight + b1 * b1p1;
    const double k_lateral = (b1 * p01) / denominator;
    const double k_heading = (b1 * (p01 * a01 + p11)) / denominator;
    if (!std::isfinite(k_lateral) || !std::isfinite(k_heading)) {
      configured_ = false;
      return false;
    }
    config_ = config;
    lateral_gain_ = k_lateral;
    heading_gain_ = k_heading;
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  const LqrConfig& config() const noexcept { return config_; }
  double lateral_gain() const noexcept { return lateral_gain_; }
  double heading_gain() const noexcept { return heading_gain_; }

  bool compute(const LqrPose2D& pose,
               const std::vector<LqrPathPoint2D>& path,
               LqrCommand& command) const noexcept {
    if (!configured_ || !finite(pose) || path.size() < 2U) return false;
    for (const auto& point : path) {
      if (!finite(point)) return false;
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    double best_lateral_error = 0.0;
    double best_path_yaw = 0.0;
    std::size_t best_segment = 0U;
    for (std::size_t i = 0U; i + 1U < path.size(); ++i) {
      const double dx = path[i + 1U].x - path[i].x;
      const double dy = path[i + 1U].y - path[i].y;
      const double length_sq = dx * dx + dy * dy;
      if (length_sq <= 0.0 || !std::isfinite(length_sq)) continue;
      const double projection = clamp(
          ((pose.x - path[i].x) * dx + (pose.y - path[i].y) * dy) /
              length_sq, 0.0, 1.0);
      const double px = path[i].x + projection * dx;
      const double py = path[i].y + projection * dy;
      const double ex = pose.x - px;
      const double ey = pose.y - py;
      const double distance_sq = ex * ex + ey * ey;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        const double length = std::sqrt(length_sq);
        // Positive lateral error means vehicle is left of path direction.
        best_lateral_error = (-dy * ex + dx * ey) / length;
        best_path_yaw = std::atan2(dy, dx);
        best_segment = i;
      }
    }
    if (!std::isfinite(best_distance_sq)) return false;

    const double heading_error = normalizeAngle(pose.yaw - best_path_yaw);
    const double raw_steering = -lateral_gain_ * best_lateral_error -
                                heading_gain_ * heading_error;
    const double steering = clamp(raw_steering,
                                  -config_.max_steering_angle,
                                  config_.max_steering_angle);
    LqrCommand next;
    next.linear_x = config_.linear_speed;
    next.steering_angle = steering;
    next.angular_z = next.linear_x * std::tan(steering) / config_.wheelbase;
    next.lateral_error = best_lateral_error;
    next.heading_error = heading_error;
    next.target_segment = best_segment;
    if (!finite(next)) return false;
    command = next;
    return true;
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  static double clamp(double value, double low, double high) noexcept {
    return value < low ? low : (value > high ? high : value);
  }
  static double normalizeAngle(double angle) noexcept {
    return std::atan2(std::sin(angle), std::cos(angle));
  }
  static bool finite(const LqrConfig& c) noexcept {
    return std::isfinite(c.lateral_error_weight) &&
           std::isfinite(c.heading_error_weight) &&
           std::isfinite(c.steering_weight) && std::isfinite(c.sample_time) &&
           std::isfinite(c.linear_speed) && std::isfinite(c.wheelbase) &&
           std::isfinite(c.max_steering_angle);
  }
  static bool finite(const LqrPose2D& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(p.yaw);
  }
  static bool finite(const LqrPathPoint2D& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y);
  }
  static bool finite(const LqrCommand& c) noexcept {
    return std::isfinite(c.linear_x) && std::isfinite(c.steering_angle) &&
           std::isfinite(c.angular_z) && std::isfinite(c.lateral_error) &&
           std::isfinite(c.heading_error);
  }

  LqrConfig config_{};
  double lateral_gain_{0.0};
  double heading_gain_{0.0};
  bool configured_{false};
};

}  // namespace mca::path_tracking
