#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mca::path_tracking {

// Stanley controller for a planar path. Coordinates use x-forward/y-left and
// positive yaw counter-clockwise. Cross-track error is signed positive when
// the vehicle is to the right of the directed path, so positive correction
// steers back toward the path.
struct StanleyPose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct StanleyPathPoint2D {
  double x{0.0};
  double y{0.0};
};

struct StanleyConfig {
  double cross_track_gain{1.0};
  double heading_gain{1.0};
  double softening_speed{0.1};
  double linear_speed{1.0};
  double wheelbase{0.5};
  double max_steering_angle{0.6};
};

struct StanleyCommand {
  double linear_x{0.0};
  double steering_angle{0.0};
  double angular_z{0.0};
  double cross_track_error{0.0};
  double heading_error{0.0};
  std::size_t target_segment{0};
};

class Stanley {
 public:
  Stanley() = default;
  explicit Stanley(const StanleyConfig& config) { configure(config); }

  bool configure(const StanleyConfig& config) noexcept {
    if (!finite(config) || config.cross_track_gain < 0.0 ||
        config.heading_gain < 0.0 || config.softening_speed <= 0.0 ||
        config.wheelbase <= 0.0 || config.max_steering_angle <= 0.0 ||
        config.max_steering_angle >= 0.5 * kPi) {
      configured_ = false;
      return false;
    }
    config_ = config;
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  const StanleyConfig& config() const noexcept { return config_; }

  bool compute(const StanleyPose2D& pose,
               const std::vector<StanleyPathPoint2D>& path,
               StanleyCommand& command) const noexcept {
    if (!configured_ || !finite(pose) || path.size() < 2U) return false;
    for (const auto& point : path) {
      if (!finite(point)) return false;
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    double best_cross_track = 0.0;
    double best_heading = 0.0;
    std::size_t best_segment = 0U;

    for (std::size_t i = 0U; i + 1U < path.size(); ++i) {
      const double dx = path[i + 1U].x - path[i].x;
      const double dy = path[i + 1U].y - path[i].y;
      const double length_sq = dx * dx + dy * dy;
      if (length_sq <= 0.0 || !std::isfinite(length_sq)) continue;

      const double projection = clamp(
          ((pose.x - path[i].x) * dx + (pose.y - path[i].y) * dy) /
              length_sq,
          0.0, 1.0);
      const double projected_x = path[i].x + projection * dx;
      const double projected_y = path[i].y + projection * dy;
      const double error_x = pose.x - projected_x;
      const double error_y = pose.y - projected_y;
      const double distance_sq = error_x * error_x + error_y * error_y;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        // Positive when the vehicle is to the right of the path direction.
        best_cross_track = (dy * error_x - dx * error_y) /
                           std::sqrt(length_sq);
        best_heading = normalizeAngle(std::atan2(dy, dx) - pose.yaw);
        best_segment = i;
      }
    }
    if (!std::isfinite(best_distance_sq)) return false;

    const double speed = std::abs(config_.linear_speed);
    const double correction = config_.heading_gain * best_heading +
        std::atan2(config_.cross_track_gain * best_cross_track,
                   config_.softening_speed + speed);
    const double steering = clamp(correction, -config_.max_steering_angle,
                                  config_.max_steering_angle);
    StanleyCommand next;
    next.linear_x = config_.linear_speed;
    next.steering_angle = steering;
    next.angular_z = next.linear_x * std::tan(steering) / config_.wheelbase;
    next.cross_track_error = best_cross_track;
    next.heading_error = best_heading;
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

  static bool finite(const StanleyConfig& c) noexcept {
    return std::isfinite(c.cross_track_gain) &&
           std::isfinite(c.heading_gain) &&
           std::isfinite(c.softening_speed) &&
           std::isfinite(c.linear_speed) && std::isfinite(c.wheelbase) &&
           std::isfinite(c.max_steering_angle);
  }
  static bool finite(const StanleyPose2D& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
  }
  static bool finite(const StanleyPathPoint2D& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y);
  }
  static bool finite(const StanleyCommand& c) noexcept {
    return std::isfinite(c.linear_x) && std::isfinite(c.steering_angle) &&
           std::isfinite(c.angular_z) && std::isfinite(c.cross_track_error) &&
           std::isfinite(c.heading_error);
  }

  StanleyConfig config_{};
  bool configured_{false};
};

}  // namespace mca::path_tracking
