#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mca::path_tracking {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PathPoint2D {
  double x{0.0};
  double y{0.0};
};

struct PurePursuitCommand {
  double linear_x{0.0};
  double angular_z{0.0};
  double curvature{0.0};
  std::size_t target_index{0};
};

struct PurePursuitConfig {
  double lookahead_distance{0.5};
  double linear_speed{1.0};
};

// Geometric pure-pursuit controller for a planar polyline.
// The controller is independent of any vehicle middleware or simulator.
class PurePursuit {
 public:
  PurePursuit() = default;
  explicit PurePursuit(const PurePursuitConfig& config) { configure(config); }

  bool configure(const PurePursuitConfig& config) noexcept {
    if (!finite(config) || config.lookahead_distance <= 0.0) {
      configured_ = false;
      return false;
    }
    config_ = config;
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  const PurePursuitConfig& config() const noexcept { return config_; }

  // Select a lookahead point on or after the nearest path point and compute
  // kappa = 2*y_vehicle / (x_vehicle^2 + y_vehicle^2).
  bool compute(const Pose2D& pose, const std::vector<PathPoint2D>& path,
               PurePursuitCommand& command) const noexcept {
    if (!configured_ || !finite(pose) || path.empty()) return false;
    for (const auto& point : path) {
      if (!finite(point)) return false;
    }

    std::size_t nearest = 0;
    double nearest_distance_sq = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double dx = path[i].x - pose.x;
      const double dy = path[i].y - pose.y;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest = i;
      }
    }

    PathPoint2D target = path.back();
    std::size_t target_index = path.size() - 1;
    double remaining = config_.lookahead_distance;
    for (std::size_t i = nearest; i + 1 < path.size(); ++i) {
      const double dx = path[i + 1].x - path[i].x;
      const double dy = path[i + 1].y - path[i].y;
      const double segment_length = std::hypot(dx, dy);
      if (segment_length <= 0.0) continue;
      if (remaining <= segment_length) {
        const double ratio = remaining / segment_length;
        target.x = path[i].x + ratio * dx;
        target.y = path[i].y + ratio * dy;
        target_index = i + 1;
        break;
      }
      remaining -= segment_length;
    }

    const double dx_world = target.x - pose.x;
    const double dy_world = target.y - pose.y;
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    const double target_x_vehicle = c * dx_world + s * dy_world;
    const double target_y_vehicle = -s * dx_world + c * dy_world;
    const double lookahead_sq = target_x_vehicle * target_x_vehicle +
                                target_y_vehicle * target_y_vehicle;
    if (!std::isfinite(lookahead_sq) || lookahead_sq <= 0.0) return false;

    command.linear_x = config_.linear_speed;
    command.curvature = 2.0 * target_y_vehicle / lookahead_sq;
    command.angular_z = command.linear_x * command.curvature;
    command.target_index = target_index;
    return finite(command);
  }

 private:
  static bool finite(const PurePursuitConfig& config) noexcept {
    return std::isfinite(config.lookahead_distance) &&
           std::isfinite(config.linear_speed);
  }

  static bool finite(const Pose2D& pose) noexcept {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.yaw);
  }

  static bool finite(const PathPoint2D& point) noexcept {
    return std::isfinite(point.x) && std::isfinite(point.y);
  }

  static bool finite(const PurePursuitCommand& command) noexcept {
    return std::isfinite(command.linear_x) &&
           std::isfinite(command.angular_z) &&
           std::isfinite(command.curvature);
  }

  PurePursuitConfig config_{};
  bool configured_{false};
};

}  // namespace mca::path_tracking
