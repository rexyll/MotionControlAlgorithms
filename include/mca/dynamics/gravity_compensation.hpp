#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "mca/dynamics/rigid_body_dynamics.hpp"

namespace mca::dynamics {

// Computes gravity feedforward torque for a fixed-base serial manipulator.
// The scale and optional symmetric per-joint limits are useful for gradual
// commissioning. The returned command is feedforward only; it is not a
// feedback controller and must not replace safety limits in the drive layer.
class GravityCompensation {
 public:
  static bool compute(const kinematics::RobotModel& model,
                      const std::vector<LinkInertial>& inertias,
                      const Eigen::VectorXd& q,
                      geometry::Vec3 gravity_acceleration,
                      double scale,
                      const Eigen::VectorXd& absolute_limits,
                      Eigen::VectorXd& out,
                      std::string* error = nullptr) {
    if (!std::isfinite(scale) || scale < 0.0)
      return fail("gravity compensation scale must be finite and non-negative", error);
    Eigen::VectorXd candidate;
    if (!RigidBodyDynamics::gravity(model, inertias, q, gravity_acceleration,
                                    candidate, error)) return false;
    if (absolute_limits.size() != 0 && absolute_limits.size() != candidate.size())
      return fail("gravity compensation limits have invalid dimension", error);
    if (absolute_limits.size() != 0) {
      for (Eigen::Index i = 0; i < absolute_limits.size(); ++i) {
        if (!std::isfinite(absolute_limits[i]) || absolute_limits[i] < 0.0)
          return fail("gravity compensation limits must be finite and non-negative", error);
      }
    }
    candidate *= scale;
    if (absolute_limits.size() != 0) {
      for (Eigen::Index i = 0; i < candidate.size(); ++i)
        candidate[i] = std::max(-absolute_limits[i], std::min(absolute_limits[i], candidate[i]));
    }
    if (!candidate.allFinite()) return fail("gravity compensation result is non-finite", error);
    out = std::move(candidate);
    return true;
  }

 private:
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
};

}  // namespace mca::dynamics
