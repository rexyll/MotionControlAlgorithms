#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>
#include "mca/kinematics/forward_kinematics.hpp"

namespace mca::kinematics {

// Geometric Jacobian rows are [vx, vy, vz, wx, wy, wz] in the base frame.
// Columns follow RobotModel::joints order with fixed joints omitted.
struct JacobianResult {
  std::array<std::vector<double>, 6> rows{};
  std::size_t columns{0};

  double operator()(std::size_t row, std::size_t column) const {
    return rows.at(row).at(column);
  }
};

class Jacobian {
 public:
  static bool solve(const RobotModel& model, const std::vector<double>& q,
                    JacobianResult& out, std::string* error = nullptr) {
    ForwardKinematicsResult fk;
    if (!ForwardKinematics::solve(model, q, fk, error)) return false;

    JacobianResult candidate;
    candidate.columns = q.size();
    for (auto& row : candidate.rows) row.assign(q.size(), 0.0);

    const geometry::Vec3 tip = fk.pose.position;
    std::size_t qi = 0;
    for (std::size_t ji = 0; ji < model.joints.size(); ++ji) {
      const auto& joint = model.joints[ji];
      if (joint.type == JointType::Fixed) continue;

      const geometry::Pose3D& parent = fk.link_poses[ji];
      geometry::Pose3D joint_frame;
      geometry::Vec3 axis;
      if (joint.standard_dh) {
        // Standard DH joint variable rotates/translates along the parent z axis.
        joint_frame = parent;
        axis = geometry::rotate(parent.rotation, {0.0, 0.0, 1.0});
      } else {
        joint_frame = geometry::compose(parent, joint.origin);
        axis = geometry::rotate(joint_frame.rotation, joint.axis);
      }
      const double axis_norm = geometry::norm(axis);
      if (!std::isfinite(axis_norm) || axis_norm <= 1e-12) {
        if (error) *error = "joint axis is invalid";
        return false;
      }
      axis = (1.0 / axis_norm) * axis;

      geometry::Vec3 linear{};
      geometry::Vec3 angular{};
      if (joint.type == JointType::Revolute || joint.type == JointType::Continuous) {
        linear = geometry::cross(axis, tip - joint_frame.position);
        angular = axis;
      } else if (joint.type == JointType::Prismatic) {
        linear = axis;
      }
      candidate.rows[0][qi] = linear.x;
      candidate.rows[1][qi] = linear.y;
      candidate.rows[2][qi] = linear.z;
      candidate.rows[3][qi] = angular.x;
      candidate.rows[4][qi] = angular.y;
      candidate.rows[5][qi] = angular.z;
      ++qi;
    }
    for (const auto& row : candidate.rows) {
      for (double value : row) {
        if (!std::isfinite(value)) {
          if (error) *error = "Jacobian contains a non-finite value";
          return false;
        }
      }
    }
    out = std::move(candidate);
    return true;
  }
};

}  // namespace mca::kinematics
