#pragma once
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>
#include "mca/kinematics/dh_urdf_model.hpp"

namespace mca::kinematics {

struct ForwardKinematicsResult {
  geometry::Pose3D pose{};
  std::vector<geometry::Pose3D> link_poses{};
};

class ForwardKinematics {
 public:
  // q contains only movable-joint coordinates, in RobotModel::joints order.
  // Revolute/continuous coordinates are radians; prismatic coordinates are metres.
  static bool solve(const RobotModel& model, const std::vector<double>& q,
                    ForwardKinematicsResult& out, std::string* error = nullptr) {
    std::string validation_error;
    if (!model.valid(&validation_error)) return fail(validation_error.c_str(), error);
    std::size_t movable = 0;
    for (const auto& joint : model.joints) {
      if (joint.type != JointType::Fixed) ++movable;
    }
    if (q.size() != movable) return fail("joint coordinate count does not match movable joints", error);
    for (double value : q) {
      if (!std::isfinite(value)) return fail("joint coordinate is non-finite", error);
    }

    ForwardKinematicsResult candidate;
    candidate.link_poses.reserve(model.links.size());
    geometry::Pose3D current{};
    candidate.link_poses.push_back(current);
    std::size_t qi = 0;
    for (const auto& joint : model.joints) {
      geometry::Pose3D local = joint.origin;
      if (joint.type == JointType::Revolute || joint.type == JointType::Continuous) {
        local.rotation = geometry::normalized(
            geometry::multiply(local.rotation, geometry::axis_angle(joint.axis, q[qi])));
        ++qi;
      } else if (joint.type == JointType::Prismatic) {
        local.position = local.position + q[qi] * joint.axis;
        ++qi;
      }
      current = geometry::compose(current, local);
      candidate.link_poses.push_back(current);
    }
    if (candidate.link_poses.size() != model.links.size()) return fail("FK link pose count mismatch", error);
    candidate.pose = current;
    if (!finite_pose(candidate.pose)) return fail("FK produced a non-finite pose", error);
    for (const auto& pose : candidate.link_poses) {
      if (!finite_pose(pose)) return fail("FK produced a non-finite link pose", error);
    }
    out = std::move(candidate);
    return true;
  }

 private:
  static bool finite_pose(const geometry::Pose3D& p) {
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.position.z) && std::isfinite(p.rotation.w) &&
           std::isfinite(p.rotation.x) && std::isfinite(p.rotation.y) &&
           std::isfinite(p.rotation.z);
  }
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
};

}  // namespace mca::kinematics
