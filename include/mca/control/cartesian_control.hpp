#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include "mca/geometry/se3_so3.hpp"
#include "mca/kinematics/forward_kinematics.hpp"
#include "mca/kinematics/jacobian.hpp"

namespace mca::control {

// Task-space PD controller with optional Cartesian feedforward wrench.
// The wrench and Jacobian use the base-frame [vx,vy,vz,wx,wy,wz] convention.
// Joint torques are tau = J^T * wrench; this is not an impedance/contact safety layer.
class CartesianControl {
 public:
  static bool compute(const kinematics::RobotModel& model,
                      const Eigen::VectorXd& q,
                      const Eigen::VectorXd& dq,
                      const geometry::Pose3D& desired_pose,
                      const Eigen::VectorXd& desired_twist,
                      const Eigen::VectorXd& desired_wrench,
                      const Eigen::VectorXd& kp,
                      const Eigen::VectorXd& kd,
                      const Eigen::VectorXd& torque_limits,
                      Eigen::VectorXd& out,
                      std::string* error = nullptr) {
    const Eigen::Index n = q.size();
    if (n <= 0 || dq.size() != n || desired_twist.size() != 6 ||
        desired_wrench.size() != 6 || kp.size() != 6 || kd.size() != 6)
      return fail("Cartesian-control vector dimensions are invalid", error);
    if (!q.allFinite() || !dq.allFinite() || !desired_twist.allFinite() ||
        !desired_wrench.allFinite() || !kp.allFinite() || !kd.allFinite())
      return fail("Cartesian-control inputs must be finite", error);
    if ((kp.array() < 0.0).any() || (kd.array() < 0.0).any())
      return fail("Cartesian-control gains must be non-negative", error);
    if (torque_limits.size() != 0 && torque_limits.size() != n)
      return fail("Cartesian-control torque limits have invalid dimension", error);
    if (torque_limits.size() != 0 &&
        (!torque_limits.allFinite() || (torque_limits.array() < 0.0).any()))
      return fail("Cartesian-control torque limits must be finite and non-negative", error);

    kinematics::ForwardKinematicsResult fk;
    if (!kinematics::ForwardKinematics::solve(model, to_std(q), fk, error)) return false;
    kinematics::JacobianResult jacobian;
    if (!kinematics::Jacobian::solve(model, to_std(q), jacobian, error)) return false;

    const geometry::Vec3 position_error = desired_pose.position - fk.pose.position;
    const geometry::Quaternion q_error = geometry::normalized(
        geometry::multiply(desired_pose.rotation, geometry::conjugate(fk.pose.rotation)));
    const geometry::Vec3 orientation_error = geometry::log_so3(q_error);
    const geometry::Vec3 orientation_error_base =
        geometry::rotate(fk.pose.rotation, orientation_error);

    Eigen::VectorXd pose_error(6);
    pose_error << position_error.x, position_error.y, position_error.z,
                  orientation_error_base.x, orientation_error_base.y,
                  orientation_error_base.z;

    Eigen::MatrixXd J(6, n);
    for (Eigen::Index c = 0; c < n; ++c)
      for (Eigen::Index r = 0; r < 6; ++r)
        J(r, c) = jacobian.rows[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
    const Eigen::VectorXd twist = J * dq;
    if (!twist.allFinite()) return fail("Cartesian-control twist is non-finite", error);

    const Eigen::VectorXd wrench = desired_wrench +
        kp.cwiseProduct(pose_error) + kd.cwiseProduct(desired_twist - twist);
    Eigen::VectorXd candidate = J.transpose() * wrench;
    if (!candidate.allFinite()) return fail("Cartesian-control torque is non-finite", error);
    if (torque_limits.size() != 0) {
      for (Eigen::Index i = 0; i < n; ++i)
        candidate[i] = std::max(-torque_limits[i], std::min(torque_limits[i], candidate[i]));
    }
    out = std::move(candidate);
    return true;
  }

 private:
  static std::vector<double> to_std(const Eigen::VectorXd& value) {
    return std::vector<double>(value.data(), value.data() + value.size());
  }
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
};

}  // namespace mca::control
