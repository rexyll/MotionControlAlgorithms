#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include "mca/geometry/se3_so3.hpp"
#include "mca/kinematics/forward_kinematics.hpp"
#include "mca/kinematics/jacobian.hpp"

namespace mca::control {

// Cartesian virtual spring-damper (impedance) controller.
// W = W_ff + K * e + D * (V_d - V), tau = J^T W.
// Pose/twist/wrench use the base-frame [x,y,z,rx,ry,rz] / [vx,vy,vz,wx,wy,wz]
// ordering. Stiffness and damping are diagonal task-space coefficients.
// Wrench limits are software limits, not drive/contact safety protections.
class CartesianImpedance {
 public:
  static bool compute(const kinematics::RobotModel& model,
                      const Eigen::VectorXd& q,
                      const Eigen::VectorXd& dq,
                      const geometry::Pose3D& equilibrium_pose,
                      const Eigen::VectorXd& desired_twist,
                      const Eigen::VectorXd& feedforward_wrench,
                      const Eigen::VectorXd& stiffness,
                      const Eigen::VectorXd& damping,
                      const Eigen::VectorXd& wrench_limits,
                      Eigen::VectorXd& out,
                      std::string* error = nullptr) {
    const Eigen::Index n = q.size();
    if (n <= 0 || dq.size() != n || desired_twist.size() != 6 ||
        feedforward_wrench.size() != 6 || stiffness.size() != 6 ||
        damping.size() != 6)
      return fail("Cartesian-impedance vector dimensions are invalid", error);
    if (!q.allFinite() || !dq.allFinite() || !desired_twist.allFinite() ||
        !feedforward_wrench.allFinite() || !stiffness.allFinite() || !damping.allFinite())
      return fail("Cartesian-impedance inputs must be finite", error);
    if ((stiffness.array() < 0.0).any() || (damping.array() < 0.0).any())
      return fail("Cartesian-impedance stiffness and damping must be non-negative", error);
    if (wrench_limits.size() != 0 && wrench_limits.size() != 6)
      return fail("Cartesian-impedance wrench limits have invalid dimension", error);
    if (wrench_limits.size() != 0 &&
        (!wrench_limits.allFinite() || (wrench_limits.array() < 0.0).any()))
      return fail("Cartesian-impedance wrench limits must be finite and non-negative", error);

    kinematics::ForwardKinematicsResult fk;
    if (!kinematics::ForwardKinematics::solve(model, to_std(q), fk, error)) return false;
    kinematics::JacobianResult jacobian;
    if (!kinematics::Jacobian::solve(model, to_std(q), jacobian, error)) return false;

    const geometry::Vec3 position_error = equilibrium_pose.position - fk.pose.position;
    const geometry::Quaternion q_error = geometry::normalized(
        geometry::multiply(equilibrium_pose.rotation, geometry::conjugate(fk.pose.rotation)));
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
    if (!twist.allFinite()) return fail("Cartesian-impedance twist is non-finite", error);

    Eigen::VectorXd wrench = feedforward_wrench +
        stiffness.cwiseProduct(pose_error) +
        damping.cwiseProduct(desired_twist - twist);
    if (!wrench.allFinite()) return fail("Cartesian-impedance wrench is non-finite", error);
    if (wrench_limits.size() != 0) {
      for (Eigen::Index i = 0; i < 6; ++i)
        wrench[i] = std::max(-wrench_limits[i], std::min(wrench_limits[i], wrench[i]));
    }
    Eigen::VectorXd candidate = J.transpose() * wrench;
    if (!candidate.allFinite()) return fail("Cartesian-impedance torque is non-finite", error);
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
