#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include "mca/dynamics/rigid_body_dynamics.hpp"

namespace mca::dynamics {

// Joint-space computed-torque controller. Desired acceleration includes PD
// tracking feedback; model bias compensation adds Coriolis and gravity terms.
// This is a stateless control law, not a trajectory generator or safety layer.
class ComputedTorque {
 public:
  static bool compute(const kinematics::RobotModel& model,
                      const std::vector<LinkInertial>& inertias,
                      const Eigen::VectorXd& q,
                      const Eigen::VectorXd& dq,
                      const Eigen::VectorXd& q_desired,
                      const Eigen::VectorXd& dq_desired,
                      const Eigen::VectorXd& ddq_desired,
                      const Eigen::VectorXd& kp,
                      const Eigen::VectorXd& kd,
                      geometry::Vec3 gravity_acceleration,
                      const Eigen::VectorXd& absolute_torque_limits,
                      Eigen::VectorXd& out,
                      std::string* error = nullptr) {
    const Eigen::Index n = q.size();
    if (n <= 0 || dq.size() != n || q_desired.size() != n ||
        dq_desired.size() != n || ddq_desired.size() != n ||
        kp.size() != n || kd.size() != n)
      return fail("computed-torque vector dimensions are invalid", error);
    if (!q.allFinite() || !dq.allFinite() || !q_desired.allFinite() ||
        !dq_desired.allFinite() || !ddq_desired.allFinite() ||
        !kp.allFinite() || !kd.allFinite())
      return fail("computed-torque inputs must be finite", error);
    if ((kp.array() < 0.0).any() || (kd.array() < 0.0).any())
      return fail("computed-torque gains must be non-negative", error);
    if (absolute_torque_limits.size() != 0 &&
        absolute_torque_limits.size() != n)
      return fail("computed-torque limits have invalid dimension", error);
    if (absolute_torque_limits.size() != 0 &&
        (!absolute_torque_limits.allFinite() ||
         (absolute_torque_limits.array() < 0.0).any()))
      return fail("computed-torque limits must be finite and non-negative", error);

    Eigen::MatrixXd mass;
    Eigen::VectorXd bias;
    if (!RigidBodyDynamics::mass_matrix(model, inertias, q, mass, error) ||
        !RigidBodyDynamics::bias_forces(model, inertias, q, dq,
                                        gravity_acceleration, bias, error))
      return false;
    const Eigen::VectorXd desired_acceleration =
        ddq_desired + kp.cwiseProduct(q_desired - q) +
        kd.cwiseProduct(dq_desired - dq);
    Eigen::VectorXd candidate = mass * desired_acceleration + bias;
    if (!candidate.allFinite())
      return fail("computed-torque result is non-finite", error);
    if (absolute_torque_limits.size() != 0) {
      for (Eigen::Index i = 0; i < n; ++i)
        candidate[i] = std::max(-absolute_torque_limits[i],
                                std::min(absolute_torque_limits[i], candidate[i]));
    }
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
