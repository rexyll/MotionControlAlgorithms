#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include <utility>

namespace mca::legged {

// Joint-space impedance torque law:
// tau = tau_ff + K (q_des - q) + D (dq_des - dq), followed by
// symmetric per-joint torque saturation. This is a stateless control law;
// it does not integrate state or perform model-based compensation.
struct JointImpedanceControlInput {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd desired_position;
  Eigen::VectorXd desired_velocity;
  Eigen::VectorXd stiffness;
  Eigen::VectorXd damping;
  Eigen::VectorXd feedforward_torque;
  Eigen::VectorXd torque_limits;
};

class JointImpedanceControl {
 public:
  static bool compute(const JointImpedanceControlInput& input,
                      Eigen::VectorXd& output,
                      std::string* error = nullptr) {
    const Eigen::Index n = input.position.size();
    if (n <= 0 || input.velocity.size() != n ||
        input.desired_position.size() != n ||
        input.desired_velocity.size() != n || input.stiffness.size() != n ||
        input.damping.size() != n || input.feedforward_torque.size() != n)
      return fail("joint-impedance vector dimensions are invalid", error);
    if (!input.position.allFinite() || !input.velocity.allFinite() ||
        !input.desired_position.allFinite() ||
        !input.desired_velocity.allFinite() || !input.stiffness.allFinite() ||
        !input.damping.allFinite() || !input.feedforward_torque.allFinite())
      return fail("joint-impedance inputs must be finite", error);
    if ((input.stiffness.array() < 0.0).any() ||
        (input.damping.array() < 0.0).any())
      return fail("joint-impedance gains must be non-negative", error);
    if (input.torque_limits.size() != 0 &&
        input.torque_limits.size() != n)
      return fail("joint-impedance torque limits must be empty or match dof",
                   error);
    if (input.torque_limits.size() != 0 &&
        (!input.torque_limits.allFinite() ||
         (input.torque_limits.array() < 0.0).any()))
      return fail("joint-impedance torque limits must be finite and non-negative",
                  error);

    Eigen::VectorXd candidate =
        input.feedforward_torque +
        input.stiffness.cwiseProduct(input.desired_position - input.position) +
        input.damping.cwiseProduct(input.desired_velocity - input.velocity);
    if (!candidate.allFinite())
      return fail("joint-impedance result is non-finite", error);
    if (input.torque_limits.size() != 0) {
      for (Eigen::Index i = 0; i < n; ++i)
        candidate[i] = std::max(-input.torque_limits[i],
                                std::min(input.torque_limits[i], candidate[i]));
    }
    output = std::move(candidate);
    return true;
  }

 private:
  static bool fail(const char* message, std::string* error) {
    if (error) *error = message;
    return false;
  }
};

}  // namespace mca::legged
