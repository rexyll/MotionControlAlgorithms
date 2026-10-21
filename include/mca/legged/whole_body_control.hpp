#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace mca::legged {

// A task Jacobian maps generalized velocity to task velocity; task_wrench is
// the corresponding generalized force. The contribution is J^T * wrench.
struct WholeBodyTask {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd task_wrench;
};

// Deterministic baseline whole-body torque composition (not a QP solver).
// tau = feedforward + sum(J_i^T * wrench_i) + secondary_torque, then clamp.
struct WholeBodyControlInput {
  Eigen::Index dof{0};
  Eigen::VectorXd feedforward_torque;
  Eigen::VectorXd secondary_torque;
  Eigen::VectorXd torque_limits;
  std::vector<WholeBodyTask> tasks;
};

class WholeBodyControl {
 public:
  static bool compute(const WholeBodyControlInput& input,
                      Eigen::VectorXd& output,
                      std::string* error = nullptr) {
    const Eigen::Index n = input.dof;
    if (n <= 0) return fail("whole-body dof must be positive", error);
    if (input.feedforward_torque.size() != n ||
        input.secondary_torque.size() != n)
      return fail("whole-body torque vectors must match dof", error);
    if (!input.feedforward_torque.allFinite() ||
        !input.secondary_torque.allFinite())
      return fail("whole-body torque vectors must be finite", error);
    if (input.torque_limits.size() != 0 &&
        input.torque_limits.size() != n)
      return fail("whole-body torque limits must be empty or match dof", error);
    if (input.torque_limits.size() != 0 &&
        (!input.torque_limits.allFinite() ||
         (input.torque_limits.array() < 0.0).any()))
      return fail("whole-body torque limits must be finite and non-negative", error);

    Eigen::VectorXd candidate =
        input.feedforward_torque + input.secondary_torque;
    for (const auto& task : input.tasks) {
      if (task.jacobian.rows() <= 0 || task.jacobian.cols() != n ||
          task.task_wrench.size() != task.jacobian.rows())
        return fail("whole-body task dimensions are inconsistent", error);
      if (!task.jacobian.allFinite() || !task.task_wrench.allFinite())
        return fail("whole-body task inputs must be finite", error);
      candidate.noalias() += task.jacobian.transpose() * task.task_wrench;
    }
    if (!candidate.allFinite())
      return fail("whole-body torque computation produced non-finite output", error);
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
