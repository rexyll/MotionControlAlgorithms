#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace mca::legged {

struct LocomotionMpcInput {
  Eigen::Vector4d initial_state = Eigen::Vector4d::Zero();
  Eigen::MatrixXd desired_states;
  double dt = 0.0;
  Eigen::Vector4d state_weight = Eigen::Vector4d::Ones();
  Eigen::Vector4d terminal_weight = Eigen::Vector4d::Ones();
  Eigen::Vector2d control_weight = Eigen::Vector2d::Ones();
  Eigen::Vector2d acceleration_limits = Eigen::Vector2d::Ones();
  int iterations = 50;
  double step_size = 0.0;
};

struct LocomotionMpcOutput {
  Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
  Eigen::MatrixXd predicted_states;
  Eigen::MatrixXd controls;
  double objective = 0.0;
  int iterations = 0;
};

class LocomotionMpc {
 public:
  bool solve(const LocomotionMpcInput& input, LocomotionMpcOutput* output) const {
    if (output == nullptr || !valid(input)) {
      return false;
    }

    const int horizon = static_cast<int>(input.desired_states.cols()) - 1;
    Eigen::MatrixXd controls = Eigen::MatrixXd::Zero(2, horizon);
    Eigen::MatrixXd states(4, horizon + 1);
    Eigen::MatrixXd gradient(2, horizon);

    const double alpha = input.step_size > 0.0
                             ? input.step_size
                             : 1.0 / (1.0 + input.dt * input.dt);

    double objective = 0.0;
    for (int iteration = 0; iteration < input.iterations; ++iteration) {
      objective = evaluate(input, controls, &states, &gradient);
      controls -= alpha * gradient;
      for (int k = 0; k < horizon; ++k) {
        for (int axis = 0; axis < 2; ++axis) {
          controls(axis, k) = std::max(-input.acceleration_limits(axis),
                                      std::min(input.acceleration_limits(axis), controls(axis, k)));
        }
      }
      if (!controls.allFinite()) {
        return false;
      }
    }

    objective = evaluate(input, controls, &states, &gradient);
    if (!std::isfinite(objective) || !states.allFinite() || !controls.allFinite()) {
      return false;
    }

    LocomotionMpcOutput candidate;
    candidate.acceleration = controls.col(0);
    candidate.predicted_states = states;
    candidate.controls = controls;
    candidate.objective = objective;
    candidate.iterations = input.iterations;
    if (!candidate.acceleration.allFinite()) {
      return false;
    }
    *output = std::move(candidate);
    return true;
  }

 private:
  static bool valid(const LocomotionMpcInput& input) {
    const int horizon = static_cast<int>(input.desired_states.cols()) - 1;
    return input.initial_state.allFinite() && input.desired_states.rows() == 4 &&
           input.desired_states.cols() >= 2 && input.desired_states.allFinite() &&
           std::isfinite(input.dt) && input.dt > 0.0 && input.state_weight.allFinite() &&
           input.state_weight.minCoeff() >= 0.0 && input.terminal_weight.allFinite() &&
           input.terminal_weight.minCoeff() >= 0.0 && input.control_weight.allFinite() &&
           input.control_weight.minCoeff() > 0.0 && input.acceleration_limits.allFinite() &&
           input.acceleration_limits.minCoeff() > 0.0 && input.iterations > 0 && horizon >= 1 &&
           (input.step_size == 0.0 || (std::isfinite(input.step_size) && input.step_size > 0.0));
  }

  static double evaluate(const LocomotionMpcInput& input,
                         const Eigen::MatrixXd& controls,
                         Eigen::MatrixXd* states,
                         Eigen::MatrixXd* gradient) {
    const int horizon = static_cast<int>(controls.cols());
    states->resize(4, horizon + 1);
    *states = Eigen::MatrixXd::Zero(4, horizon + 1);
    states->col(0) = input.initial_state;

    const double dt = input.dt;
    for (int k = 0; k < horizon; ++k) {
      const Eigen::Vector2d p = states->col(k).head<2>();
      const Eigen::Vector2d v = states->col(k).tail<2>();
      states->col(k + 1).head<2>() = p + dt * v + 0.5 * dt * dt * controls.col(k);
      states->col(k + 1).tail<2>() = v + dt * controls.col(k);
    }

    double objective = 0.0;
    for (int k = 0; k < horizon; ++k) {
      const Eigen::Vector4d error = states->col(k) - input.desired_states.col(k);
      objective += (error.array().square() * input.state_weight.array()).sum();
      objective += (controls.col(k).array().square() * input.control_weight.array()).sum();
    }
    const Eigen::Vector4d terminal_error = states->col(horizon) - input.desired_states.col(horizon);
    objective += (terminal_error.array().square() * input.terminal_weight.array()).sum();

    gradient->setZero(2, horizon);
    Eigen::Vector4d adjoint = 2.0 * terminal_error.cwiseProduct(input.terminal_weight);
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    A(0, 2) = dt;
    A(1, 3) = dt;
    Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();
    B(0, 0) = 0.5 * dt * dt;
    B(1, 1) = 0.5 * dt * dt;
    B(2, 0) = dt;
    B(3, 1) = dt;

    for (int k = horizon - 1; k >= 0; --k) {
      const Eigen::Vector4d error = states->col(k) - input.desired_states.col(k);
      const Eigen::Vector2d control_gradient =
          2.0 * controls.col(k).cwiseProduct(input.control_weight) + B.transpose() * adjoint;
      (*gradient).col(k) = control_gradient;
      adjoint = A.transpose() * adjoint + 2.0 * error.cwiseProduct(input.state_weight);
    }
    return objective;
  }
};

}  // namespace mca::legged
