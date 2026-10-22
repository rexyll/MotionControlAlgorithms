#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace mca::control {

struct GenericMpcProblem {
  Eigen::VectorXd initial_state;
  Eigen::VectorXd control_lower;
  Eigen::VectorXd control_upper;
  std::size_t horizon{0U};
};

struct GenericMpcResult {
  Eigen::VectorXd control;
  Eigen::MatrixXd predicted_states;
  Eigen::MatrixXd control_sequence;
  double cost{0.0};
};

struct GenericMpcOptions {
  std::size_t horizon{10U};
};

using MpcSolverFunction = std::function<bool(
    const GenericMpcProblem&, const GenericMpcOptions&, GenericMpcResult&,
    std::string*)>;

inline bool validate_generic_mpc_problem(const GenericMpcProblem& problem,
                                         const GenericMpcOptions& options,
                                         std::string* error = nullptr) {
  const auto fail = [&](const char* message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  const Eigen::Index nx = problem.initial_state.size();
  const Eigen::Index nu = problem.control_lower.size();
  if (nx <= 0 || nu <= 0) {
    return fail("state and control dimensions must be positive");
  }
  if (!problem.initial_state.allFinite() ||
      !problem.control_lower.allFinite() ||
      !problem.control_upper.allFinite()) {
    return fail("state and control bounds must be finite");
  }
  if (problem.control_upper.size() != nu) {
    return fail("control bound dimensions are inconsistent");
  }
  if ((problem.control_lower.array() > problem.control_upper.array()).any()) {
    return fail("control lower bound exceeds upper bound");
  }
  if (problem.horizon == 0U || problem.horizon > 200U ||
      options.horizon != problem.horizon) {
    return fail("MPC horizon is invalid or inconsistent");
  }
  return true;
}

inline bool validate_generic_mpc_result(const GenericMpcProblem& problem,
                                        const GenericMpcResult& result,
                                        std::string* error = nullptr) {
  const auto fail = [&](const char* message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  const Eigen::Index nx = problem.initial_state.size();
  const Eigen::Index nu = problem.control_lower.size();
  const Eigen::Index horizon = static_cast<Eigen::Index>(problem.horizon);
  if (result.control.size() != nu ||
      result.control_sequence.rows() != nu ||
      result.control_sequence.cols() != horizon ||
      result.predicted_states.rows() != nx ||
      result.predicted_states.cols() != horizon + 1) {
    return fail("MPC result dimensions are inconsistent");
  }
  if (!result.control.allFinite() || !result.control_sequence.allFinite() ||
      !result.predicted_states.allFinite() || !std::isfinite(result.cost)) {
    return fail("MPC result contains non-finite values");
  }
  if ((result.control.array() < problem.control_lower.array()).any() ||
      (result.control.array() > problem.control_upper.array()).any() ||
      (result.control_sequence.colwise() < problem.control_lower).any() ||
      (result.control_sequence.colwise() > problem.control_upper).any()) {
    return fail("MPC result violates control bounds");
  }
  return true;
}

inline bool solve_generic_mpc(const GenericMpcProblem& problem,
                              const GenericMpcOptions& options,
                              const MpcSolverFunction& solver,
                              GenericMpcResult& output,
                              std::string* error = nullptr) {
  const auto fail = [&](const char* message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  if (!solver) {
    return fail("MPC solver callback is empty");
  }
  std::string validation_error;
  if (!validate_generic_mpc_problem(problem, options, &validation_error)) {
    return fail(validation_error.c_str());
  }

  GenericMpcResult candidate;
  std::string solver_error;
  if (!solver(problem, options, candidate, &solver_error)) {
    if (error != nullptr) {
      *error = solver_error.empty() ? "MPC solver failed" : solver_error;
    }
    return false;
  }
  if (!validate_generic_mpc_result(problem, candidate, &validation_error)) {
    return fail(validation_error.c_str());
  }

  output = std::move(candidate);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace mca::control
