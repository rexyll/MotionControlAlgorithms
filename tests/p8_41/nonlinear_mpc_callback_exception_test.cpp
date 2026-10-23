#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mca/control/nonlinear_mpc.hpp"

namespace {
using namespace mca::control;

NonlinearMpcProblem make_problem() {
  NonlinearMpcProblem p;
  p.control_lower = Eigen::VectorXd::Constant(1, -1.0);
  p.control_upper = Eigen::VectorXd::Constant(1, 1.0);
  p.dynamics = [](const Eigen::VectorXd& x, const Eigen::VectorXd&, Eigen::VectorXd& next) {
    next = x;
    return true;
  };
  p.stage_cost = [](const Eigen::VectorXd&, const Eigen::VectorXd& u) { return u(0); };
  p.terminal_cost = [](const Eigen::VectorXd&) { return 0.0; };
  return p;
}

void expect_transactional_failure(const NonlinearMpcProblem& p) {
  NonlinearMpcOptions options;
  options.horizon = 1;
  options.optimization_iterations = 1;
  options.finite_difference_step = 1e-3;
  options.gradient_step = 0.05;

  Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(1);
  NonlinearMpcResult result;
  result.cost = 42.0;
  result.control = Eigen::VectorXd::Constant(1, 7.0);
  result.control_sequence = Eigen::MatrixXd::Constant(1, 1, 8.0);
  std::string error;
  EXPECT_FALSE(solve_nonlinear_mpc(p, initial_state, options, result, &error));
  EXPECT_EQ(error, "nonlinear MPC callback threw an exception");
  EXPECT_DOUBLE_EQ(result.cost, 42.0);
  ASSERT_EQ(result.control.size(), 1);
  EXPECT_DOUBLE_EQ(result.control(0), 7.0);
  ASSERT_EQ(result.control_sequence.rows(), 1);
  ASSERT_EQ(result.control_sequence.cols(), 1);
  EXPECT_DOUBLE_EQ(result.control_sequence(0, 0), 8.0);
}

TEST(NonlinearMpcCallbackException, FiniteDifferenceExceptionIsTransactional) {
  auto p = make_problem();
  p.stage_cost = [](const Eigen::VectorXd&, const Eigen::VectorXd& u) {
    if (u(0) != 0.0) {
      throw std::runtime_error("finite-difference callback failure");
    }
    return 0.0;
  };
  expect_transactional_failure(p);
}

TEST(NonlinearMpcCallbackException, LineSearchExceptionIsTransactional) {
  auto p = make_problem();
  int stage_cost_calls = 0;
  p.stage_cost = [&stage_cost_calls](const Eigen::VectorXd&, const Eigen::VectorXd& u) {
    ++stage_cost_calls;
    if (stage_cost_calls == 4) {
      throw std::runtime_error("line-search callback failure");
    }
    return u(0);
  };
  expect_transactional_failure(p);
}

}  // namespace
