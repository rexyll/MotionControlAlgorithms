#include <stdexcept>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mca/control/nonlinear_mpc.hpp"

namespace {
using namespace mca::control;

NonlinearMpcProblem make_exception_test_problem() {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x;
    return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd&) { return 0.0; };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  return p;
}

void expect_transactional_callback_failure(const NonlinearMpcProblem& p) {
  NonlinearMpcOptions o;
  o.horizon=2;
  o.optimization_iterations=1;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  r.cost=42.0;
  r.control=Eigen::VectorXd::Constant(1,7.0);
  EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.cost,42.0);
  ASSERT_EQ(r.control.size(),1);
  EXPECT_DOUBLE_EQ(r.control(0),7.0);
}

TEST(NonlinearMpcCallbackException, ConvertsStandardStageCostExceptionToTransactionalFailure) {
  auto p=make_exception_test_problem();
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd&) -> double {
    throw std::runtime_error("stage cost failure");
  };
  expect_transactional_callback_failure(p);
}

TEST(NonlinearMpcCallbackException, ConvertsNonStandardDynamicsExceptionToTransactionalFailure) {
  auto p=make_exception_test_problem();
  p.dynamics=[](const Eigen::VectorXd&,const Eigen::VectorXd&,Eigen::VectorXd&) -> bool {
    throw 42;
  };
  expect_transactional_callback_failure(p);
}

TEST(NonlinearMpcCallbackException, ConvertsStandardTerminalCostExceptionToTransactionalFailure) {
  auto p=make_exception_test_problem();
  p.terminal_cost=[](const Eigen::VectorXd&) -> double {
    throw std::logic_error("terminal cost failure");
  };
  expect_transactional_callback_failure(p);
}

}  // namespace
