#include <cstddef>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "mca/control/nonlinear_mpc.hpp"

namespace {
using namespace mca::control;

NonlinearMpcProblem makeProblem(std::size_t* stage_calls) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x;
    return true;
  };
  p.stage_cost=[stage_calls](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    ++(*stage_calls);
    return 0.5*u.squaredNorm();
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  return p;
}

TEST(NonlinearMpcP846, ReusesCurrentCostForClippedUpperProbe) {
  std::size_t stage_calls=0U;
  auto p=makeProblem(&stage_calls);
  NonlinearMpcOptions o;
  o.horizon=1U;
  o.optimization_iterations=1U;
  o.finite_difference_step=0.25;
  o.gradient_step=0.1;
  o.initial_control_sequence=Eigen::MatrixXd::Constant(1,1,1.0);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;

  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.line_search_evaluations,1U);
  EXPECT_DOUBLE_EQ(r.cost,0.41632812499999999);
  EXPECT_DOUBLE_EQ(r.control(0),0.91249999999999998);
  // Initial rollout + one finite-difference rollout + one accepted line-search rollout.
  // The +h probe is clipped to the current upper bound and must reuse current_cost;
  // a redundant rollout would increase stage_calls to four.
  EXPECT_EQ(stage_calls,3U);
}

TEST(NonlinearMpcP846, ReusesCurrentCostForClippedLowerProbe) {
  std::size_t stage_calls=0U;
  auto p=makeProblem(&stage_calls);
  NonlinearMpcOptions o;
  o.horizon=1U;
  o.optimization_iterations=1U;
  o.finite_difference_step=0.25;
  o.gradient_step=0.1;
  o.initial_control_sequence=Eigen::MatrixXd::Constant(1,1,-1.0);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;

  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.line_search_evaluations,1U);
  EXPECT_DOUBLE_EQ(r.cost,0.41632812499999999);
  EXPECT_DOUBLE_EQ(r.control(0),-0.91249999999999998);
  // Initial rollout + one finite-difference rollout + one accepted line-search rollout.
  // The -h probe is clipped to the current lower bound and must reuse current_cost.
  EXPECT_EQ(stage_calls,3U);
}

TEST(NonlinearMpcP846, CentralDifferenceStillEvaluatesBothFullProbes) {
  std::size_t stage_calls=0U;
  auto p=makeProblem(&stage_calls);
  NonlinearMpcOptions o;
  o.horizon=1U;
  o.optimization_iterations=1U;
  o.finite_difference_step=0.25;
  o.gradient_step=0.1;
  o.initial_control_sequence=Eigen::MatrixXd::Constant(1,1,0.0);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;

  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.line_search_evaluations,0U);
  // Initial rollout + both full central-difference probes.
  EXPECT_EQ(stage_calls,3U);
  EXPECT_NEAR(r.projected_gradient_norm,0.0,1e-12);
}

} // namespace
