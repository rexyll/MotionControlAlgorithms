#include <cmath>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mca/control/nonlinear_mpc.hpp"

namespace {
using namespace mca::control;

TEST(NonlinearMpcProjectedGradient, PreservesTinyInteriorGradientWhenStepProductUnderflows) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x;
    return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return 1.0e-200*u(0);
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };

  NonlinearMpcOptions o;
  o.horizon=1;
  o.optimization_iterations=1;
  o.finite_difference_step=1.0;
  o.gradient_step=1.0e-200;
  o.projected_gradient_tolerance=1.0e-201;

  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::NoImprovingStep);
  EXPECT_EQ(r.iterations_completed,1U);
  EXPECT_EQ(r.line_search_evaluations,1U);
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
  EXPECT_NEAR(r.projected_gradient_norm,1.0e-200,1.0e-215);
}

TEST(NonlinearMpcProjectedGradient, KeepsOutwardActiveBoundGradientStationaryWhenStepProductUnderflows) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,0.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x;
    return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return 1.0e-200*u(0);
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };

  NonlinearMpcOptions o;
  o.horizon=1;
  o.optimization_iterations=1;
  o.finite_difference_step=0.5;
  o.gradient_step=1.0e-200;
  o.projected_gradient_tolerance=1.0e-201;
  o.initial_control_sequence=Eigen::MatrixXd::Zero(1,1);

  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::ProjectedGradientToleranceSatisfied);
  EXPECT_EQ(r.iterations_completed,1U);
  EXPECT_EQ(r.line_search_evaluations,0U);
  EXPECT_DOUBLE_EQ(r.projected_gradient_norm,0.0);
  EXPECT_DOUBLE_EQ(r.control(0),0.0);
}

}  // namespace
