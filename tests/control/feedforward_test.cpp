#include <limits>

#include <gtest/gtest.h>

#include "mca/control/feedforward.hpp"
#include "mca/control/nonlinear_mpc.hpp"

TEST(PidFeedforward, ComputesVelocityAndAccelerationTerms) {
  mca::control::PidFeedforward ff({2.0, 3.0, 0.5});
  EXPECT_DOUBLE_EQ(ff.compute(1.0, 2.0), 8.5);
}

TEST(PidFeedforward, RejectsNonFiniteConfiguration) {
  mca::control::PidFeedforward ff;
  mca::control::PidFeedforwardConfig config;
  config.velocity_gain = std::numeric_limits<double>::infinity();
  EXPECT_EQ(ff.configure(config).code, mca::core::StatusCode::InvalidArgument);
}

TEST(NonlinearMpc, KeepsProjectedGradientRepresentableWhenDisplacementRoundsAway) {
  mca::control::NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,0.0);
  p.control_upper=Eigen::VectorXd::Constant(1,2.0e16);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x;
    return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return u(0);
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };

  mca::control::NonlinearMpcOptions o;
  o.horizon=1U;
  o.optimization_iterations=1U;
  o.gradient_step=1.0e-16;
  o.finite_difference_step=1.0;
  o.projected_gradient_tolerance=0.5;

  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  mca::control::NonlinearMpcResult r;
  ASSERT_TRUE(mca::control::solve_nonlinear_mpc(p,x,o,r));
  EXPECT_GT(r.projected_gradient_norm,0.5);
  EXPECT_EQ(r.termination,mca::control::NonlinearMpcTermination::IterationLimitReached);
  EXPECT_LT(r.control(0),1.0e16);
  EXPECT_EQ(r.line_search_evaluations,1U);
}

TEST(NonlinearMpc, UsesOneSidedFiniteDifferenceWhenLowerBoundClipsProbe) {
  mca::control::NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,0.0);
  p.control_upper=Eigen::VectorXd::Constant(1,2.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x;
    return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return u.squaredNorm();
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };

  mca::control::NonlinearMpcOptions o;
  o.horizon=1U;
  o.optimization_iterations=1U;
  o.gradient_step=0.1;
  o.finite_difference_step=1.0;
  o.initial_control_sequence=Eigen::MatrixXd::Constant(1,1,0.25);

  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  mca::control::NonlinearMpcResult r;
  ASSERT_TRUE(mca::control::solve_nonlinear_mpc(p,x,o,r));

  // At 0.25 the lower-side probe is clipped to 0. A central secant over
  // [0,1.25] would report 1.25, while the proper forward secant reports 1.5.
  // The latter gives the expected first line-search candidate 0.25-0.1*1.5=0.1.
  EXPECT_NEAR(r.control(0),0.1,1e-12);
  EXPECT_NEAR(r.cost,0.01,1e-12);
}
