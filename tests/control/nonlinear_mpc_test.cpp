#include <cmath>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "mca/control/nonlinear_mpc.hpp"

namespace {
using namespace mca::control;

NonlinearMpcProblem makeProblem() {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u,Eigen::VectorXd& next){
    next.resize(1); next(0)=x(0)+0.2*std::tanh(u(0)); return true;
  };
  p.stage_cost=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u){ return x.squaredNorm()+0.01*u.squaredNorm(); };
  p.terminal_cost=[](const Eigen::VectorXd& x){ return 2.0*x.squaredNorm(); };
  return p;
}

TEST(NonlinearMpc, DrivesScalarStateTowardZero) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=8; o.optimization_iterations=35; o.gradient_step=0.12;
  Eigen::VectorXd x(1); x<<1.0; NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  ASSERT_EQ(r.control.size(),1); ASSERT_EQ(r.predicted_states.cols(),9);
  EXPECT_LT(std::abs(r.predicted_states(0,8)),0.75);
  EXPECT_LE(r.control(0),1.0); EXPECT_GE(r.control(0),-1.0);
  EXPECT_TRUE(std::isfinite(r.cost));
}

TEST(NonlinearMpc, RespectsBoundsAndTransactionalFailure) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=4; o.optimization_iterations=5;
  Eigen::VectorXd x(1); x<<0.5; NonlinearMpcResult r; r.control=Eigen::VectorXd::Constant(1,42.0);
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r)); EXPECT_LE(r.control(0),1.0); EXPECT_GE(r.control(0),-1.0);
  NonlinearMpcProblem bad=p; bad.control_lower(0)=2.0; bad.control_upper(0)=1.0;
  const double old=r.control(0); EXPECT_FALSE(solve_nonlinear_mpc(bad,x,o,r)); EXPECT_DOUBLE_EQ(r.control(0),old);
}

TEST(NonlinearMpc, RejectsInvalidCallbacksAndOptions) {
  auto p=makeProblem(); Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  NonlinearMpcOptions o; o.horizon=0; EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  p.dynamics={}; o.horizon=4; EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
}
}
