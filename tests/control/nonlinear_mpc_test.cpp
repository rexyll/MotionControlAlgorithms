#include <cmath>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "mca/control/nonlinear_mpc.hpp"
#include "mca/control/constrained_qp.hpp"

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

TEST(ConstrainedQp, SolvesBoxConstrainedConvexProblem) {
  ConstrainedQpProblem p;
  p.hessian=2.0*Eigen::MatrixXd::Identity(2,2);
  p.gradient=Eigen::Vector2d(-4.0,-6.0);
  p.lower_bound=Eigen::Vector2d(-1.0,-1.0);
  p.upper_bound=Eigen::Vector2d(1.0,1.5);
  ConstrainedQpOptions o; o.max_iterations=500U;
  ConstrainedQpResult r;
  ASSERT_TRUE(solve_constrained_qp(p,o,r));
  EXPECT_NEAR(r.solution(0),1.0,1e-5);
  EXPECT_NEAR(r.solution(1),1.5,1e-5);
  EXPECT_NEAR(r.objective,-8.25,1e-5);
  EXPECT_LT(r.primal_residual,1e-5);
}

TEST(ConstrainedQp, HandlesEqualityAndBounds) {
  ConstrainedQpProblem p;
  p.hessian=2.0*Eigen::MatrixXd::Identity(2,2);
  p.gradient=Eigen::Vector2d(-2.0,-4.0);
  p.equality_matrix=Eigen::MatrixXd::Zero(1,2); p.equality_matrix(0,0)=1.0; p.equality_matrix(0,1)=1.0;
  p.equality_rhs=Eigen::VectorXd::Constant(1,1.0);
  p.lower_bound=Eigen::Vector2d(0.0,0.0);
  p.upper_bound=Eigen::Vector2d(1.0,1.0);
  ConstrainedQpOptions o; o.max_iterations=1000U; o.rho=1.0;
  ConstrainedQpResult r;
  ASSERT_TRUE(solve_constrained_qp(p,o,r));
  EXPECT_NEAR(r.solution(0),0.0,2e-4);
  EXPECT_NEAR(r.solution(1),1.0,2e-4);
  EXPECT_NEAR((p.equality_matrix*r.solution-p.equality_rhs).norm(),0.0,2e-4);
}

TEST(ConstrainedQp, RejectsInvalidInputTransactionally) {
  ConstrainedQpProblem p;
  p.hessian=Eigen::MatrixXd::Identity(1,1);
  p.gradient=Eigen::VectorXd::Zero(1);
  p.lower_bound=Eigen::VectorXd::Constant(1,1.0);
  p.upper_bound=Eigen::VectorXd::Constant(1,0.0);
  ConstrainedQpResult r; r.solution=Eigen::VectorXd::Constant(1,42.0);
  const double old=r.solution(0);
  EXPECT_FALSE(solve_constrained_qp(p,ConstrainedQpOptions{},r));
  EXPECT_DOUBLE_EQ(r.solution(0),old);
}

TEST(ConstrainedQp, RejectsNonConvexHessian) {
  ConstrainedQpProblem p;
  p.hessian=Eigen::MatrixXd::Identity(2,2); p.hessian(0,0)=-1.0;
  p.gradient=Eigen::VectorXd::Zero(2);
  p.lower_bound=Eigen::VectorXd::Constant(2,-1.0);
  p.upper_bound=Eigen::VectorXd::Constant(2,1.0);
  ConstrainedQpResult r;
  EXPECT_FALSE(solve_constrained_qp(p,ConstrainedQpOptions{},r));
}
}
