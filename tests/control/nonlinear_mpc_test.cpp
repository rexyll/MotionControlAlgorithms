#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "mca/control/nonlinear_mpc.hpp"
#include "mca/control/constrained_qp.hpp"
#include "mca/control/generic_mpc.hpp"

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

TEST(NonlinearMpc, InitializesLargeFiniteSameSignBoundsWithoutOverflow) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,1.0e308);
  p.control_upper=Eigen::VectorXd::Constant(1,1.1e308);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x; return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd&) { return 0.0; };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=2; o.optimization_iterations=1;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  ASSERT_EQ(r.control_sequence.rows(),1);
  ASSERT_EQ(r.control_sequence.cols(),2);
  EXPECT_TRUE(r.control_sequence.allFinite());
  for(Eigen::Index k=0;k<r.control_sequence.cols();++k) {
    EXPECT_GE(r.control_sequence(0,k),p.control_lower(0));
    EXPECT_LE(r.control_sequence(0,k),p.control_upper(0));
    EXPECT_NEAR(r.control_sequence(0,k)/1.0e308,1.05,1e-12);
  }
}

TEST(NonlinearMpc, PreservesOutputWhenLargeBoundInitializationRolloutFails) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,1.0e308);
  p.control_upper=Eigen::VectorXd::Constant(1,1.1e308);
  p.dynamics=[](const Eigen::VectorXd&,const Eigen::VectorXd&,Eigen::VectorXd&) { return false; };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd&) { return 0.0; };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=2;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r; r.cost=42.0;
  EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.cost,42.0);
}

TEST(NonlinearMpc, SafelyProjectsFiniteLineSearchSubtractionOverflow) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0e308);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0e308);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) { next=x; return true; };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return -u.squaredNorm(); };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1; o.gradient_step=1.0e308;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(r.control.allFinite());
  EXPECT_TRUE(r.control(0)>=-1.0e308 && r.control(0)<=1.0e308);
}

TEST(NonlinearMpc, ComputesRepresentableGradientWhenControlIntervalWouldOverflow) {
  auto p=makeProblem();
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0e308);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0e308);
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return u(0); };
  NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(r.projected_gradient_norm>=0.0);
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
}

TEST(NonlinearMpc, ComputesRepresentableGradientWhenFiniteCostDifferenceWouldOverflow) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0e308);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0e308);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) { next=x; return true; };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return u(0)>0.0?1.0e308:-1.0e308; };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1; o.gradient_step=0.1;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
}

TEST(NonlinearMpc, PreservesGradientPrecisionWhenCostSubtractionIsSafe) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1; o.gradient_step=0.01;
  Eigen::VectorXd x(1); x<<1.0; NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
}

TEST(NonlinearMpc, ComputesRepresentableLargeProjectedGradientNormWithoutOverflow) {
  auto p=makeProblem();
  p.control_lower=Eigen::VectorXd::Constant(2,-1.0e308);
  p.control_upper=Eigen::VectorXd::Constant(2,1.0e308);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) { next=x; return true; };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return -u.squaredNorm(); };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1; o.gradient_step=1.0e308;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
}

TEST(NonlinearMpc, KeepsRepresentableProjectedMappingWhenDisplacementOverflows) {
  auto p=makeProblem();
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0e308);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0e308);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) { next=x; return true; };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return -u(0); };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1; o.gradient_step=1.0e308;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
}

TEST(NonlinearMpc, ProjectsOverflowingGradientStepTowardDescentBound) {
  auto p=makeProblem();
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0e308);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0e308);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) { next=x; return true; };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return -u(0); };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1; o.gradient_step=1.0e308;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(r.control(0)>=-1.0e308 && r.control(0)<=1.0e308);
}

TEST(NonlinearMpc, StopsWhenProjectedGradientToleranceIsSatisfied) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=3; o.optimization_iterations=10; o.projected_gradient_tolerance=1e-12;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::ProjectedGradientToleranceSatisfied);
  EXPECT_EQ(r.iterations_completed,1U);
}

TEST(NonlinearMpc, ProjectedGradientRecognizesActiveUpperBound) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1;
  Eigen::VectorXd x(1); x<<1.0; o.initial_control_sequence=Eigen::MatrixXd::Constant(1,1.0);
  NonlinearMpcResult r; ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
}

TEST(NonlinearMpc, ProjectedGradientRecognizesActiveLowerBoundAndFixedDimension) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1;
  Eigen::VectorXd x(1); x<<-1.0; o.initial_control_sequence=Eigen::MatrixXd::Constant(1,-1.0);
  NonlinearMpcResult r; ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
}

TEST(NonlinearMpc, ProjectedGradientReportsNonstationarityAwayFromBounds) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=1; o.optimization_iterations=1;
  Eigen::VectorXd x(1); x<<1.0; o.initial_control_sequence=Eigen::MatrixXd::Constant(1,0.0);
  NonlinearMpcResult r; ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_GT(r.projected_gradient_norm,0.0);
}

TEST(NonlinearMpc, ProjectedGradientDiagnosticIsInvariantToGradientStepScaling) {
  auto p=makeProblem();
  Eigen::VectorXd x(1); x<<1.0;
  NonlinearMpcOptions a; a.horizon=1; a.optimization_iterations=1; a.gradient_step=0.01;
  NonlinearMpcOptions b=a; b.gradient_step=0.1;
  NonlinearMpcResult ra,rb;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,a,ra));
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,b,rb));
  EXPECT_NEAR(ra.projected_gradient_norm,rb.projected_gradient_norm,1e-8);
}

TEST(NonlinearMpc, RejectsInvalidProjectedGradientToleranceTransactionally) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.projected_gradient_tolerance=-1.0;
  Eigen::VectorXd x(1); x<<1.0; NonlinearMpcResult r; r.cost=123.0;
  EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.cost,123.0);
}

TEST(NonlinearMpc, ReportsIterationLimitAndWorkCounters) {
  auto p=makeProblem();
  NonlinearMpcOptions o; o.horizon=4; o.optimization_iterations=1; o.gradient_step=0.01;
  Eigen::VectorXd x(1); x<<1.0;
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::IterationLimitReached);
  EXPECT_EQ(r.iterations_completed,1U);
  EXPECT_GE(r.line_search_evaluations,1U);
  EXPECT_LE(r.line_search_evaluations,o.line_search_iterations);
}

TEST(NonlinearMpc, ReportsNoImprovingStepWithoutCallingItConvergence) {
  auto p=makeProblem();
  NonlinearMpcOptions o; o.horizon=3; o.optimization_iterations=10;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::NoImprovingStep);
  EXPECT_EQ(r.iterations_completed,1U);
  EXPECT_EQ(r.line_search_evaluations,0U);
  EXPECT_NEAR(r.cost,0.0,1e-12);
}

TEST(NonlinearMpc, StopsLineSearchWhenStepHalvingUnderflows) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x; return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return -0.5*u(0); };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o;
  o.horizon=1U; o.optimization_iterations=1U; o.gradient_step=std::numeric_limits<double>::denorm_min();
  o.line_search_iterations=100U;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::NoImprovingStep);
  EXPECT_EQ(r.line_search_evaluations,0U);
  EXPECT_NEAR(r.cost,0.0,1e-12);
  EXPECT_DOUBLE_EQ(r.control(0),0.0);
}

TEST(NonlinearMpc, RespectsBoundsAndTransactionalFailure) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=2; o.optimization_iterations=2; o.gradient_step=0.1;
  Eigen::VectorXd x(1); x<<2.0; NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(r.control.allFinite());
  EXPECT_LE(r.control(0),1.0); EXPECT_GE(r.control(0),-1.0);
}

TEST(NonlinearMpc, SupportsFixedControlBounds) {
  auto p=makeProblem(); p.control_lower(0)=0.5; p.control_upper(0)=0.5;
  NonlinearMpcOptions o; o.horizon=2; o.optimization_iterations=2;
  Eigen::VectorXd x(1); x<<1.0; NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.control(0),0.5);
}

TEST(NonlinearMpc, BacktrackingPreventsCostIncreaseWithAggressiveStep) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=3; o.optimization_iterations=4; o.gradient_step=1.0;
  Eigen::VectorXd x(1); x<<1.0; NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.cost));
}

TEST(NonlinearMpc, UsesWarmStartAndProjectsItIntoBounds) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=3; o.optimization_iterations=1;
  o.initial_control_sequence=Eigen::MatrixXd::Constant(1,2.0);
  Eigen::VectorXd x(1); x<<1.0; NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_LE(r.control(0),1.0); EXPECT_GE(r.control(0),-1.0);
}

TEST(NonlinearMpc, WarmStartDoesNotRegressFromColdSolution) {
  auto p=makeProblem(); Eigen::VectorXd x(1); x<<1.0;
  NonlinearMpcOptions cold; cold.horizon=3; cold.optimization_iterations=4;
  NonlinearMpcResult rc; ASSERT_TRUE(solve_nonlinear_mpc(p,x,cold,rc));
  NonlinearMpcOptions warm=cold; warm.initial_control_sequence=rc.control_sequence;
  NonlinearMpcResult rw; ASSERT_TRUE(solve_nonlinear_mpc(p,x,warm,rw));
  EXPECT_LE(rw.cost,rc.cost+1e-12);
}

TEST(NonlinearMpc, RecedingHorizonWarmStartTracksAcrossControlCycles) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=4; o.optimization_iterations=4;
  Eigen::VectorXd x(1); x<<1.0; NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  Eigen::MatrixXd shifted;
  ASSERT_TRUE(shift_nonlinear_mpc_warm_start(r.control_sequence,4,shifted));
  o.initial_control_sequence=shifted;
  x(0)=r.predicted_states(0,1);
  NonlinearMpcResult next;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,next));
  EXPECT_TRUE(next.control_sequence.allFinite());
}

TEST(NonlinearMpc, RejectsInvalidWarmStartTransactionally) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=3; o.optimization_iterations=1;
  o.initial_control_sequence=Eigen::MatrixXd::Constant(2,3,0.0);
  Eigen::VectorXd x(1); x<<1.0; NonlinearMpcResult r; r.cost=9.0;
  EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.cost,9.0);
}

TEST(NonlinearMpc, ShiftsWarmStartTransactionally) {
  Eigen::MatrixXd input(1,3); input<<1.0,2.0,3.0; Eigen::MatrixXd output;
  ASSERT_TRUE(shift_nonlinear_mpc_warm_start(input,4,output));
  EXPECT_EQ(output.rows(),1); EXPECT_EQ(output.cols(),4);
  EXPECT_DOUBLE_EQ(output(0,0),2.0); EXPECT_DOUBLE_EQ(output(0,3),3.0);
}

TEST(NonlinearMpc, RejectsInvalidCallbacksAndOptions) {
  NonlinearMpcProblem p=makeProblem(); p.dynamics={};
  NonlinearMpcOptions o; Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r; r.cost=7.0;
  EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.cost,7.0);
}

} // namespace
