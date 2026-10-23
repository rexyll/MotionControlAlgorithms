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

TEST(NonlinearMpc, ComputesRepresentableGradientWhenFiniteCostDifferenceWouldOverflow) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-8.0e307);
  p.control_upper=Eigen::VectorXd::Constant(1,8.0e307);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x; return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    if(u(0)>0.0) return 1.2e308;
    if(u(0)<0.0) return -1.2e308;
    return 0.0;
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o;
  o.horizon=1; o.optimization_iterations=2; o.finite_difference_step=8.0e307;
  o.gradient_step=0.05;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(r.control_sequence.allFinite());
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
  EXPECT_LT(r.control(0),0.0);
  EXPECT_DOUBLE_EQ(r.cost,-1.2e308);
}

TEST(NonlinearMpc, PreservesGradientPrecisionWhenCostSubtractionIsSafe) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0e150);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0e150);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x; return true;
  };
  const double high=1.0e308;
  const double low=std::nextafter(high,0.0);
  p.stage_cost=[high,low](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return u(0)<0.0?low:high;
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o;
  o.horizon=1; o.optimization_iterations=1;
  o.finite_difference_step=1.0e150;
  o.gradient_step=1.0e-150;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  const double expected=(high-low)/(2.0e150);
  EXPECT_TRUE(std::isfinite(expected));
  EXPECT_NEAR(r.projected_gradient_norm/expected,1.0,1.0e-12);
  EXPECT_LT(r.control(0),0.0);
  EXPECT_DOUBLE_EQ(r.cost,low);
}

TEST(NonlinearMpc, ComputesRepresentableLargeProjectedGradientNormWithoutOverflow) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x; return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return 1.0e200*u(0);
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o;
  o.horizon=1; o.optimization_iterations=1;
  o.finite_difference_step=1.0;
  o.gradient_step=1.0e-200;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
  EXPECT_NEAR(r.projected_gradient_norm/1.0e200,1.0,1.0e-12);
  EXPECT_LT(r.control(0),0.0);
  EXPECT_DOUBLE_EQ(r.cost,-1.0e200);
}

TEST(NonlinearMpc, KeepsRepresentableProjectedMappingWhenDisplacementOverflows) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.7e308);
  p.control_upper=Eigen::VectorXd::Constant(1,1.7e308);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x; return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return 1.5*u(0);
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o;
  o.horizon=1; o.optimization_iterations=1;
  o.finite_difference_step=1.0e300;
  o.gradient_step=1.5e308;
  o.initial_control_sequence=Eigen::MatrixXd::Constant(1,1,1.0e308);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
  EXPECT_NEAR(r.projected_gradient_norm,1.8,1.0e-6);
  EXPECT_TRUE(r.control_sequence.allFinite());
  EXPECT_TRUE(std::isfinite(r.cost));
}

TEST(NonlinearMpc, ProjectsOverflowingGradientStepTowardDescentBound) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x; return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return 1.0e308*u(0);
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o;
  o.horizon=1; o.optimization_iterations=1;
  o.finite_difference_step=1.0e-5;
  o.gradient_step=10.0;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.control(0),-1.0);
  EXPECT_TRUE(std::isfinite(r.projected_gradient_norm));
  EXPECT_NEAR(r.projected_gradient_norm,0.1,1.0e-12);
  EXPECT_DOUBLE_EQ(r.cost,-1.0e308);
}

TEST(NonlinearMpc, StopsWhenProjectedGradientToleranceIsSatisfied) {
  auto p=makeProblem();
  NonlinearMpcOptions o; o.horizon=4; o.optimization_iterations=20;
  o.projected_gradient_tolerance=1e-8;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::ProjectedGradientToleranceSatisfied);
  EXPECT_EQ(r.iterations_completed,1U);
  EXPECT_EQ(r.line_search_evaluations,0U);
  EXPECT_LE(r.projected_gradient_norm,o.projected_gradient_tolerance);
  EXPECT_NEAR(r.cost,0.0,1e-12);
}

TEST(NonlinearMpc, ProjectedGradientRecognizesActiveUpperBound) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd& u,Eigen::VectorXd& next) {
    next=x; (void)u; return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return -u(0); };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=3; o.optimization_iterations=10;
  o.projected_gradient_tolerance=1e-10;
  o.initial_control_sequence=Eigen::MatrixXd::Ones(1,3);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::ProjectedGradientToleranceSatisfied);
  EXPECT_EQ(r.iterations_completed,1U);
  EXPECT_EQ(r.line_search_evaluations,0U);
  EXPECT_LE(r.projected_gradient_norm,o.projected_gradient_tolerance);
  EXPECT_NEAR(r.cost,-3.0,1e-12);
  EXPECT_TRUE((r.control_sequence.array()==1.0).all());
}


TEST(NonlinearMpc, ProjectedGradientRecognizesActiveLowerBoundAndFixedDimension) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::Vector2d(-1.0,0.25);
  p.control_upper=Eigen::Vector2d(1.0,0.25);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) {
    next=x; return true;
  };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) {
    return u(0)+7.0*u(1);
  };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  NonlinearMpcOptions o; o.horizon=2; o.optimization_iterations=10;
  o.projected_gradient_tolerance=1e-10;
  o.initial_control_sequence.resize(2,2);
  o.initial_control_sequence.row(0).setConstant(-1.0);
  o.initial_control_sequence.row(1).setConstant(0.25);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_EQ(r.termination,NonlinearMpcTermination::ProjectedGradientToleranceSatisfied);
  EXPECT_EQ(r.iterations_completed,1U);
  EXPECT_EQ(r.line_search_evaluations,0U);
  EXPECT_LE(r.projected_gradient_norm,o.projected_gradient_tolerance);
  EXPECT_NEAR(r.cost,1.5,1e-12);
  EXPECT_TRUE((r.control_sequence.row(0).array()==-1.0).all());
  EXPECT_TRUE((r.control_sequence.row(1).array()==0.25).all());
}

TEST(NonlinearMpc, ProjectedGradientReportsNonstationarityAwayFromBounds) {
  auto p=makeProblem();
  NonlinearMpcOptions o; o.horizon=3; o.optimization_iterations=1;
  o.projected_gradient_tolerance=1e-12;
  Eigen::VectorXd x(1); x<<0.8;
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_GT(r.projected_gradient_norm,o.projected_gradient_tolerance);
  EXPECT_NE(r.termination,NonlinearMpcTermination::ProjectedGradientToleranceSatisfied);
}

TEST(NonlinearMpc, ProjectedGradientDiagnosticIsInvariantToGradientStepScaling) {
  NonlinearMpcProblem p;
  p.control_lower=Eigen::VectorXd::Constant(1,-2.0);
  p.control_upper=Eigen::VectorXd::Constant(1,2.0);
  p.dynamics=[](const Eigen::VectorXd& x,const Eigen::VectorXd&,Eigen::VectorXd& next) { next=x; return true; };
  p.stage_cost=[](const Eigen::VectorXd&,const Eigen::VectorXd& u) { return 0.5*u.squaredNorm(); };
  p.terminal_cost=[](const Eigen::VectorXd&) { return 0.0; };
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);

  NonlinearMpcOptions small_step;
  small_step.horizon=1; small_step.optimization_iterations=1; small_step.gradient_step=0.25;
  small_step.initial_control_sequence=Eigen::MatrixXd::Constant(1,1,0.5);
  NonlinearMpcResult small_result;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,small_step,small_result));

  NonlinearMpcOptions large_step=small_step;
  large_step.gradient_step=1.0;
  NonlinearMpcResult large_result;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,large_step,large_result));

  EXPECT_NEAR(small_result.projected_gradient_norm,large_result.projected_gradient_norm,1e-6);
  EXPECT_NEAR(small_result.projected_gradient_norm,0.5,1e-6);
}

TEST(NonlinearMpc, RejectsInvalidProjectedGradientToleranceTransactionally) {
  auto p=makeProblem();
  NonlinearMpcOptions o; o.projected_gradient_tolerance=-1.0;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(1);
  NonlinearMpcResult r; r.cost=123.0;
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
  EXPECT_EQ(r.line_search_evaluations,o.line_search_iterations);
  EXPECT_NEAR(r.cost,0.0,1e-12);
}

TEST(NonlinearMpc, RespectsBoundsAndTransactionalFailure) {
  auto p=makeProblem(); NonlinearMpcOptions o; o.horizon=4; o.optimization_iterations=5;
  Eigen::VectorXd x(1); x<<0.5; NonlinearMpcResult r; r.control=Eigen::VectorXd::Constant(1,42.0);
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r)); EXPECT_LE(r.control(0),1.0); EXPECT_GE(r.control(0),-1.0);
  NonlinearMpcProblem bad=p; bad.control_lower(0)=2.0; bad.control_upper(0)=1.0;
  const double old=r.control(0); EXPECT_FALSE(solve_nonlinear_mpc(bad,x,o,r)); EXPECT_DOUBLE_EQ(r.control(0),old);
}

TEST(NonlinearMpc, SupportsFixedControlBounds) {
  auto p=makeProblem();
  p.control_lower=Eigen::VectorXd::Constant(1,0.25);
  p.control_upper=Eigen::VectorXd::Constant(1,0.25);
  NonlinearMpcOptions o; o.horizon=4; o.optimization_iterations=8;
  Eigen::VectorXd x(1); x<<0.5;
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_NEAR(r.control(0),0.25,1e-12);
  EXPECT_TRUE((r.control_sequence.array()==0.25).all());
  EXPECT_TRUE(std::isfinite(r.cost));
}

TEST(NonlinearMpc, BacktrackingPreventsCostIncreaseWithAggressiveStep) {
  auto p=makeProblem();
  NonlinearMpcOptions o; o.horizon=8; o.optimization_iterations=35; o.gradient_step=100.0;
  Eigen::VectorXd x(1); x<<1.0;
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  // The initial midpoint sequence is all zeros: 8 unit stage costs + terminal cost 2.
  EXPECT_LE(r.cost,10.0+1e-10);
  EXPECT_LT(r.cost,10.0);
}

TEST(NonlinearMpc, UsesWarmStartAndProjectsItIntoBounds) {
  auto p=makeProblem();
  NonlinearMpcOptions o; o.horizon=4; o.optimization_iterations=12;
  o.initial_control_sequence=Eigen::MatrixXd::Constant(1,4,-0.6);
  o.initial_control_sequence(0,0)=-4.0;
  Eigen::VectorXd x(1); x<<1.0;
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_LE(r.control_sequence.minCoeff(),1.0);
  EXPECT_GE(r.control_sequence.minCoeff(),-1.0);
  EXPECT_LE(r.control_sequence.maxCoeff(),1.0);
  EXPECT_GE(r.control_sequence.maxCoeff(),-1.0);
  EXPECT_TRUE(std::isfinite(r.cost));
}

TEST(NonlinearMpc, WarmStartDoesNotRegressFromColdSolution) {
  auto p=makeProblem();
  Eigen::VectorXd x(1); x<<1.0;
  NonlinearMpcOptions cold_options; cold_options.horizon=8;
  cold_options.optimization_iterations=20; cold_options.gradient_step=0.12;
  NonlinearMpcResult cold;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,cold_options,cold));
  ASSERT_TRUE(cold.control_sequence.allFinite());
  ASSERT_TRUE(std::isfinite(cold.cost));

  NonlinearMpcOptions warm_options=cold_options;
  warm_options.initial_control_sequence=cold.control_sequence;
  NonlinearMpcResult warm;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,warm_options,warm));
  EXPECT_LE(warm.cost,cold.cost+1e-10);
  EXPECT_TRUE(warm.control_sequence.allFinite());
  EXPECT_TRUE(warm.predicted_states.allFinite());
}

TEST(NonlinearMpc, RecedingHorizonWarmStartTracksAcrossControlCycles) {
  auto p=makeProblem();
  constexpr std::size_t horizon=8U;
  constexpr std::size_t control_cycles=12U;
  NonlinearMpcOptions o; o.horizon=horizon; o.optimization_iterations=20;
  o.gradient_step=0.12;
  Eigen::VectorXd x(1); x<<1.0;
  NonlinearMpcResult r;
  ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
  const double initial_abs_state=std::abs(x(0));

  for(std::size_t cycle=0U;cycle<control_cycles;++cycle) {
    ASSERT_EQ(r.control.size(),1);
    ASSERT_TRUE(r.control.allFinite());
    EXPECT_GE(r.control(0),-1.0);
    EXPECT_LE(r.control(0),1.0);

    Eigen::VectorXd next;
    ASSERT_TRUE(p.dynamics(x,r.control,next));
    ASSERT_EQ(next.size(),1);
    ASSERT_TRUE(next.allFinite());
    x=next;

    if(cycle+1U<control_cycles) {
      Eigen::MatrixXd shifted;
      ASSERT_TRUE(shift_nonlinear_mpc_warm_start(r.control_sequence,horizon,shifted));
      ASSERT_EQ(shifted.rows(),1);
      ASSERT_EQ(shifted.cols(),static_cast<Eigen::Index>(horizon));
      EXPECT_TRUE(shifted.allFinite());
      EXPECT_GE(shifted.minCoeff(),-1.0);
      EXPECT_LE(shifted.maxCoeff(),1.0);
      o.initial_control_sequence=shifted;
      ASSERT_TRUE(solve_nonlinear_mpc(p,x,o,r));
      EXPECT_TRUE(r.predicted_states.allFinite());
      EXPECT_TRUE(std::isfinite(r.cost));
    }
  }

  EXPECT_LT(std::abs(x(0)),initial_abs_state);
}

TEST(NonlinearMpc, RejectsInvalidWarmStartTransactionally) {
  auto p=makeProblem();
  NonlinearMpcOptions o; o.horizon=4;
  o.initial_control_sequence=Eigen::MatrixXd::Zero(2,4);
  Eigen::VectorXd x(1); x<<0.5;
  NonlinearMpcResult r; r.control=Eigen::VectorXd::Constant(1,42.0);
  EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.control(0),42.0);
  o.initial_control_sequence=Eigen::MatrixXd::Constant(1,4,std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  EXPECT_DOUBLE_EQ(r.control(0),42.0);
}

TEST(NonlinearMpc, ShiftsWarmStartTransactionally) {
  Eigen::MatrixXd previous(1,3); previous<<0.1,0.2,0.3;
  Eigen::MatrixXd shifted;
  ASSERT_TRUE(shift_nonlinear_mpc_warm_start(previous,4,shifted));
  ASSERT_EQ(shifted.rows(),1); ASSERT_EQ(shifted.cols(),4);
  EXPECT_NEAR(shifted(0,0),0.2,1e-12);
  EXPECT_NEAR(shifted(0,1),0.3,1e-12);
  EXPECT_NEAR(shifted(0,2),0.3,1e-12);
  EXPECT_NEAR(shifted(0,3),0.3,1e-12);
  Eigen::MatrixXd sentinel=Eigen::MatrixXd::Constant(1,1,42.0);
  Eigen::MatrixXd invalid=previous; invalid(0,1)=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(shift_nonlinear_mpc_warm_start(invalid,2,sentinel));
  ASSERT_EQ(sentinel.rows(),1); ASSERT_EQ(sentinel.cols(),1);
  EXPECT_DOUBLE_EQ(sentinel(0,0),42.0);
}

TEST(NonlinearMpc, RejectsInvalidCallbacksAndOptions) {
  auto p=makeProblem(); Eigen::VectorXd x=Eigen::VectorXd::Zero(1); NonlinearMpcResult r;
  NonlinearMpcOptions o; o.horizon=0; EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  o.horizon=4; o.line_search_iterations=0; EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
  o.line_search_iterations=12; p.dynamics={}; EXPECT_FALSE(solve_nonlinear_mpc(p,x,o,r));
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
  EXPECT_NEAR(r.objective,-9.75,1e-5);
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

TEST(GenericMpc, DelegatesSolverAndValidatesContract) {
  GenericMpcProblem p;
  p.initial_state=Eigen::Vector2d(1.0,-2.0);
  p.control_lower=Eigen::Vector2d(-1.0,-0.5);
  p.control_upper=Eigen::Vector2d(1.0,0.5);
  p.horizon=3U;
  GenericMpcOptions o; o.horizon=3U;
  MpcSolverFunction solver=[](const GenericMpcProblem& problem,
                              const GenericMpcOptions&,
                              GenericMpcResult& result,
                              std::string*) {
    const Eigen::Index nx=problem.initial_state.size();
    const Eigen::Index nu=problem.control_lower.size();
    const Eigen::Index N=static_cast<Eigen::Index>(problem.horizon);
    result.control_sequence=Eigen::MatrixXd::Zero(nu,N);
    result.predicted_states=Eigen::MatrixXd::Zero(nx,N+1);
    result.predicted_states.col(0)=problem.initial_state;
    result.control=Eigen::VectorXd::Zero(nu);
    result.cost=1.25;
    return true;
  };
  GenericMpcResult r;
  ASSERT_TRUE(solve_generic_mpc(p,o,solver,r));
  EXPECT_EQ(r.control.size(),2);
  EXPECT_EQ(r.control_sequence.cols(),3);
  EXPECT_EQ(r.predicted_states.cols(),4);
  EXPECT_DOUBLE_EQ(r.cost,1.25);
}

TEST(GenericMpc, RejectsInvalidProblemTransactionally) {
  GenericMpcProblem p;
  p.initial_state=Eigen::VectorXd::Zero(1);
  p.control_lower=Eigen::VectorXd::Constant(1,1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,-1.0);
  p.horizon=2U;
  GenericMpcOptions o; o.horizon=2U;
  GenericMpcResult r; r.control=Eigen::VectorXd::Constant(1,42.0);
  const double old=r.control(0);
  MpcSolverFunction solver=[](const GenericMpcProblem&,const GenericMpcOptions&,GenericMpcResult&,std::string*){ return true; };
  EXPECT_FALSE(solve_generic_mpc(p,o,solver,r));
  EXPECT_DOUBLE_EQ(r.control(0),old);
}

TEST(GenericMpc, RejectsInvalidSolverResult) {
  GenericMpcProblem p;
  p.initial_state=Eigen::VectorXd::Zero(1);
  p.control_lower=Eigen::VectorXd::Constant(1,-1.0);
  p.control_upper=Eigen::VectorXd::Constant(1,1.0);
  p.horizon=2U;
  GenericMpcOptions o; o.horizon=2U;
  GenericMpcResult r; r.control=Eigen::VectorXd::Constant(1,42.0);
  const double old=r.control(0);
  MpcSolverFunction solver=[](const GenericMpcProblem&,const GenericMpcOptions&,GenericMpcResult& result,std::string*){
    result.control=Eigen::VectorXd::Constant(1,2.0);
    return true;
  };
  EXPECT_FALSE(solve_generic_mpc(p,o,solver,r));
  EXPECT_DOUBLE_EQ(r.control(0),old);
}
}
