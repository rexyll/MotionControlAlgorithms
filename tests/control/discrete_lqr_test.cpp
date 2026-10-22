#include <limits>
#include <cmath>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "mca/control/discrete_lqr.hpp"

namespace {
using mca::control::DiscreteLqrOptions;
using mca::control::DiscreteLqrResult;
using mca::control::solve_discrete_lqr;

TEST(DiscreteLqr, SolvesScalarIntegratorReference) {
  Eigen::MatrixXd A(1,1), B(1,1), Q(1,1), R(1,1);
  A << 1.0; B << 1.0; Q << 1.0; R << 1.0;
  DiscreteLqrResult result;
  ASSERT_TRUE(solve_discrete_lqr(A,B,Q,R,result));
  EXPECT_NEAR(result.K(0,0), 0.61803398875, 1e-7);
  EXPECT_GT(result.P(0,0), 0.0);
  EXPECT_GT(result.iterations, 0);
  EXPECT_LT(result.residual, 1e-8);
}
TEST(DiscreteLqr, StabilizesDoubleIntegrator) {
  Eigen::MatrixXd A(2,2), B(2,1), Q=Eigen::MatrixXd::Identity(2,2), R(1,1);
  A << 1.0, 1.0, 0.0, 1.0; B << 0.5, 1.0; R << 0.2;
  DiscreteLqrResult result;
  ASSERT_TRUE(solve_discrete_lqr(A,B,Q,R,result));
  Eigen::EigenSolver<Eigen::MatrixXd> eig(A-B*result.K);
  ASSERT_EQ(eig.info(), Eigen::Success);
  for (Eigen::Index i=0;i<eig.eigenvalues().size();++i) EXPECT_LT(std::abs(eig.eigenvalues()(i)),1.0);
}
TEST(DiscreteLqr, InvalidInputsPreserveOutput) {
  Eigen::MatrixXd A=Eigen::MatrixXd::Identity(2,2), B=Eigen::MatrixXd::Ones(2,1);
  Eigen::MatrixXd Q=Eigen::MatrixXd::Identity(2,2), R=Eigen::MatrixXd::Identity(1,1);
  DiscreteLqrResult result; result.K=Eigen::MatrixXd::Constant(1,1,42.0);
  EXPECT_FALSE(solve_discrete_lqr(A,B,Q,R,result,DiscreteLqrOptions{0,1e-10,1e-9}));
  EXPECT_DOUBLE_EQ(result.K(0,0),42.0);
  R(0,0)=0.0;
  EXPECT_FALSE(solve_discrete_lqr(A,B,Q,R,result));
  EXPECT_DOUBLE_EQ(result.K(0,0),42.0);
  Q(0,0)=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(solve_discrete_lqr(A,B,Q,R,result));
  EXPECT_DOUBLE_EQ(result.K(0,0),42.0);
}
TEST(DiscreteLqr, RejectsDimensionAndSymmetryErrors) {
  Eigen::MatrixXd A=Eigen::MatrixXd::Identity(2,2), B=Eigen::MatrixXd::Ones(2,1);
  Eigen::MatrixXd Q=Eigen::MatrixXd::Identity(2,2), R=Eigen::MatrixXd::Identity(1,1);
  DiscreteLqrResult result;
  EXPECT_FALSE(solve_discrete_lqr(A,B,Eigen::MatrixXd::Identity(1,1),R,result));
  Q(0,1)=0.5;
  EXPECT_FALSE(solve_discrete_lqr(A,B,Q,R,result));
}
TEST(DiscreteLqr, ReportsNonConvergence) {
  Eigen::MatrixXd A(1,1), B(1,1), Q(1,1), R(1,1);
  A << 1.0; B << 1.0; Q << 1.0; R << 1.0;
  DiscreteLqrResult result;
  EXPECT_FALSE(solve_discrete_lqr(A,B,Q,R,result,DiscreteLqrOptions{1,1e-15,1e-9}));
}
} // namespace
