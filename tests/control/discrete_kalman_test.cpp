#include <limits>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "mca/control/discrete_kalman.hpp"

namespace {
using namespace mca::control;
TEST(DiscreteKalman, ScalarMeasurementMatchesAnalyticUpdate) {
  DiscreteKalmanState prior; prior.x=Eigen::VectorXd::Constant(1,0.0); prior.P=Eigen::MatrixXd::Constant(1,1,2.0);
  Eigen::MatrixXd C=Eigen::MatrixXd::Identity(1,1), R=Eigen::MatrixXd::Constant(1,1,1.0);
  Eigen::VectorXd y=Eigen::VectorXd::Constant(1,3.0); DiscreteKalmanState out;
  ASSERT_TRUE(kalman_update(C,R,y,prior,out));
  EXPECT_NEAR(out.x(0),2.0,1e-12); EXPECT_NEAR(out.P(0,0),2.0/3.0,1e-12);
}
TEST(DiscreteKalman, PredictionUsesControlAndProcessNoise) {
  Eigen::MatrixXd A=Eigen::MatrixXd::Identity(1,1), B=Eigen::MatrixXd::Identity(1,1), Q=Eigen::MatrixXd::Constant(1,1,0.25);
  Eigen::VectorXd u=Eigen::VectorXd::Constant(1,2.0);
  DiscreteKalmanState in; in.x=Eigen::VectorXd::Constant(1,1.0); in.P=Eigen::MatrixXd::Constant(1,1,2.0); DiscreteKalmanState out;
  ASSERT_TRUE(kalman_predict(A,B,u,Q,in,out)); EXPECT_DOUBLE_EQ(out.x(0),3.0); EXPECT_DOUBLE_EQ(out.P(0,0),2.25);
}
TEST(DiscreteKalman, LqgUsesEstimatedState) {
  Eigen::MatrixXd K(2,2); K<<2.0,0.0,0.0,3.0;
  Eigen::VectorXd x(2);x<<4.0,-2.0;Eigen::VectorXd u;
  ASSERT_TRUE(lqg_control(K,x,u));EXPECT_DOUBLE_EQ(u(0),-8.0);EXPECT_DOUBLE_EQ(u(1),6.0);
}
TEST(DiscreteKalman, InvalidUpdatePreservesOutput) {
  DiscreteKalmanState prior;prior.x=Eigen::VectorXd::Zero(1);prior.P=Eigen::MatrixXd::Identity(1,1);
  Eigen::MatrixXd C=Eigen::MatrixXd::Identity(1,1),R=Eigen::MatrixXd::Identity(1,1);
  Eigen::VectorXd y=Eigen::VectorXd::Zero(1);DiscreteKalmanState out;out.x=Eigen::VectorXd::Constant(1,42.0);out.P=Eigen::MatrixXd::Constant(1,1,43.0);
  R(0,0)=0.0;EXPECT_FALSE(kalman_update(C,R,y,prior,out));EXPECT_DOUBLE_EQ(out.x(0),42.0);EXPECT_DOUBLE_EQ(out.P(0,0),43.0);
  Eigen::MatrixXd A=Eigen::MatrixXd::Identity(1,1),B=Eigen::MatrixXd::Identity(1,1),Q=Eigen::MatrixXd::Identity(1,1);
  EXPECT_FALSE(kalman_predict(A,B,y,Q,prior,out,DiscreteKalmanOptions{0.0,-1.0}));EXPECT_DOUBLE_EQ(out.x(0),42.0);
}
TEST(DiscreteKalman, RejectsDimensionAndNonFiniteInputs) {
  DiscreteKalmanState prior;prior.x=Eigen::VectorXd::Zero(1);prior.P=Eigen::MatrixXd::Identity(1,1);DiscreteKalmanState out;
  Eigen::MatrixXd C=Eigen::MatrixXd::Identity(1,1),R=Eigen::MatrixXd::Identity(1,1);Eigen::VectorXd y=Eigen::VectorXd::Zero(2);
  EXPECT_FALSE(kalman_update(C,R,y,prior,out));
  y=Eigen::VectorXd::Constant(1,std::numeric_limits<double>::infinity());EXPECT_FALSE(kalman_update(C,R,y,prior,out));
}
} // namespace
