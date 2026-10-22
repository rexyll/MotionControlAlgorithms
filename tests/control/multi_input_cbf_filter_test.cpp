#include <limits>
#include <gtest/gtest.h>
#include "mca/control/multi_input_cbf_filter.hpp"
#include "mca/control/relative_degree_two_cbf_filter.hpp"
namespace {
mca::control::MultiInputCbfFilter make_filter() {
  mca::control::MultiInputCbfFilter filter;
  mca::control::MultiInputCbfConfig cfg;
  cfg.control_min = Eigen::Vector2d(-2.0, -2.0);
  cfg.control_max = Eigen::Vector2d(2.0, 2.0);
  EXPECT_TRUE(filter.configure(cfg).ok());
  return filter;
}
TEST(MultiInputCbfFilter, ProjectsOntoMultipleHalfSpaces) {
  auto f = make_filter();
  mca::control::MultiInputCbfInput in;
  in.nominal_control = Eigen::Vector2d(-1.0, -1.0);
  in.barrier_values = Eigen::Vector2d::Zero();
  in.drift_derivatives = Eigen::Vector2d::Zero();
  in.control_derivatives.resize(2, 2);
  in.control_derivatives << 1, 0, 0, 1;
  mca::control::MultiInputCbfOutput out;
  ASSERT_TRUE(f.filter(in, out).ok());
  EXPECT_NEAR(out.control.x(), 0.0, 1e-9);
  EXPECT_NEAR(out.control.y(), 0.0, 1e-9);
  EXPECT_TRUE(out.modified);
  EXPECT_GE(out.minimum_safety_margin, -1e-8);
}
TEST(MultiInputCbfFilter, LeavesSafeNominalUnchanged) {
  auto f = make_filter();
  mca::control::MultiInputCbfInput in;
  in.nominal_control = Eigen::Vector2d(0.5, 0.25);
  in.barrier_values = Eigen::VectorXd::Constant(1, 1.0);
  in.drift_derivatives = Eigen::VectorXd::Zero(1);
  in.control_derivatives = Eigen::MatrixXd::Ones(1, 2);
  mca::control::MultiInputCbfOutput out;
  ASSERT_TRUE(f.filter(in, out).ok());
  EXPECT_TRUE(out.control.isApprox(in.nominal_control));
  EXPECT_FALSE(out.modified);
}
TEST(MultiInputCbfFilter, EnforcesActuatorBounds) {
  auto f = make_filter();
  mca::control::MultiInputCbfInput in;
  in.nominal_control = Eigen::Vector2d(9.0, -8.0);
  in.barrier_values = Eigen::VectorXd::Ones(1);
  in.drift_derivatives = Eigen::VectorXd::Zero(1);
  in.control_derivatives = Eigen::MatrixXd::Ones(1, 2);
  mca::control::MultiInputCbfOutput out;
  ASSERT_TRUE(f.filter(in, out).ok());
  EXPECT_LE(out.control.x(), 2.0);
  EXPECT_GE(out.control.y(), -2.0);
  EXPECT_TRUE(out.modified);
}
TEST(MultiInputCbfFilter, InfeasibleConstraintsPreserveOutput) {
  auto f = make_filter();
  mca::control::MultiInputCbfInput in;
  in.nominal_control = Eigen::Vector2d::Zero();
  in.barrier_values = Eigen::Vector2d::Constant(-1.0);
  in.drift_derivatives = Eigen::Vector2d::Zero(2);
  in.control_derivatives.resize(2, 2);
  in.control_derivatives << 1, 0, -1, 0;
  mca::control::MultiInputCbfOutput out;
  out.control = Eigen::Vector2d(77.0, 88.0);
  EXPECT_EQ(f.filter(in, out).code, mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(out.control.x(), 77.0);
  EXPECT_DOUBLE_EQ(out.control.y(), 88.0);
}
TEST(MultiInputCbfFilter, RejectsInvalidInputTransactionally) {
  auto f = make_filter();
  mca::control::MultiInputCbfInput in;
  in.nominal_control = Eigen::Vector2d::Zero();
  in.barrier_values = Eigen::VectorXd::Ones(1);
  in.drift_derivatives = Eigen::VectorXd::Zero(1);
  in.control_derivatives = Eigen::MatrixXd::Ones(1, 1);
  mca::control::MultiInputCbfOutput out;
  out.control = Eigen::Vector2d(4.0, 5.0);
  EXPECT_EQ(f.filter(in, out).code, mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(out.control.x(), 4.0);
  in.control_derivatives = Eigen::MatrixXd::Constant(1, 2,
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(f.filter(in, out).code, mca::core::StatusCode::InvalidArgument);
}
TEST(MultiInputCbfFilter, RejectsExcessiveConstraintCount) {
  mca::control::MultiInputCbfFilter f;
  mca::control::MultiInputCbfConfig cfg;
  cfg.control_min = Eigen::Vector2d(-1.0, -1.0);
  cfg.control_max = Eigen::Vector2d(1.0, 1.0);
  cfg.max_total_constraints = 3U;
  ASSERT_TRUE(f.configure(cfg).ok());
  mca::control::MultiInputCbfInput in;
  in.nominal_control = Eigen::Vector2d::Zero();
  in.barrier_values = Eigen::Vector2d::Ones(2);
  in.drift_derivatives = Eigen::VectorXd::Zero(2);
  in.control_derivatives = Eigen::MatrixXd::Zero(2, 2);
  mca::control::MultiInputCbfOutput out;
  EXPECT_EQ(f.filter(in, out).code, mca::core::StatusCode::InvalidArgument);
}

mca::control::RelativeDegreeTwoCbfFilter make_rd2_filter() {
  mca::control::RelativeDegreeTwoCbfFilter filter;
  mca::control::RelativeDegreeTwoCbfConfig cfg;
  cfg.alpha1 = 2.0;
  cfg.alpha2 = 3.0;
  cfg.control_min = Eigen::Vector2d(-2.0, -2.0);
  cfg.control_max = Eigen::Vector2d(2.0, 2.0);
  EXPECT_TRUE(filter.configure(cfg).ok());
  return filter;
}
TEST(RelativeDegreeTwoCbfFilter, EnforcesCascadedBarrierCondition) {
  auto f = make_rd2_filter();
  mca::control::RelativeDegreeTwoCbfInput in;
  in.barrier_values = Eigen::VectorXd::Constant(1, -0.2);
  in.barrier_derivatives = Eigen::VectorXd::Zero(1);
  in.second_drift_derivatives = Eigen::VectorXd::Zero(1);
  in.control_derivatives = Eigen::MatrixXd::Zero(1, 2);
  in.control_derivatives(0, 0) = 1.0;
  in.nominal_control = Eigen::Vector2d(-2.0, 0.0);
  mca::control::RelativeDegreeTwoCbfOutput out;
  ASSERT_TRUE(f.filter(in, out).ok());
  EXPECT_NEAR(out.control.x(), 1.2, 1e-9);
  EXPECT_GE(out.minimum_safety_margin, -1e-8);
}
TEST(RelativeDegreeTwoCbfFilter, LeavesSafeNominalUnchanged) {
  auto f = make_rd2_filter();
  mca::control::RelativeDegreeTwoCbfInput in;
  in.barrier_values = Eigen::VectorXd::Constant(1, 1.0);
  in.barrier_derivatives = Eigen::VectorXd::Constant(1, 0.5);
  in.second_drift_derivatives = Eigen::VectorXd::Zero(1);
  in.control_derivatives = Eigen::MatrixXd::Zero(1, 2);
  in.nominal_control = Eigen::Vector2d(0.25, -0.1);
  mca::control::RelativeDegreeTwoCbfOutput out;
  ASSERT_TRUE(f.filter(in, out).ok());
  EXPECT_TRUE(out.control.isApprox(in.nominal_control));
  EXPECT_FALSE(out.modified);
}
TEST(RelativeDegreeTwoCbfFilter, ReportsInfeasibleConstraintTransactionally) {
  auto f = make_rd2_filter();
  mca::control::RelativeDegreeTwoCbfInput in;
  in.barrier_values = Eigen::VectorXd::Constant(1, -10.0);
  in.barrier_derivatives = Eigen::VectorXd::Zero(1);
  in.second_drift_derivatives = Eigen::VectorXd::Zero(1);
  in.control_derivatives = Eigen::MatrixXd::Zero(1, 2);
  in.nominal_control = Eigen::Vector2d::Zero();
  mca::control::RelativeDegreeTwoCbfOutput out;
  out.control = Eigen::Vector2d(7.0, 8.0);
  EXPECT_EQ(f.filter(in, out).code, mca::core::StatusCode::ConstraintViolation);
  EXPECT_DOUBLE_EQ(out.control.x(), 7.0);
  EXPECT_DOUBLE_EQ(out.control.y(), 8.0);
}
TEST(RelativeDegreeTwoCbfFilter, RejectsDimensionAndFiniteValueErrors) {
  auto f = make_rd2_filter();
  mca::control::RelativeDegreeTwoCbfInput in;
  in.barrier_values = Eigen::VectorXd::Ones(2);
  in.barrier_derivatives = Eigen::VectorXd::Ones(1);
  in.second_drift_derivatives = Eigen::VectorXd::Ones(2);
  in.control_derivatives = Eigen::MatrixXd::Zero(2, 2);
  in.nominal_control = Eigen::Vector2d::Zero();
  mca::control::RelativeDegreeTwoCbfOutput out;
  EXPECT_EQ(f.filter(in, out).code, mca::core::StatusCode::InvalidArgument);
  in.barrier_derivatives = Eigen::VectorXd::Ones(2);
  in.second_drift_derivatives(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(f.filter(in, out).code, mca::core::StatusCode::InvalidArgument);
}
}
