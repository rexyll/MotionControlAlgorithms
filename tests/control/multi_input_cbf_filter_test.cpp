#include <limits>
#include <gtest/gtest.h>
#include "mca/control/multi_input_cbf_filter.hpp"
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
  in.drift_derivatives = Eigen::Vector2d::Zero();
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
}
