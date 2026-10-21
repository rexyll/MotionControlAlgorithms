#include <cmath>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "mca/control/cartesian_admittance.hpp"

namespace {

mca::geometry::Pose3D pose(double x = 0.0, double y = 0.0, double z = 0.0) {
  mca::geometry::Pose3D p;
  p.position = {x, y, z};
  p.rotation = {1.0, 0.0, 0.0, 0.0};
  return p;
}

Eigen::VectorXd ones(double value) {
  return Eigen::VectorXd::Constant(6, value);
}

}  // namespace

TEST(CartesianAdmittanceTest, ExternalForceProducesExpectedVelocityAndPose) {
  mca::control::CartesianAdmittance controller;
  ASSERT_TRUE(controller.configure(ones(2.0), ones(0.0), ones(0.0), 0.1));
  ASSERT_TRUE(controller.reset(pose()));

  Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
  force[0] = 4.0;
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  mca::geometry::Pose3D out_pose;
  Eigen::VectorXd out_twist, out_acceleration;
  ASSERT_TRUE(controller.update(pose(), zero, force, zero, out_pose, out_twist, out_acceleration));

  EXPECT_DOUBLE_EQ(out_acceleration[0], 2.0);
  EXPECT_DOUBLE_EQ(out_twist[0], 0.2);
  EXPECT_DOUBLE_EQ(out_pose.position.x, 0.02);
}

TEST(CartesianAdmittanceTest, DampingOpposesVelocity) {
  mca::control::CartesianAdmittance controller;
  ASSERT_TRUE(controller.configure(ones(1.0), ones(2.0), ones(0.0), 0.1));
  Eigen::VectorXd initial_twist = Eigen::VectorXd::Zero(6);
  initial_twist[1] = 1.0;
  ASSERT_TRUE(controller.reset(pose(), initial_twist));

  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  mca::geometry::Pose3D out_pose;
  Eigen::VectorXd out_twist, out_acceleration;
  ASSERT_TRUE(controller.update(pose(), zero, zero, zero, out_pose, out_twist, out_acceleration));

  EXPECT_DOUBLE_EQ(out_acceleration[1], -2.0);
  EXPECT_DOUBLE_EQ(out_twist[1], 0.8);
}

TEST(CartesianAdmittanceTest, StiffnessReturnsTowardEquilibrium) {
  mca::control::CartesianAdmittance controller;
  ASSERT_TRUE(controller.configure(ones(1.0), ones(0.0), ones(4.0), 0.1));
  ASSERT_TRUE(controller.reset(pose()));

  const auto desired = pose(1.0);
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  mca::geometry::Pose3D out_pose;
  Eigen::VectorXd out_twist, out_acceleration;
  ASSERT_TRUE(controller.update(desired, zero, zero, zero, out_pose, out_twist, out_acceleration));

  EXPECT_DOUBLE_EQ(out_acceleration[0], 4.0);
  EXPECT_DOUBLE_EQ(out_twist[0], 0.4);
  EXPECT_DOUBLE_EQ(out_pose.position.x, 0.04);
}

TEST(CartesianAdmittanceTest, OrientationUsesExponentialMap) {
  mca::control::CartesianAdmittance controller;
  ASSERT_TRUE(controller.configure(ones(1.0), ones(0.0), ones(1.0), 0.1));
  ASSERT_TRUE(controller.reset(pose()));

  const auto desired = [&] {
    auto p = pose();
    p.rotation = mca::geometry::axis_angle({0.0, 0.0, 1.0}, 0.5);
    return p;
  }();
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  mca::geometry::Pose3D out_pose;
  Eigen::VectorXd out_twist, out_acceleration;
  ASSERT_TRUE(controller.update(desired, zero, zero, zero, out_pose, out_twist, out_acceleration));

  EXPECT_NEAR(out_acceleration[5], 0.5, 1e-12);
  EXPECT_NEAR(out_twist[5], 0.05, 1e-12);
  EXPECT_NEAR(mca::geometry::log_so3(out_pose.rotation).z, 0.005, 1e-12);
}

TEST(CartesianAdmittanceTest, OrientationErrorRemainsInBaseFrameForRotatedCurrentPose) {
  mca::control::CartesianAdmittance controller;
  ASSERT_TRUE(controller.configure(ones(1.0), ones(0.0), ones(1.0), 0.01));

  auto current = pose();
  current.rotation = mca::geometry::axis_angle({0.0, 0.0, 1.0}, 0.5 * std::acos(-1.0));
  ASSERT_TRUE(controller.reset(current));

  const auto base_error = mca::geometry::axis_angle({1.0, 0.0, 0.0}, 0.4);
  auto desired = current;
  desired.rotation = mca::geometry::normalized(
      mca::geometry::multiply(base_error, current.rotation));

  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  mca::geometry::Pose3D out_pose;
  Eigen::VectorXd out_twist, out_acceleration;
  ASSERT_TRUE(controller.update(desired, zero, zero, zero, out_pose, out_twist, out_acceleration));

  EXPECT_NEAR(out_acceleration[3], 0.4, 1e-12);
  EXPECT_NEAR(out_acceleration[4], 0.0, 1e-12);
  EXPECT_NEAR(out_acceleration[5], 0.0, 1e-12);
}

TEST(CartesianAdmittanceTest, LimitsAreAppliedBeforeStateCommit) {
  mca::control::CartesianAdmittance controller;
  ASSERT_TRUE(controller.configure(ones(1.0), ones(0.0), ones(0.0), 0.1,
                                   ones(1.0), ones(0.2), ones(0.5)));
  ASSERT_TRUE(controller.reset(pose()));

  Eigen::VectorXd force = Eigen::VectorXd::Zero(6);
  force[0] = 10.0;
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  mca::geometry::Pose3D out_pose;
  Eigen::VectorXd out_twist, out_acceleration;
  ASSERT_TRUE(controller.update(pose(), zero, force, zero, out_pose, out_twist, out_acceleration));

  EXPECT_DOUBLE_EQ(out_acceleration[0], 0.5);
  EXPECT_DOUBLE_EQ(out_twist[0], 0.05);
  EXPECT_DOUBLE_EQ(out_pose.position.x, 0.005);
}

TEST(CartesianAdmittanceTest, ResetAndInvalidUpdatePreserveState) {
  mca::control::CartesianAdmittance controller;
  ASSERT_TRUE(controller.configure(ones(1.0), ones(1.0), ones(1.0), 0.1));
  ASSERT_TRUE(controller.reset(pose(0.2)));

  const auto before_pose = controller.commanded_pose();
  const auto before_twist = controller.commanded_twist();
  Eigen::VectorXd bad = Eigen::VectorXd::Zero(5);
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  mca::geometry::Pose3D out_pose;
  Eigen::VectorXd out_twist, out_acceleration;
  EXPECT_FALSE(controller.update(pose(), bad, zero, zero, out_pose, out_twist, out_acceleration));
  EXPECT_DOUBLE_EQ(controller.commanded_pose().position.x, before_pose.position.x);
  EXPECT_TRUE(controller.commanded_twist().isApprox(before_twist));

  ASSERT_TRUE(controller.reset(pose(0.7)));
  EXPECT_DOUBLE_EQ(controller.commanded_pose().position.x, 0.7);
  EXPECT_TRUE(controller.commanded_twist().isApprox(zero));
}

TEST(CartesianAdmittanceTest, ResetRejectsOverflowingQuaternionWithoutChangingState) {
  mca::control::CartesianAdmittance controller;
  ASSERT_TRUE(controller.configure(ones(1.0), ones(0.0), ones(0.0), 0.01));
  ASSERT_TRUE(controller.reset(pose(0.25)));

  auto invalid_pose = pose(9.0);
  invalid_pose.rotation = {1e308, 1e308, 0.0, 0.0};
  EXPECT_FALSE(controller.reset(invalid_pose));
  EXPECT_DOUBLE_EQ(controller.commanded_pose().position.x, 0.25);
}

TEST(CartesianAdmittanceTest, InvalidConfigurationIsRejected) {
  mca::control::CartesianAdmittance controller;
  EXPECT_FALSE(controller.configure(ones(0.0), ones(1.0), ones(1.0), 0.01));
  EXPECT_FALSE(controller.configure(ones(1.0), ones(1.0), ones(1.0), 0.0));
}
