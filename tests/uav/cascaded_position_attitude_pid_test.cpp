#include "mca/uav/cascaded_position_attitude_pid.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {
using mca::geometry::Quaternion;
using mca::geometry::Vec3;
using mca::uav::CascadedPositionAttitudePid;
using mca::uav::CascadedPositionAttitudePidConfig;
using mca::uav::CascadedPositionAttitudePidInput;
using mca::uav::CascadedPositionAttitudePidOutput;

CascadedPositionAttitudePidConfig config() {
  CascadedPositionAttitudePidConfig c;
  c.mass = 2.0;
  for (auto& pid : c.position_pid) {
    pid.kp = 2.0;
    pid.output_min = -5.0;
    pid.output_max = 5.0;
  }
  for (auto& pid : c.attitude_pid) {
    pid.kp = 3.0;
    pid.output_min = -2.0;
    pid.output_max = 2.0;
  }
  return c;
}

TEST(CascadedPositionAttitudePidTest, HoverProducesGravityCompensatingForce) {
  CascadedPositionAttitudePid controller;
  ASSERT_TRUE(controller.configure(config()).ok());
  CascadedPositionAttitudePidOutput output;
  ASSERT_TRUE(controller.update({}, output).ok());
  EXPECT_NEAR(output.force_body.x, 0.0, 1e-12);
  EXPECT_NEAR(output.force_body.y, 0.0, 1e-12);
  EXPECT_NEAR(output.force_body.z, 19.62, 1e-12);
  EXPECT_NEAR(output.torque_body.x, 0.0, 1e-12);
  EXPECT_NEAR(output.torque_body.y, 0.0, 1e-12);
  EXPECT_NEAR(output.torque_body.z, 0.0, 1e-12);
}

TEST(CascadedPositionAttitudePidTest, PositionAndAttitudeErrorsProduceCommands) {
  CascadedPositionAttitudePid controller;
  ASSERT_TRUE(controller.configure(config()).ok());
  CascadedPositionAttitudePidInput input;
  input.position_setpoint_world = {1.0, -0.5, 0.25};
  input.attitude_setpoint_body_to_world = mca::geometry::axis_angle({1.0, 0.0, 0.0}, 0.1);
  CascadedPositionAttitudePidOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_NEAR(output.desired_acceleration_world.x, 2.0, 1e-12);
  EXPECT_NEAR(output.desired_acceleration_world.y, -1.0, 1e-12);
  EXPECT_NEAR(output.desired_acceleration_world.z, 0.5, 1e-12);
  EXPECT_NEAR(output.force_body.x, 4.0, 1e-12);
  EXPECT_NEAR(output.force_body.y, -2.0, 1e-12);
  EXPECT_NEAR(output.force_body.z, 20.62, 1e-12);
  EXPECT_NEAR(output.torque_body.x, 0.3, 1e-12);
}

TEST(CascadedPositionAttitudePidTest, WrapsYawErrorAcrossPiBoundary) {
  CascadedPositionAttitudePid controller;
  auto c = config();
  c.attitude_pid[2].kp = 2.0;
  ASSERT_TRUE(controller.configure(c).ok());
  CascadedPositionAttitudePidInput input;
  input.attitude_body_to_world = mca::geometry::axis_angle(
      {0.0, 0.0, 1.0}, 3.14159265358979323846 - 0.1);
  input.attitude_setpoint_body_to_world = mca::geometry::axis_angle(
      {0.0, 0.0, 1.0}, -3.14159265358979323846 + 0.1);
  CascadedPositionAttitudePidOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_NEAR(output.torque_body.z, 0.4, 1e-9);
}

TEST(CascadedPositionAttitudePidTest, RejectsInvalidInputWithoutChangingOutput) {
  CascadedPositionAttitudePid controller;
  ASSERT_TRUE(controller.configure(config()).ok());
  CascadedPositionAttitudePidInput input;
  input.dt = 0.0;
  CascadedPositionAttitudePidOutput output;
  output.force_body = {7.0, 8.0, 9.0};
  EXPECT_FALSE(controller.update(input, output).ok());
  EXPECT_DOUBLE_EQ(output.force_body.x, 7.0);
  EXPECT_DOUBLE_EQ(output.force_body.y, 8.0);
  EXPECT_DOUBLE_EQ(output.force_body.z, 9.0);

  input.dt = 0.001;
  input.attitude_body_to_world = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(controller.update(input, output).ok());
  EXPECT_DOUBLE_EQ(output.force_body.x, 7.0);
}

TEST(CascadedPositionAttitudePidTest, RejectsInvalidConfiguration) {
  CascadedPositionAttitudePid controller;
  auto c = config();
  c.mass = 0.0;
  EXPECT_FALSE(controller.configure(c).ok());
  EXPECT_FALSE(controller.configured());
  c = config();
  c.position_pid[1].output_min = 1.0;
  c.position_pid[1].output_max = -1.0;
  EXPECT_FALSE(controller.configure(c).ok());
  EXPECT_FALSE(controller.configured());
}

}  // namespace
