#include "mca/uav/geometric_attitude_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {
using mca::geometry::Quaternion;
using mca::geometry::Vec3;
using mca::uav::GeometricAttitudeController;
using mca::uav::GeometricAttitudeControllerConfig;
using mca::uav::GeometricAttitudeControllerInput;
using mca::uav::GeometricAttitudeControllerOutput;
constexpr double kPi = 3.14159265358979323846;

GeometricAttitudeControllerConfig config() {
  GeometricAttitudeControllerConfig c;
  c.inertia_diagonal = {0.03, 0.04, 0.05};
  c.attitude_gain = {4.0, 4.0, 2.0};
  c.angular_rate_gain = {0.2, 0.2, 0.1};
  c.torque_limit = {10.0, 10.0, 5.0};
  return c;
}

TEST(GeometricAttitudeControllerTest, IdentitySetpointProducesZeroTorque) {
  GeometricAttitudeController controller;
  ASSERT_TRUE(controller.configure(config()).ok());
  GeometricAttitudeControllerOutput output;
  ASSERT_TRUE(controller.update({}, output).ok());
  EXPECT_NEAR(output.attitude_error_body.x, 0.0, 1e-12);
  EXPECT_NEAR(output.attitude_error_body.y, 0.0, 1e-12);
  EXPECT_NEAR(output.attitude_error_body.z, 0.0, 1e-12);
  EXPECT_NEAR(output.torque_body.x, 0.0, 1e-12);
  EXPECT_NEAR(output.torque_body.y, 0.0, 1e-12);
  EXPECT_NEAR(output.torque_body.z, 0.0, 1e-12);
}

TEST(GeometricAttitudeControllerTest, QuaternionErrorProducesBodyTorque) {
  GeometricAttitudeController controller;
  ASSERT_TRUE(controller.configure(config()).ok());
  GeometricAttitudeControllerInput input;
  input.desired_attitude_body_to_world =
      mca::geometry::axis_angle({1.0, 0.0, 0.0}, 0.2);
  GeometricAttitudeControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_NEAR(output.attitude_error_body.x, 2.0 * std::sin(0.1), 1e-12);
  EXPECT_NEAR(output.torque_body.x, 8.0 * std::sin(0.1), 1e-12);
  EXPECT_NEAR(output.torque_body.y, 0.0, 1e-12);
}

TEST(GeometricAttitudeControllerTest, AntipodalQuaternionChoosesSameShortestArc) {
  GeometricAttitudeController controller;
  ASSERT_TRUE(controller.configure(config()).ok());
  GeometricAttitudeControllerInput input;
  const Quaternion q = mca::geometry::axis_angle({0.0, 0.0, 1.0}, 0.4);
  input.desired_attitude_body_to_world = q;
  GeometricAttitudeControllerOutput a;
  ASSERT_TRUE(controller.update(input, a).ok());
  input.desired_attitude_body_to_world = {-q.w, -q.x, -q.y, -q.z};
  GeometricAttitudeControllerOutput b;
  ASSERT_TRUE(controller.update(input, b).ok());
  EXPECT_NEAR(a.torque_body.z, b.torque_body.z, 1e-12);
  EXPECT_NEAR(a.attitude_error_body.z, b.attitude_error_body.z, 1e-12);
}

TEST(GeometricAttitudeControllerTest, AddsRateFeedbackFeedforwardAndGyroscopicTerm) {
  GeometricAttitudeController controller;
  auto c = config();
  c.attitude_gain = {0.0, 0.0, 0.0};
  c.angular_rate_gain = {1.0, 1.0, 1.0};
  c.torque_limit = {100.0, 100.0, 100.0};
  ASSERT_TRUE(controller.configure(c).ok());
  GeometricAttitudeControllerInput input;
  input.angular_velocity_body = {1.0, 2.0, 3.0};
  input.desired_angular_velocity_body = {2.0, 4.0, 6.0};
  input.feedforward_torque_body = {0.1, 0.2, 0.3};
  GeometricAttitudeControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_NEAR(output.torque_body.x, 1.0 + 0.06 + 0.1, 1e-12);
  EXPECT_NEAR(output.torque_body.y, 2.0 - 0.06 + 0.2, 1e-12);
  EXPECT_NEAR(output.torque_body.z, 3.0 + 0.02 + 0.3, 1e-12);
}

TEST(GeometricAttitudeControllerTest, SaturatesEachTorqueAxis) {
  GeometricAttitudeController controller;
  auto c = config();
  c.attitude_gain = {100.0, 100.0, 100.0};
  c.torque_limit = {0.5, 0.6, 0.7};
  ASSERT_TRUE(controller.configure(c).ok());
  GeometricAttitudeControllerInput input;
  input.desired_attitude_body_to_world =
      mca::geometry::axis_angle({1.0, 1.0, 1.0}, 1.0);
  GeometricAttitudeControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_LE(std::abs(output.torque_body.x), 0.5);
  EXPECT_LE(std::abs(output.torque_body.y), 0.6);
  EXPECT_LE(std::abs(output.torque_body.z), 0.7);
}

TEST(GeometricAttitudeControllerTest, RejectsInvalidInputTransactionally) {
  GeometricAttitudeController controller;
  ASSERT_TRUE(controller.configure(config()).ok());
  GeometricAttitudeControllerInput input;
  input.attitude_body_to_world = {0.0, 0.0, 0.0, 0.0};
  GeometricAttitudeControllerOutput output;
  output.torque_body = {7.0, 8.0, 9.0};
  EXPECT_FALSE(controller.update(input, output).ok());
  EXPECT_DOUBLE_EQ(output.torque_body.x, 7.0);
  EXPECT_DOUBLE_EQ(output.torque_body.y, 8.0);
  EXPECT_DOUBLE_EQ(output.torque_body.z, 9.0);
  input.attitude_body_to_world = {};
  input.angular_velocity_body.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(controller.update(input, output).ok());
  EXPECT_DOUBLE_EQ(output.torque_body.x, 7.0);
}

TEST(GeometricAttitudeControllerTest, RejectsInvalidConfiguration) {
  GeometricAttitudeController controller;
  auto c = config();
  c.inertia_diagonal.y = 0.0;
  EXPECT_FALSE(controller.configure(c).ok());
  EXPECT_FALSE(controller.configured());
  c = config();
  c.torque_limit.z = -1.0;
  EXPECT_FALSE(controller.configure(c).ok());
  EXPECT_FALSE(controller.configured());
}

}  // namespace
