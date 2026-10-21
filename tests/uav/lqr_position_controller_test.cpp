#include "mca/uav/lqr_position_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {
using mca::geometry::Vec3;
using mca::uav::LqrPositionController;
using mca::uav::LqrPositionControllerConfig;
using mca::uav::LqrPositionControllerInput;
using mca::uav::LqrPositionControllerOutput;

TEST(LqrPositionController, ConfiguresFinitePositiveWeightsAndLimits) {
  LqrPositionController controller;
  ASSERT_TRUE(controller.configure(LqrPositionControllerConfig{}).ok());
  EXPECT_TRUE(controller.configured());
  EXPECT_GT(controller.position_gain().x, 0.0);
  EXPECT_GT(controller.velocity_gain().x, 0.0);

  auto config = LqrPositionControllerConfig{};
  config.position_weight.y = 0.0;
  EXPECT_FALSE(controller.configure(config).ok());
  EXPECT_FALSE(controller.configured());
  config = LqrPositionControllerConfig{};
  config.sample_time = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(controller.configure(config).ok());
}

TEST(LqrPositionController, ZeroErrorPassesFeedforwardAcceleration) {
  LqrPositionController controller;
  ASSERT_TRUE(controller.configure(LqrPositionControllerConfig{}).ok());
  LqrPositionControllerInput input;
  input.feedforward_acceleration_world = {1.0, -2.0, 0.5};
  LqrPositionControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_NEAR(output.acceleration_world.x, 1.0, 1e-12);
  EXPECT_NEAR(output.acceleration_world.y, -2.0, 1e-12);
  EXPECT_NEAR(output.acceleration_world.z, 0.5, 1e-12);
  EXPECT_NEAR(output.position_error_world.x, 0.0, 1e-12);
}

TEST(LqrPositionController, FeedbackUsesConfiguredRiccatiGains) {
  LqrPositionController controller;
  ASSERT_TRUE(controller.configure(LqrPositionControllerConfig{}).ok());
  LqrPositionControllerInput input;
  input.position_world = {1.0, -2.0, 0.5};
  input.velocity_world = {0.2, 0.3, -0.4};
  input.desired_position_world = {0.0, 0.0, 0.0};
  input.desired_velocity_world = {0.0, 0.0, 0.0};
  LqrPositionControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  const Vec3 kp = controller.position_gain();
  const Vec3 kv = controller.velocity_gain();
  EXPECT_NEAR(output.acceleration_world.x, -kp.x - 0.2 * kv.x, 1e-12);
  EXPECT_NEAR(output.acceleration_world.y, 2.0 * kp.y - 0.3 * kv.y, 1e-12);
  EXPECT_NEAR(output.acceleration_world.z, -0.5 * kp.z + 0.4 * kv.z, 1e-12);
}

TEST(LqrPositionController, SaturatesAccelerationPerAxis) {
  LqrPositionController controller;
  auto config = LqrPositionControllerConfig{};
  config.max_acceleration = {0.5, 0.75, 1.0};
  ASSERT_TRUE(controller.configure(config).ok());
  LqrPositionControllerInput input;
  input.desired_position_world = {100.0, -100.0, 100.0};
  LqrPositionControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_NEAR(output.acceleration_world.x, 0.5, 1e-12);
  EXPECT_NEAR(output.acceleration_world.y, -0.75, 1e-12);
  EXPECT_NEAR(output.acceleration_world.z, 1.0, 1e-12);
}

TEST(LqrPositionController, InvalidInputLeavesOutputUnchanged) {
  LqrPositionController controller;
  ASSERT_TRUE(controller.configure(LqrPositionControllerConfig{}).ok());
  LqrPositionControllerInput input;
  input.velocity_world.x = std::numeric_limits<double>::quiet_NaN();
  LqrPositionControllerOutput output;
  output.acceleration_world = {7.0, 8.0, 9.0};
  EXPECT_FALSE(controller.update(input, output).ok());
  EXPECT_DOUBLE_EQ(output.acceleration_world.x, 7.0);
  EXPECT_DOUBLE_EQ(output.acceleration_world.y, 8.0);
  EXPECT_DOUBLE_EQ(output.acceleration_world.z, 9.0);
}

TEST(LqrPositionController, ClosedLoopDoubleIntegratorConverges) {
  LqrPositionControllerConfig config;
  config.sample_time = 0.01;
  config.max_acceleration = {5.0, 5.0, 5.0};
  LqrPositionController controller;
  ASSERT_TRUE(controller.configure(config).ok());
  LqrPositionControllerInput input;
  input.position_world = {2.0, -1.0, 0.5};
  input.velocity_world = {-0.5, 0.25, 0.0};
  LqrPositionControllerOutput output;
  for (int step = 0; step < 2000; ++step) {
    ASSERT_TRUE(controller.update(input, output).ok());
    const double dt = config.sample_time;
    input.position_world.x += input.velocity_world.x * dt +
                              0.5 * output.acceleration_world.x * dt * dt;
    input.position_world.y += input.velocity_world.y * dt +
                              0.5 * output.acceleration_world.y * dt * dt;
    input.position_world.z += input.velocity_world.z * dt +
                              0.5 * output.acceleration_world.z * dt * dt;
    input.velocity_world.x += output.acceleration_world.x * dt;
    input.velocity_world.y += output.acceleration_world.y * dt;
    input.velocity_world.z += output.acceleration_world.z * dt;
  }
  EXPECT_NEAR(input.position_world.x, 0.0, 1e-3);
  EXPECT_NEAR(input.position_world.y, 0.0, 1e-3);
  EXPECT_NEAR(input.position_world.z, 0.0, 1e-3);
  EXPECT_NEAR(input.velocity_world.x, 0.0, 1e-3);
  EXPECT_NEAR(input.velocity_world.y, 0.0, 1e-3);
  EXPECT_NEAR(input.velocity_world.z, 0.0, 1e-3);
}

}  // namespace
