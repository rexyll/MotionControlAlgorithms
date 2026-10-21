#include "mca/uav/mpc_position_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {
using mca::uav::MpcPositionController;
using mca::uav::MpcPositionControllerConfig;
using mca::uav::MpcPositionControllerInput;
using mca::uav::MpcPositionControllerOutput;

TEST(MpcPositionController, RejectsInvalidConfiguration) {
  MpcPositionController controller;
  auto config = MpcPositionControllerConfig{};
  config.horizon = 0U;
  EXPECT_FALSE(controller.configure(config).ok());
  config = MpcPositionControllerConfig{};
  config.horizon = MpcPositionController::kMaxHorizon + 1U;
  EXPECT_FALSE(controller.configure(config).ok());
  config = MpcPositionControllerConfig{};
  config.max_acceleration.y = 0.0;
  EXPECT_FALSE(controller.configure(config).ok());
  config = MpcPositionControllerConfig{};
  config.gradient_step = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.configure(config).ok());
}

TEST(MpcPositionController, ZeroErrorPassesFeedforwardAndHasFiniteCost) {
  MpcPositionController controller;
  ASSERT_TRUE(controller.configure(MpcPositionControllerConfig{}).ok());
  MpcPositionControllerInput input;
  input.feedforward_acceleration_world = {1.0, -2.0, 0.5};
  MpcPositionControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_NEAR(output.acceleration_world.x, 1.0, 1e-12);
  EXPECT_NEAR(output.acceleration_world.y, -2.0, 1e-12);
  EXPECT_NEAR(output.acceleration_world.z, 0.5, 1e-12);
  EXPECT_TRUE(std::isfinite(output.predicted_cost));
  EXPECT_NEAR(output.predicted_cost, 0.0, 1e-12);
}

TEST(MpcPositionController, OptimizesTowardDesiredPosition) {
  MpcPositionController controller;
  auto config = MpcPositionControllerConfig{};
  config.gradient_step = 0.005;
  ASSERT_TRUE(controller.configure(config).ok());
  MpcPositionControllerInput input;
  input.position_world = {1.0, -1.0, 0.5};
  MpcPositionControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_LT(output.acceleration_world.x, 0.0);
  EXPECT_GT(output.acceleration_world.y, 0.0);
  EXPECT_LT(output.acceleration_world.z, 0.0);
  EXPECT_GT(output.predicted_cost, 0.0);
}

TEST(MpcPositionController, AppliesIndependentAxisAccelerationLimits) {
  MpcPositionController controller;
  auto config = MpcPositionControllerConfig{};
  config.max_acceleration = {0.2, 0.4, 0.6};
  config.gradient_step = 0.001;
  ASSERT_TRUE(controller.configure(config).ok());
  MpcPositionControllerInput input;
  input.desired_position_world = {100.0, -100.0, 100.0};
  MpcPositionControllerOutput output;
  ASSERT_TRUE(controller.update(input, output).ok());
  EXPECT_NEAR(output.acceleration_world.x, 0.2, 1e-12);
  EXPECT_NEAR(output.acceleration_world.y, -0.4, 1e-12);
  EXPECT_NEAR(output.acceleration_world.z, 0.6, 1e-12);
}

TEST(MpcPositionController, InvalidInputLeavesOutputUnchanged) {
  MpcPositionController controller;
  ASSERT_TRUE(controller.configure(MpcPositionControllerConfig{}).ok());
  MpcPositionControllerInput input;
  input.velocity_world.z = std::numeric_limits<double>::infinity();
  MpcPositionControllerOutput output;
  output.acceleration_world = {7.0, 8.0, 9.0};
  output.predicted_cost = 42.0;
  EXPECT_FALSE(controller.update(input, output).ok());
  EXPECT_DOUBLE_EQ(output.acceleration_world.x, 7.0);
  EXPECT_DOUBLE_EQ(output.acceleration_world.y, 8.0);
  EXPECT_DOUBLE_EQ(output.acceleration_world.z, 9.0);
  EXPECT_DOUBLE_EQ(output.predicted_cost, 42.0);
}

TEST(MpcPositionController, RecedingHorizonDoubleIntegratorReducesPositionError) {
  MpcPositionControllerConfig config;
  config.sample_time = 0.02;
  config.horizon = 40U;
  config.optimization_iterations = 160U;
  config.gradient_step = 0.001;
  config.max_acceleration = {3.0, 3.0, 3.0};
  MpcPositionController controller;
  ASSERT_TRUE(controller.configure(config).ok());
  MpcPositionControllerInput input;
  input.position_world = {1.5, -0.8, 0.4};
  MpcPositionControllerOutput output;
  for (int step = 0; step < 600; ++step) {
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
  EXPECT_NEAR(input.position_world.x, 0.0, 0.03);
  EXPECT_NEAR(input.position_world.y, 0.0, 0.03);
  EXPECT_NEAR(input.position_world.z, 0.0, 0.03);
  EXPECT_NEAR(input.velocity_world.x, 0.0, 0.03);
  EXPECT_NEAR(input.velocity_world.y, 0.0, 0.03);
  EXPECT_NEAR(input.velocity_world.z, 0.0, 0.03);
}

}  // namespace
