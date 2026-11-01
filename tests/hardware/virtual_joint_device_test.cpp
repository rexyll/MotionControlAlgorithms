#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/hardware/virtual_joint_device.hpp"

TEST(VirtualJointDevice, LifecycleAndDefaultProfileAreUsable) {
  mca::hardware::VirtualJointDevice<2> device;
  EXPECT_EQ(device.lifecycle_state(),
            mca::hardware::HardwareLifecycleState::Uninitialized);
  EXPECT_EQ(device.activate().code, mca::core::StatusCode::NotConfigured);
  ASSERT_TRUE(device.initialize().ok());
  ASSERT_TRUE(device.activate().ok());

  mca::hardware::JointState<double, 2> state;
  ASSERT_TRUE(device.read_state(state).ok());
  EXPECT_EQ(state.valid_fields,
            state.kPositionValid | state.kVelocityValid |
                state.kAccelerationValid | state.kEffortValid);
  EXPECT_DOUBLE_EQ(state.position[0], 0.0);
  EXPECT_DOUBLE_EQ(state.position[1], 0.0);
  EXPECT_EQ(state.sequence, 1U);
}

TEST(VirtualJointDevice, IntegratesEffortAndPublishesCoherentSamples) {
  mca::hardware::VirtualJointDevice<2> device;
  ASSERT_TRUE(device.initialize().ok());
  ASSERT_TRUE(device.activate().ok());

  mca::hardware::JointEffortCommand<double, 2> command;
  command.effort = {1.0, -0.5};
  command.sequence = 1;
  ASSERT_TRUE(device.write_effort(command).ok());
  ASSERT_TRUE(device.step(0.001).ok());

  mca::hardware::JointState<double, 2> state;
  ASSERT_TRUE(device.read_state(state).ok());
  EXPECT_GT(state.position[0], 0.0);
  EXPECT_LT(state.position[1], 0.0);
  EXPECT_GT(state.velocity[0], 0.0);
  EXPECT_LT(state.velocity[1], 0.0);
  EXPECT_DOUBLE_EQ(state.effort[0], 1.0);
  EXPECT_DOUBLE_EQ(state.effort[1], -0.5);
  EXPECT_EQ(state.sequence, 1U);
  ASSERT_TRUE(device.read_state(state).ok());
  EXPECT_EQ(state.sequence, 2U);
}

TEST(VirtualJointDevice, RejectsInvalidCommandsAndOutOfProfileEffortAtomically) {
  mca::hardware::VirtualJointDevice<2> device;
  ASSERT_TRUE(device.initialize().ok());
  ASSERT_TRUE(device.activate().ok());

  mca::hardware::JointEffortCommand<double, 2> command;
  command.effort = {1.0, 2.0};
  ASSERT_TRUE(device.write_effort(command).ok());
  command.effort = {1.0, 6.0};
  EXPECT_EQ(device.write_effort(command).code,
            mca::core::StatusCode::ConstraintViolation);
  command.effort = {std::numeric_limits<double>::quiet_NaN(), 0.0};
  EXPECT_EQ(device.write_effort(command).code,
            mca::core::StatusCode::InvalidArgument);

  mca::hardware::JointState<double, 2> state;
  ASSERT_TRUE(device.read_state(state).ok());
  EXPECT_DOUBLE_EQ(state.effort[0], 1.0);
  EXPECT_DOUBLE_EQ(state.effort[1], 2.0);
}

TEST(VirtualJointDevice, EnforcesStepBoundsAndSoftwareSafeStop) {
  mca::hardware::VirtualJointDevice<1> device;
  ASSERT_TRUE(device.initialize().ok());
  ASSERT_TRUE(device.activate().ok());

  mca::hardware::JointEffortCommand<double, 1> command;
  command.effort = {1.0};
  ASSERT_TRUE(device.write_effort(command).ok());
  EXPECT_EQ(device.step(0.02).code, mca::core::StatusCode::InvalidArgument);
  ASSERT_TRUE(device.step(0.001).ok());
  ASSERT_TRUE(device.request_safe_stop().ok());
  mca::hardware::JointState<double, 1> state;
  ASSERT_TRUE(device.read_state(state).ok());
  EXPECT_DOUBLE_EQ(state.effort[0], 0.0);
  ASSERT_TRUE(device.deactivate().ok());
  EXPECT_EQ(device.step(0.001).code, mca::core::StatusCode::NotConfigured);
}

TEST(VirtualJointDevice, InvalidConfigurationDoesNotInitialize) {
  mca::hardware::VirtualJointDeviceConfig<2> config;
  config.inertia_kg_m2[1] = 0.0;
  mca::hardware::VirtualJointDevice<2> device(config);
  EXPECT_EQ(device.initialize().code, mca::core::StatusCode::InvalidArgument);
  EXPECT_EQ(device.lifecycle_state(),
            mca::hardware::HardwareLifecycleState::Uninitialized);
}
