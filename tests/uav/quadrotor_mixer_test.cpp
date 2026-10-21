#include "mca/uav/quadrotor_mixer.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {
using mca::uav::QuadrotorMixer;
using mca::uav::QuadrotorMixerConfig;
using mca::uav::QuadrotorMixerOutput;
using mca::uav::QuadrotorWrench;

TEST(QuadrotorMixer, ConfiguresAndAllocatesPureCollectiveThrust) {
  QuadrotorMixer mixer;
  ASSERT_TRUE(mixer.configure(QuadrotorMixerConfig{}).ok());
  QuadrotorMixerOutput output;
  ASSERT_TRUE(mixer.allocate({12.0, {}}, output).ok());
  for (double thrust : output.rotor_thrust) EXPECT_NEAR(thrust, 3.0, 1e-10);
  EXPECT_NEAR(output.achieved.thrust, 12.0, 1e-10);
  EXPECT_NEAR(output.torque_scale, 1.0, 1e-10);
}

TEST(QuadrotorMixer, ReconstructsUnsaturatedWrench) {
  QuadrotorMixer mixer;
  ASSERT_TRUE(mixer.configure(QuadrotorMixerConfig{}).ok());
  QuadrotorMixerOutput output;
  QuadrotorWrench requested{16.0, {0.2, -0.12, 0.04}};
  ASSERT_TRUE(mixer.allocate(requested, output).ok());
  EXPECT_NEAR(output.achieved.thrust, requested.thrust, 1e-9);
  EXPECT_NEAR(output.achieved.torque.x, requested.torque.x, 1e-9);
  EXPECT_NEAR(output.achieved.torque.y, requested.torque.y, 1e-9);
  EXPECT_NEAR(output.achieved.torque.z, requested.torque.z, 1e-9);
  EXPECT_DOUBLE_EQ(output.torque_scale, 1.0);
}

TEST(QuadrotorMixer, DesaturatesMomentsWithoutChangingCollectiveThrust) {
  QuadrotorMixer mixer;
  auto config = QuadrotorMixerConfig{};
  config.max_rotor_thrust = 5.0;
  ASSERT_TRUE(mixer.configure(config).ok());
  QuadrotorMixerOutput output;
  ASSERT_TRUE(mixer.allocate({12.0, {3.0, 2.0, 0.5}}, output).ok());
  EXPECT_LT(output.torque_scale, 1.0);
  EXPECT_NEAR(output.achieved.thrust, 12.0, 1e-8);
  for (double thrust : output.rotor_thrust) {
    EXPECT_GE(thrust, -1e-10);
    EXPECT_LE(thrust, config.max_rotor_thrust + 1e-10);
  }
}

TEST(QuadrotorMixer, RejectsInvalidInputTransactionally) {
  QuadrotorMixer mixer;
  ASSERT_TRUE(mixer.configure(QuadrotorMixerConfig{}).ok());
  QuadrotorMixerOutput output;
  output.rotor_thrust = {1.0, 2.0, 3.0, 4.0};
  EXPECT_FALSE(mixer.allocate({-1.0, {}}, output).ok());
  EXPECT_DOUBLE_EQ(output.rotor_thrust[0], 1.0);
  EXPECT_FALSE(mixer.allocate({std::numeric_limits<double>::quiet_NaN(), {}}, output).ok());
  auto bad = QuadrotorMixerConfig{};
  bad.arm_length = 0.0;
  EXPECT_FALSE(mixer.configure(bad).ok());
}

TEST(QuadrotorMixer, RequiresConfiguration) {
  QuadrotorMixer mixer;
  QuadrotorMixerOutput output;
  EXPECT_FALSE(mixer.allocate({4.0, {}}, output).ok());
}
}  // namespace
