#include "mca/legged/centroidal_dynamics.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::geometry::Vec3;
using mca::legged::CentroidalAppliedForce;
using mca::legged::CentroidalDynamicsInput;
using mca::legged::CentroidalDynamicsOutput;
using mca::legged::evaluate_centroidal_dynamics;

TEST(CentroidalDynamics, GravityOnlyProducesExpectedAcceleration) {
  CentroidalDynamicsInput input;
  input.total_mass = 12.0;
  CentroidalDynamicsOutput output;
  ASSERT_TRUE(evaluate_centroidal_dynamics(input, output));
  EXPECT_DOUBLE_EQ(output.net_force_world.x, 0.0);
  EXPECT_DOUBLE_EQ(output.net_force_world.y, 0.0);
  EXPECT_NEAR(output.net_force_world.z, -117.72, 1e-12);
  EXPECT_NEAR(output.center_of_mass_acceleration_world.z, -9.81, 1e-12);
  EXPECT_DOUBLE_EQ(output.linear_momentum_rate_world.z,
                   output.net_force_world.z);
  EXPECT_DOUBLE_EQ(output.angular_momentum_rate_world.x, 0.0);
  EXPECT_DOUBLE_EQ(output.angular_momentum_rate_world.y, 0.0);
  EXPECT_DOUBLE_EQ(output.angular_momentum_rate_world.z, 0.0);
}

TEST(CentroidalDynamics, AppliedForcesAndMomentsAggregateCorrectly) {
  CentroidalDynamicsInput input;
  input.total_mass = 2.0;
  input.gravity_world = {};
  input.center_of_mass_world = {0.0, 0.0, 0.0};
  input.applied_forces = {
      CentroidalAppliedForce{{0.0, 1.0, 0.0}, {10.0, 0.0, 0.0}},
      CentroidalAppliedForce{{-1.0, 0.0, 0.0}, {0.0, 4.0, 0.0}}};
  input.external_torque_world = {0.0, 0.0, 3.0};
  CentroidalDynamicsOutput output;
  ASSERT_TRUE(evaluate_centroidal_dynamics(input, output));
  EXPECT_DOUBLE_EQ(output.net_force_world.x, 10.0);
  EXPECT_DOUBLE_EQ(output.net_force_world.y, 4.0);
  EXPECT_DOUBLE_EQ(output.center_of_mass_acceleration_world.x, 5.0);
  EXPECT_DOUBLE_EQ(output.center_of_mass_acceleration_world.y, 2.0);
  // r x F gives -10 z for the first force and -4 z for the second.
  EXPECT_DOUBLE_EQ(output.angular_momentum_rate_world.x, 0.0);
  EXPECT_DOUBLE_EQ(output.angular_momentum_rate_world.y, 0.0);
  EXPECT_DOUBLE_EQ(output.angular_momentum_rate_world.z, -11.0);
}

TEST(CentroidalDynamics, TranslatingCOMDoesNotChangeForceMomentArmConvention) {
  CentroidalDynamicsInput input;
  input.gravity_world = {};
  input.center_of_mass_world = {1.0, 2.0, 3.0};
  input.applied_forces = {
      CentroidalAppliedForce{{1.0, 3.0, 3.0}, {2.0, 0.0, 0.0}}};
  CentroidalDynamicsOutput output;
  ASSERT_TRUE(evaluate_centroidal_dynamics(input, output));
  EXPECT_DOUBLE_EQ(output.angular_momentum_rate_world.z, -2.0);
}

TEST(CentroidalDynamics, InvalidInputPreservesOutput) {
  CentroidalDynamicsInput input;
  CentroidalDynamicsOutput output;
  output.net_force_world = {7.0, 8.0, 9.0};
  output.angular_momentum_rate_world = {1.0, 2.0, 3.0};
  std::string error;
  input.total_mass = 0.0;
  EXPECT_FALSE(evaluate_centroidal_dynamics(input, output, &error));
  EXPECT_EQ(error, "total mass must be finite and positive");
  EXPECT_DOUBLE_EQ(output.net_force_world.x, 7.0);
  EXPECT_DOUBLE_EQ(output.angular_momentum_rate_world.z, 3.0);
  input.total_mass = 1.0;
  input.applied_forces = {CentroidalAppliedForce{
      {}, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}};
  EXPECT_FALSE(evaluate_centroidal_dynamics(input, output, &error));
  EXPECT_DOUBLE_EQ(output.net_force_world.x, 7.0);
}

TEST(CentroidalDynamics, RejectsOverflowedComputedOutput) {
  CentroidalDynamicsInput input;
  input.total_mass = std::numeric_limits<double>::max();
  input.gravity_world = {0.0, 0.0, -2.0};
  CentroidalDynamicsOutput output;
  output.net_force_world = {4.0, 5.0, 6.0};
  EXPECT_FALSE(evaluate_centroidal_dynamics(input, output));
  EXPECT_DOUBLE_EQ(output.net_force_world.x, 4.0);
}
}  // namespace
