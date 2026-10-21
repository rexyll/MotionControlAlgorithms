#include "mca/legged/floating_base_state.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::legged::FloatingBaseState;
using mca::legged::validate_floating_base_state;

TEST(FloatingBaseStateTest, AcceptsDefaultAndJointState) {
  FloatingBaseState state;
  EXPECT_TRUE(validate_floating_base_state(state));
  state.joint_positions = {0.1, -0.2};
  state.joint_velocities = {0.3, 0.4};
  EXPECT_TRUE(validate_floating_base_state(state));
}

TEST(FloatingBaseStateTest, RejectsMismatchedJointDimensions) {
  FloatingBaseState state;
  state.joint_positions = {0.1};
  std::string error;
  EXPECT_FALSE(validate_floating_base_state(state, &error));
  EXPECT_EQ(error, "joint position and velocity sizes must match");
}

TEST(FloatingBaseStateTest, RejectsZeroQuaternion) {
  FloatingBaseState state;
  state.base_orientation_body_to_world = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(validate_floating_base_state(state));
}

TEST(FloatingBaseStateTest, RejectsNonFiniteBaseAndJointValues) {
  FloatingBaseState state;
  state.base_linear_velocity_world.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validate_floating_base_state(state));
  state = {};
  state.joint_positions = {std::numeric_limits<double>::quiet_NaN()};
  state.joint_velocities = {0.0};
  EXPECT_FALSE(validate_floating_base_state(state));
}

TEST(FloatingBaseStateTest, AcceptsNonUnitQuaternionWithoutMutatingState) {
  FloatingBaseState state;
  state.base_orientation_body_to_world = {2.0, 0.0, 0.0, 0.0};
  EXPECT_TRUE(validate_floating_base_state(state));
  EXPECT_DOUBLE_EQ(state.base_orientation_body_to_world.w, 2.0);
}
}  // namespace
