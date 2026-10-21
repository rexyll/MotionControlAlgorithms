#include "mca/legged/contact_model.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::geometry::Vec3;
using mca::legged::ContactModelParameters;
using mca::legged::ContactPointState;
using mca::legged::ContactWrench;
using mca::legged::evaluate_contact;

TEST(ContactModelTest, SeparatedPointHasNoContactForce) {
  ContactWrench output;
  ContactPointState state;
  state.gap = 0.01;
  state.velocity_world = {1.0, 2.0, 0.0};
  ASSERT_TRUE(evaluate_contact({}, state, output));
  EXPECT_FALSE(output.active);
  EXPECT_DOUBLE_EQ(output.normal_force, 0.0);
  EXPECT_DOUBLE_EQ(output.force_world.x, 0.0);
  EXPECT_DOUBLE_EQ(output.force_world.y, 0.0);
}

TEST(ContactModelTest, PenetrationProducesUnilateralNormalForce) {
  ContactModelParameters p;
  p.normal_stiffness = 1000.0;
  p.normal_damping = 0.0;
  ContactPointState state;
  state.gap = -0.01;
  ContactWrench output;
  ASSERT_TRUE(evaluate_contact(p, state, output));
  EXPECT_TRUE(output.active);
  EXPECT_NEAR(output.normal_force, 10.0, 1e-12);
  EXPECT_NEAR(output.force_world.z, 10.0, 1e-12);
}

TEST(ContactModelTest, DampingOpposesApproachAndNeverPulls) {
  ContactModelParameters p;
  p.normal_stiffness = 0.0;
  p.normal_damping = 20.0;
  ContactPointState state;
  state.gap = 0.0;
  state.velocity_world.z = -0.5;
  ContactWrench output;
  ASSERT_TRUE(evaluate_contact(p, state, output));
  EXPECT_NEAR(output.normal_force, 10.0, 1e-12);
  state.velocity_world.z = 0.5;
  ASSERT_TRUE(evaluate_contact(p, state, output));
  EXPECT_DOUBLE_EQ(output.normal_force, 0.0);
}

TEST(ContactModelTest, FrictionOpposesTangentialMotionAndRespectsCoulombLimit) {
  ContactModelParameters p;
  p.normal_stiffness = 1000.0;
  p.normal_damping = 0.0;
  p.friction_coefficient = 0.5;
  p.tangential_damping = 1000.0;
  ContactPointState state;
  state.gap = -0.01;  // normal force = 10 N, friction cap = 5 N
  state.velocity_world = {2.0, 0.0, 0.0};
  ContactWrench output;
  ASSERT_TRUE(evaluate_contact(p, state, output));
  EXPECT_NEAR(output.force_world.x, -5.0, 1e-12);
  EXPECT_NEAR(output.force_world.z, 10.0, 1e-12);
  state.velocity_world.x = 0.001;
  ASSERT_TRUE(evaluate_contact(p, state, output));
  EXPECT_NEAR(output.force_world.x, -1.0, 1e-12);
}

TEST(ContactModelTest, NormalIsNormalized) {
  ContactPointState state;
  state.gap = -0.01;
  state.plane_normal_world = {0.0, 0.0, 2.0};
  ContactModelParameters p;
  p.normal_stiffness = 1000.0;
  p.normal_damping = 0.0;
  ContactWrench output;
  ASSERT_TRUE(evaluate_contact(p, state, output));
  EXPECT_NEAR(output.force_world.z, 10.0, 1e-12);
}

TEST(ContactModelTest, InvalidInputPreservesOutput) {
  ContactWrench output;
  output.force_world = {7.0, 8.0, 9.0};
  output.normal_force = 11.0;
  output.active = true;
  ContactPointState state;
  state.plane_normal_world = {};
  std::string error;
  EXPECT_FALSE(evaluate_contact({}, state, output, &error));
  EXPECT_EQ(error, "plane normal must have non-zero finite norm");
  EXPECT_DOUBLE_EQ(output.force_world.x, 7.0);
  EXPECT_DOUBLE_EQ(output.normal_force, 11.0);
  EXPECT_TRUE(output.active);
  state.plane_normal_world = {0.0, 0.0, 1.0};
  ContactModelParameters p;
  p.friction_coefficient = -1.0;
  EXPECT_FALSE(evaluate_contact(p, state, output, &error));
}
}  // namespace
