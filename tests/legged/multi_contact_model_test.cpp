#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mca/legged/multi_contact_model.hpp"

namespace {

mca::legged::MultiContactPoint make_contact(double x, double y, double penetration) {
  mca::legged::MultiContactPoint point;
  point.position_world = {x, y, 0.0};
  point.state.gap = -penetration;
  point.state.velocity_world = {0.0, 0.0, 0.0};
  point.state.plane_normal_world = {0.0, 0.0, 1.0};
  point.parameters.normal_stiffness = 1000.0;
  point.parameters.normal_damping = 20.0;
  point.parameters.friction_coefficient = 0.5;
  point.parameters.tangential_damping = 50.0;
  return point;
}

}  // namespace

TEST(MultiContactModelTest, AggregatesForcesAndMoments) {
  mca::legged::MultiContactInput input;
  input.reference_point_world = {0.0, 0.0, 0.0};
  input.points.push_back(make_contact(0.5, 0.0, 0.01));
  input.points.push_back(make_contact(-0.5, 0.0, 0.02));

  mca::legged::MultiContactOutput output;
  ASSERT_TRUE(mca::legged::MultiContactModel().evaluate(input, &output));
  ASSERT_EQ(output.contacts.size(), 2U);
  EXPECT_EQ(output.active_contacts, 2U);
  EXPECT_NEAR(output.total_force_world.z, 30.0, 1e-12);
  EXPECT_NEAR(output.total_force_world.x, 0.0, 1e-12);
  EXPECT_NEAR(output.total_force_world.y, 0.0, 1e-12);
  EXPECT_NEAR(output.total_torque_world.x, 0.0, 1e-12);
  EXPECT_NEAR(output.total_torque_world.y, 0.0, 1e-12);
  EXPECT_NEAR(output.total_torque_world.z, 0.0, 1e-12);
}

TEST(MultiContactModelTest, PreservesContactOrderAndInactiveContacts) {
  mca::legged::MultiContactInput input;
  input.reference_point_world = {0.0, 0.0, 0.0};
  input.points.push_back(make_contact(0.0, 0.0, 0.01));
  input.points.push_back(make_contact(1.0, 0.0, 0.0));

  mca::legged::MultiContactOutput output;
  ASSERT_TRUE(mca::legged::MultiContactModel().evaluate(input, &output));
  ASSERT_EQ(output.contacts.size(), 2U);
  EXPECT_TRUE(output.contacts[0].active);
  EXPECT_FALSE(output.contacts[1].active);
  EXPECT_EQ(output.active_contacts, 1U);
  EXPECT_NEAR(output.total_force_world.z, 10.0, 1e-12);
}

TEST(MultiContactModelTest, RejectsEmptySetAndNonFinitePosition) {
  mca::legged::MultiContactModel model;
  mca::legged::MultiContactOutput output;
  output.total_force_world = {1.0, 2.0, 3.0};

  mca::legged::MultiContactInput empty;
  empty.reference_point_world = {0.0, 0.0, 0.0};
  EXPECT_FALSE(model.evaluate(empty, &output));
  EXPECT_NEAR(output.total_force_world.x, 1.0, 1e-12);

  auto invalid = make_contact(0.0, 0.0, 0.01);
  invalid.position_world.x = std::numeric_limits<double>::quiet_NaN();
  mca::legged::MultiContactInput input;
  input.reference_point_world = {0.0, 0.0, 0.0};
  input.points.push_back(invalid);
  EXPECT_FALSE(model.evaluate(input, &output));
  EXPECT_NEAR(output.total_force_world.y, 2.0, 1e-12);
}

TEST(MultiContactModelTest, RejectsInvalidContactTransactionally) {
  mca::legged::MultiContactInput input;
  input.reference_point_world = {0.0, 0.0, 0.0};
  input.points.push_back(make_contact(0.0, 0.0, 0.01));
  auto invalid = make_contact(0.5, 0.0, 0.01);
  invalid.parameters.friction_coefficient = -1.0;
  input.points.push_back(invalid);

  mca::legged::MultiContactOutput output;
  output.total_torque_world = {4.0, 5.0, 6.0};
  output.active_contacts = 9U;
  ASSERT_FALSE(mca::legged::MultiContactModel().evaluate(input, &output));
  EXPECT_NEAR(output.total_torque_world.x, 4.0, 1e-12);
  EXPECT_NEAR(output.total_torque_world.y, 5.0, 1e-12);
  EXPECT_NEAR(output.total_torque_world.z, 6.0, 1e-12);
  EXPECT_EQ(output.active_contacts, 9U);
}
