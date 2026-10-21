#include "mca/uav/rigid_body_model_6d.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::geometry::Quaternion;
using mca::geometry::Vec3;
using mca::uav::RigidBodyDerivative6D;
using mca::uav::RigidBodyInput6D;
using mca::uav::RigidBodyModel6D;
using mca::uav::RigidBodyParameters6D;
using mca::uav::RigidBodyState6D;

RigidBodyParameters6D parameters() {
  RigidBodyParameters6D p;
  p.mass = 2.0;
  p.inertia_body = {{2.0, 0.0, 0.0,
                     0.0, 3.0, 0.0,
                     0.0, 0.0, 4.0}};
  return p;
}

TEST(RigidBodyModel6DTest, HoverThrustCancelsGravity) {
  const auto p = parameters();
  const RigidBodyState6D state{};
  const RigidBodyInput6D input{{0.0, 0.0, 19.62}, {}};
  RigidBodyDerivative6D derivative;
  ASSERT_TRUE(RigidBodyModel6D::derivative(p, state, input, derivative));
  EXPECT_TRUE((derivative.position_rate_world.x == 0.0));
  EXPECT_NEAR(derivative.velocity_rate_world.x, 0.0, 1e-12);
  EXPECT_NEAR(derivative.velocity_rate_world.y, 0.0, 1e-12);
  EXPECT_NEAR(derivative.velocity_rate_world.z, 0.0, 1e-12);
}

TEST(RigidBodyModel6DTest, RotatesBodyForceIntoWorldFrame) {
  const auto p = parameters();
  RigidBodyState6D state;
  state.attitude_body_to_world = mca::geometry::axis_angle({0.0, 1.0, 0.0},
                                                            1.5707963267948966);
  const RigidBodyInput6D input{{0.0, 0.0, 2.0}, {}};
  RigidBodyDerivative6D derivative;
  ASSERT_TRUE(RigidBodyModel6D::derivative(p, state, input, derivative));
  EXPECT_NEAR(derivative.velocity_rate_world.x, 1.0, 1e-12);
  EXPECT_NEAR(derivative.velocity_rate_world.y, 0.0, 1e-12);
  EXPECT_NEAR(derivative.velocity_rate_world.z, -9.81, 1e-12);
}

TEST(RigidBodyModel6DTest, EulerRigidBodyEquationMatchesDiagonalInertia) {
  const auto p = parameters();
  RigidBodyState6D state;
  state.angular_velocity_body = {1.0, 2.0, 3.0};
  RigidBodyDerivative6D derivative;
  ASSERT_TRUE(RigidBodyModel6D::derivative(p, state, {}, derivative));
  EXPECT_NEAR(derivative.angular_acceleration_body.x, -3.0, 1e-12);
  EXPECT_NEAR(derivative.angular_acceleration_body.y, 2.0, 1e-12);
  EXPECT_NEAR(derivative.angular_acceleration_body.z, -0.5, 1e-12);
  EXPECT_NEAR(derivative.attitude_rate.w, 0.0, 1e-12);
  EXPECT_NEAR(derivative.attitude_rate.x, 0.5, 1e-12);
  EXPECT_NEAR(derivative.attitude_rate.y, 1.0, 1e-12);
  EXPECT_NEAR(derivative.attitude_rate.z, 1.5, 1e-12);
}

TEST(RigidBodyModel6DTest, NormalizesQuaternionForDynamics) {
  auto state = RigidBodyState6D{};
  state.attitude_body_to_world = {2.0, 0.0, 0.0, 0.0};
  RigidBodyDerivative6D derivative;
  ASSERT_TRUE(RigidBodyModel6D::derivative(parameters(), state, {}, derivative));
  EXPECT_NEAR(derivative.velocity_rate_world.z, -9.81, 1e-12);
}

TEST(RigidBodyModel6DTest, InvalidInputPreservesOutput) {
  auto p = parameters();
  RigidBodyState6D state;
  state.attitude_body_to_world = {0.0, 0.0, 0.0, 0.0};
  RigidBodyDerivative6D derivative;
  derivative.position_rate_world = {7.0, 8.0, 9.0};
  std::string error;
  EXPECT_FALSE(RigidBodyModel6D::derivative(p, state, {}, derivative, &error));
  EXPECT_EQ(error, "attitude quaternion must have non-zero finite norm");
  EXPECT_DOUBLE_EQ(derivative.position_rate_world.x, 7.0);
  EXPECT_DOUBLE_EQ(derivative.position_rate_world.y, 8.0);
  EXPECT_DOUBLE_EQ(derivative.position_rate_world.z, 9.0);

  state.attitude_body_to_world = {};
  p.mass = 0.0;
  EXPECT_FALSE(RigidBodyModel6D::derivative(p, state, {}, derivative, &error));
  EXPECT_DOUBLE_EQ(derivative.position_rate_world.x, 7.0);
}

TEST(RigidBodyModel6DTest, RejectsNonPhysicalInertia) {
  auto p = parameters();
  p.inertia_body(0, 1) = 0.25;
  RigidBodyDerivative6D derivative;
  EXPECT_FALSE(RigidBodyModel6D::derivative(p, {}, {}, derivative));
  p = parameters();
  p.inertia_body(2, 2) = -1.0;
  EXPECT_FALSE(RigidBodyModel6D::derivative(p, {}, {}, derivative));
}

}  // namespace
