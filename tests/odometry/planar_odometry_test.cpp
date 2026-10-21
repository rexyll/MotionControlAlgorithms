#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/odometry/planar_odometry.hpp"

TEST(PlanarOdometry, ResetAndStraightMotion) {
  mca::odometry::PlanarOdometry odometry;
  ASSERT_TRUE(odometry.reset({1.0, -2.0, 0.0}));
  ASSERT_TRUE(odometry.update({2.0, 0.0, 0.0}, 0.5));
  EXPECT_NEAR(odometry.pose().x, 2.0, 1e-12);
  EXPECT_NEAR(odometry.pose().y, -2.0, 1e-12);
  EXPECT_NEAR(odometry.pose().yaw, 0.0, 1e-12);
}

TEST(PlanarOdometry, RespectsInitialHeading) {
  mca::odometry::PlanarOdometry odometry;
  ASSERT_TRUE(odometry.reset({0.0, 0.0, M_PI_2}));
  ASSERT_TRUE(odometry.update({2.0, 0.0, 0.0}, 0.5));
  EXPECT_NEAR(odometry.pose().x, 0.0, 1e-12);
  EXPECT_NEAR(odometry.pose().y, 1.0, 1e-12);
}

TEST(PlanarOdometry, ConstantTwistMatchesExactCircularArc) {
  mca::odometry::PlanarOdometry odometry;
  constexpr double v = 1.0;
  constexpr double wz = 2.0;
  constexpr double dt = 0.5;
  ASSERT_TRUE(odometry.update({v, 0.0, wz}, dt));
  EXPECT_NEAR(odometry.pose().x, std::sin(1.0) / 2.0, 1e-12);
  EXPECT_NEAR(odometry.pose().y, (1.0 - std::cos(1.0)) / 2.0, 1e-12);
  EXPECT_NEAR(odometry.pose().yaw, 1.0, 1e-12);
}

TEST(PlanarOdometry, SupportsBodyLateralVelocity) {
  mca::odometry::PlanarOdometry odometry;
  ASSERT_TRUE(odometry.update({0.0, 1.0, 0.0}, 0.75));
  EXPECT_NEAR(odometry.pose().x, 0.0, 1e-12);
  EXPECT_NEAR(odometry.pose().y, 0.75, 1e-12);
}

TEST(PlanarOdometry, RejectsInvalidInputsWithoutChangingPose) {
  mca::odometry::PlanarOdometry odometry;
  ASSERT_TRUE(odometry.reset({1.0, 2.0, 0.3}));
  const auto before = odometry.pose();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(odometry.update({nan, 0.0, 0.0}, 0.1));
  EXPECT_FALSE(odometry.update({0.0, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(odometry.update({0.0, 0.0, 0.0}, -0.1));
  EXPECT_NEAR(odometry.pose().x, before.x, 0.0);
  EXPECT_NEAR(odometry.pose().y, before.y, 0.0);
  EXPECT_NEAR(odometry.pose().yaw, before.yaw, 0.0);
  EXPECT_FALSE(odometry.reset({nan, 0.0, 0.0}));
}
