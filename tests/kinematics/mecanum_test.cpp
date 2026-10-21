#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/kinematics/mecanum.hpp"

TEST(MecanumKinematics, RejectsInvalidGeometry) {
  mca::kinematics::MecanumKinematics kinematics;
  EXPECT_FALSE(kinematics.configure(0.0, 0.2, 0.2));
  EXPECT_FALSE(kinematics.configured());
  EXPECT_FALSE(kinematics.configure(0.1, -0.2, 0.2));
  EXPECT_FALSE(kinematics.configure(0.1, 0.2, 0.0));
  EXPECT_FALSE(kinematics.configure(std::numeric_limits<double>::quiet_NaN(), 0.2, 0.2));
}

TEST(MecanumKinematics, ForwardMotionUsesEqualWheelSpeeds) {
  const mca::kinematics::MecanumKinematics kinematics(0.1, 0.2, 0.2);
  mca::kinematics::MecanumWheelSpeeds wheels{};
  ASSERT_TRUE(kinematics.inverse(1.0, 0.0, 0.0, wheels));
  EXPECT_NEAR(wheels.front_left, 10.0, 1e-12);
  EXPECT_NEAR(wheels.front_right, 10.0, 1e-12);
  EXPECT_NEAR(wheels.rear_left, 10.0, 1e-12);
  EXPECT_NEAR(wheels.rear_right, 10.0, 1e-12);
}

TEST(MecanumKinematics, LateralMotionUsesExpectedSigns) {
  const mca::kinematics::MecanumKinematics kinematics(0.1, 0.2, 0.2);
  mca::kinematics::MecanumWheelSpeeds wheels{};
  ASSERT_TRUE(kinematics.inverse(0.0, 1.0, 0.0, wheels));
  EXPECT_NEAR(wheels.front_left, -10.0, 1e-12);
  EXPECT_NEAR(wheels.front_right, 10.0, 1e-12);
  EXPECT_NEAR(wheels.rear_left, 10.0, 1e-12);
  EXPECT_NEAR(wheels.rear_right, -10.0, 1e-12);
}

TEST(MecanumKinematics, PureRotationUsesExpectedSigns) {
  const mca::kinematics::MecanumKinematics kinematics(0.1, 0.2, 0.2);
  mca::kinematics::MecanumWheelSpeeds wheels{};
  ASSERT_TRUE(kinematics.inverse(0.0, 0.0, 1.0, wheels));
  EXPECT_NEAR(wheels.front_left, -4.0, 1e-12);
  EXPECT_NEAR(wheels.front_right, 4.0, 1e-12);
  EXPECT_NEAR(wheels.rear_left, -4.0, 1e-12);
  EXPECT_NEAR(wheels.rear_right, 4.0, 1e-12);
}

TEST(MecanumKinematics, ForwardMatchesAnalyticalModel) {
  const mca::kinematics::MecanumKinematics kinematics(0.1, 0.2, 0.2);
  mca::kinematics::MecanumWheelSpeeds wheels{8.0, 12.0, 10.0, 14.0};
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;
  ASSERT_TRUE(kinematics.forward(wheels, linear_x, linear_y, angular_z));
  EXPECT_NEAR(linear_x, 1.1, 1e-12);
  EXPECT_NEAR(linear_y, 0.2, 1e-12);
  EXPECT_NEAR(angular_z, 0.25, 1e-12);
}

TEST(MecanumKinematics, InverseAndForwardAreRoundTrip) {
  const mca::kinematics::MecanumKinematics kinematics(0.075, 0.31, 0.27);
  constexpr double expected_x = 0.73;
  constexpr double expected_y = -0.41;
  constexpr double expected_yaw = -1.17;
  mca::kinematics::MecanumWheelSpeeds wheels{};
  double actual_x = 0.0;
  double actual_y = 0.0;
  double actual_yaw = 0.0;
  ASSERT_TRUE(kinematics.inverse(expected_x, expected_y, expected_yaw, wheels));
  ASSERT_TRUE(kinematics.forward(wheels, actual_x, actual_y, actual_yaw));
  EXPECT_NEAR(actual_x, expected_x, 1e-12);
  EXPECT_NEAR(actual_y, expected_y, 1e-12);
  EXPECT_NEAR(actual_yaw, expected_yaw, 1e-12);
}

TEST(MecanumKinematics, RejectsNonFiniteInputs) {
  const mca::kinematics::MecanumKinematics kinematics(0.1, 0.2, 0.2);
  mca::kinematics::MecanumWheelSpeeds wheels{};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(kinematics.inverse(nan, 0.0, 0.0, wheels));
  EXPECT_FALSE(kinematics.inverse(0.0, nan, 0.0, wheels));
  EXPECT_FALSE(kinematics.forward({nan, 0.0, 0.0, 0.0}, wheels.front_left,
                                  wheels.front_right, wheels.rear_left));
}
