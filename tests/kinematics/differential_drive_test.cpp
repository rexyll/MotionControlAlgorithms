#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/kinematics/differential_drive.hpp"

TEST(DifferentialDriveKinematics, RejectsInvalidGeometry) {
  mca::kinematics::DifferentialDriveKinematics kinematics;
  EXPECT_FALSE(kinematics.configure(0.0, 0.5));
  EXPECT_FALSE(kinematics.configured());
  EXPECT_FALSE(kinematics.configure(0.1, -0.5));
  EXPECT_FALSE(kinematics.configure(std::numeric_limits<double>::quiet_NaN(), 0.5));
}

TEST(DifferentialDriveKinematics, StraightMotionUsesEqualWheelSpeeds) {
  const mca::kinematics::DifferentialDriveKinematics kinematics(0.1, 0.5);
  mca::kinematics::WheelSpeeds wheels{};
  ASSERT_TRUE(kinematics.inverse({1.0, 0.0}, wheels));
  EXPECT_NEAR(wheels.left, 10.0, 1e-12);
  EXPECT_NEAR(wheels.right, 10.0, 1e-12);
}

TEST(DifferentialDriveKinematics, PureRotationUsesOppositeWheelSpeeds) {
  const mca::kinematics::DifferentialDriveKinematics kinematics(0.1, 0.5);
  mca::kinematics::WheelSpeeds wheels{};
  ASSERT_TRUE(kinematics.inverse({0.0, 2.0}, wheels));
  EXPECT_NEAR(wheels.left, -5.0, 1e-12);
  EXPECT_NEAR(wheels.right, 5.0, 1e-12);
}

TEST(DifferentialDriveKinematics, ForwardMatchesAnalyticalModel) {
  const mca::kinematics::DifferentialDriveKinematics kinematics(0.1, 0.5);
  mca::kinematics::Twist2D twist{};
  ASSERT_TRUE(kinematics.forward({8.0, 12.0}, twist));
  EXPECT_NEAR(twist.linear_x, 1.0, 1e-12);
  EXPECT_NEAR(twist.angular_z, 0.8, 1e-12);
}

TEST(DifferentialDriveKinematics, InverseAndForwardAreRoundTrip) {
  const mca::kinematics::DifferentialDriveKinematics kinematics(0.075, 0.42);
  const mca::kinematics::Twist2D expected{0.73, -1.17};
  mca::kinematics::WheelSpeeds wheels{};
  mca::kinematics::Twist2D actual{};
  ASSERT_TRUE(kinematics.inverse(expected, wheels));
  ASSERT_TRUE(kinematics.forward(wheels, actual));
  EXPECT_NEAR(actual.linear_x, expected.linear_x, 1e-12);
  EXPECT_NEAR(actual.angular_z, expected.angular_z, 1e-12);
}

TEST(DifferentialDriveKinematics, RejectsNonFiniteInputs) {
  const mca::kinematics::DifferentialDriveKinematics kinematics(0.1, 0.5);
  mca::kinematics::WheelSpeeds wheels{};
  mca::kinematics::Twist2D twist{};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(kinematics.inverse({nan, 0.0}, wheels));
  EXPECT_FALSE(kinematics.forward({0.0, nan}, twist));
}
