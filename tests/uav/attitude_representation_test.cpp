#include "mca/uav/attitude_representation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace {
using mca::geometry::Quaternion;
using mca::uav::AttitudeRepresentation;
using mca::uav::EulerZYX;
constexpr double kPi = 3.14159265358979323846;

TEST(AttitudeRepresentationTest, EulerQuaternionRoundTrip) {
  const EulerZYX expected{0.3, -0.4, 1.2};
  Quaternion q;
  ASSERT_TRUE(AttitudeRepresentation::from_euler_zyx(expected, q));
  EulerZYX actual;
  ASSERT_TRUE(AttitudeRepresentation::to_euler_zyx(q, actual));
  EXPECT_NEAR(actual.roll, expected.roll, 1e-12);
  EXPECT_NEAR(actual.pitch, expected.pitch, 1e-12);
  EXPECT_NEAR(actual.yaw, expected.yaw, 1e-12);
}

TEST(AttitudeRepresentationTest, QuaternionAndMatrixAgree) {
  Quaternion q;
  ASSERT_TRUE(AttitudeRepresentation::from_euler_zyx({0.2, 0.5, -0.8}, q));
  const auto matrix = AttitudeRepresentation::to_rotation_matrix(q);
  Quaternion recovered;
  ASSERT_TRUE(AttitudeRepresentation::from_rotation_matrix(matrix, recovered));
  const auto a = mca::geometry::to_rotation_matrix(q);
  const auto b = mca::geometry::to_rotation_matrix(recovered);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      EXPECT_NEAR(a(r,c), b(r,c), 1e-12);
}

TEST(AttitudeRepresentationTest, RejectsNonFiniteEulerTransactionally) {
  Quaternion q{0.5, 0.5, 0.5, 0.5};
  std::string error;
  EXPECT_FALSE(AttitudeRepresentation::from_euler_zyx(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, q, &error));
  EXPECT_EQ(error, "Euler angles must be finite");
  EXPECT_DOUBLE_EQ(q.w, 0.5);
}

TEST(AttitudeRepresentationTest, RejectsInvalidQuaternionTransactionally) {
  EulerZYX e{1.0, 2.0, 3.0};
  EXPECT_FALSE(AttitudeRepresentation::to_euler_zyx({0.0, 0.0, 0.0, 0.0}, e));
  EXPECT_DOUBLE_EQ(e.roll, 1.0);
}

TEST(AttitudeRepresentationTest, RejectsNonRotationMatrix) {
  auto matrix = mca::geometry::Mat3{};
  matrix(0,0) = 2.0;
  Quaternion q{0.5, 0.5, 0.5, 0.5};
  EXPECT_FALSE(AttitudeRepresentation::from_rotation_matrix(matrix, q));
  EXPECT_DOUBLE_EQ(q.w, 0.5);
}

TEST(AttitudeRepresentationTest, HandlesPitchNearGimbalLock) {
  Quaternion q;
  ASSERT_TRUE(AttitudeRepresentation::from_euler_zyx({0.1, kPi / 2.0 - 1e-7, -0.2}, q));
  EulerZYX e;
  ASSERT_TRUE(AttitudeRepresentation::to_euler_zyx(q, e));
  EXPECT_NEAR(e.pitch, kPi / 2.0 - 1e-7, 1e-8);
}
}  // namespace
