#include <gtest/gtest.h>

#include "mca/core/math.hpp"

TEST(Math, Clamp) {
  EXPECT_DOUBLE_EQ(mca::math::clamp(2.0, -1.0, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(mca::math::clamp(-2.0, -1.0, 1.0), -1.0);
}

TEST(Math, WrapAngle) {
  constexpr double pi = 3.14159265358979323846;
  EXPECT_NEAR(mca::math::wrap_angle(3.0 * pi), pi, 1e-12);
  EXPECT_NEAR(mca::math::wrap_angle(-3.0 * pi), -pi, 1e-12);
}

TEST(Math, NearlyEqual) {
  EXPECT_TRUE(mca::math::is_nearly_equal(1.0, 1.0 + 1e-10));
  EXPECT_FALSE(mca::math::is_nearly_equal(1.0, 1.1));
}
