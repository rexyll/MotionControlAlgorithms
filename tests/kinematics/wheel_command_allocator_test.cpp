#include <array>
#include <limits>

#include <gtest/gtest.h>

#include "mca/kinematics/wheel_command_allocator.hpp"

using Allocator2 = mca::kinematics::WheelCommandAllocator<2>;
using Allocator4 = mca::kinematics::WheelCommandAllocator<4>;

TEST(WheelCommandAllocator, LeavesCommandsUnchangedWhenWithinLimits) {
  const Allocator2 allocator;
  const std::array<double, 2> requested{2.0, -3.0};
  const std::array<double, 2> limits{4.0, 3.0};
  std::array<double, 2> allocated{};
  double scale = 0.0;
  ASSERT_TRUE(allocator.allocate(requested, limits, allocated, scale));
  EXPECT_DOUBLE_EQ(scale, 1.0);
  EXPECT_DOUBLE_EQ(allocated[0], 2.0);
  EXPECT_DOUBLE_EQ(allocated[1], -3.0);
}

TEST(WheelCommandAllocator, ScalesAllWheelsProportionallyToTightestLimit) {
  const Allocator4 allocator;
  const std::array<double, 4> requested{8.0, -4.0, 2.0, -10.0};
  const std::array<double, 4> limits{4.0, 4.0, 1.0, 5.0};
  std::array<double, 4> allocated{};
  double scale = 0.0;
  ASSERT_TRUE(allocator.allocate(requested, limits, allocated, scale));
  EXPECT_DOUBLE_EQ(scale, 0.5);
  EXPECT_DOUBLE_EQ(allocated[0], 4.0);
  EXPECT_DOUBLE_EQ(allocated[1], -2.0);
  EXPECT_DOUBLE_EQ(allocated[2], 1.0);
  EXPECT_DOUBLE_EQ(allocated[3], -5.0);
}

TEST(WheelCommandAllocator, SupportsZeroCommands) {
  const Allocator2 allocator;
  std::array<double, 2> allocated{7.0, 8.0};
  double scale = -1.0;
  ASSERT_TRUE(allocator.allocate({0.0, 0.0}, {1.0, 2.0}, allocated, scale));
  EXPECT_DOUBLE_EQ(scale, 1.0);
  EXPECT_DOUBLE_EQ(allocated[0], 0.0);
  EXPECT_DOUBLE_EQ(allocated[1], 0.0);
}

TEST(WheelCommandAllocator, RejectsInvalidInputsTransactionally) {
  const Allocator2 allocator;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 2> allocated{17.0, 18.0};
  double scale = 19.0;
  EXPECT_FALSE(allocator.allocate({nan, 1.0}, {2.0, 2.0}, allocated, scale));
  EXPECT_DOUBLE_EQ(allocated[0], 17.0);
  EXPECT_DOUBLE_EQ(allocated[1], 18.0);
  EXPECT_DOUBLE_EQ(scale, 19.0);
  EXPECT_FALSE(allocator.allocate({1.0, 2.0}, {2.0, 0.0}, allocated, scale));
  EXPECT_FALSE(allocator.allocate({1.0, 2.0},
                                  {2.0, std::numeric_limits<double>::infinity()},
                                  allocated, scale));
}

TEST(WheelCommandAllocator, PreservesCommandRatios) {
  const Allocator2 allocator;
  std::array<double, 2> allocated{};
  double scale = 0.0;
  ASSERT_TRUE(allocator.allocate({6.0, -3.0}, {2.0, 9.0}, allocated, scale));
  EXPECT_DOUBLE_EQ(allocated[0], 2.0);
  EXPECT_DOUBLE_EQ(allocated[1], -1.0);
  EXPECT_NEAR(allocated[0] / allocated[1], -2.0, 1e-12);
}
