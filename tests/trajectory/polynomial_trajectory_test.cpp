#include <cmath>
#include <gtest/gtest.h>

#include "mca/trajectory/polynomial.hpp"

namespace {
using mca::trajectory::CubicTrajectory;
using mca::trajectory::QuinticTrajectory;

TEST(PolynomialTrajectory, CubicMatchesPositionAndVelocityBoundaryConditions) {
  const CubicTrajectory trajectory(1.0, 0.5, 4.0, -0.25, 2.0);
  const auto start = trajectory.sample(0.0);
  const auto finish = trajectory.sample(2.0);
  EXPECT_DOUBLE_EQ(start.position, 1.0);
  EXPECT_DOUBLE_EQ(start.velocity, 0.5);
  EXPECT_NEAR(finish.position, 4.0, 1e-12);
  EXPECT_NEAR(finish.velocity, -0.25, 1e-12);
}

TEST(PolynomialTrajectory, CubicProducesAnalyticalInteriorSample) {
  const CubicTrajectory trajectory(0.0, 0.0, 1.0, 0.0, 2.0);
  const auto middle = trajectory.sample(1.0);
  EXPECT_NEAR(middle.position, 0.5, 1e-12);
  EXPECT_NEAR(middle.velocity, 0.75, 1e-12);
  EXPECT_NEAR(middle.acceleration, 0.0, 1e-12);
}

TEST(PolynomialTrajectory, QuinticMatchesPositionVelocityAndAccelerationAtBothEnds) {
  const QuinticTrajectory trajectory(1.0, 0.2, -0.1, 3.0, -0.4, 0.3, 1.5);
  const auto start = trajectory.sample(0.0);
  const auto finish = trajectory.sample(1.5);
  EXPECT_DOUBLE_EQ(start.position, 1.0);
  EXPECT_DOUBLE_EQ(start.velocity, 0.2);
  EXPECT_DOUBLE_EQ(start.acceleration, -0.1);
  EXPECT_NEAR(finish.position, 3.0, 1e-10);
  EXPECT_NEAR(finish.velocity, -0.4, 1e-10);
  EXPECT_NEAR(finish.acceleration, 0.3, 1e-10);
}

TEST(PolynomialTrajectory, QuinticRestToRestHasZeroEndpointDerivatives) {
  const QuinticTrajectory trajectory(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0);
  const auto start = trajectory.sample(0.0);
  const auto finish = trajectory.sample(2.0);
  EXPECT_DOUBLE_EQ(start.velocity, 0.0);
  EXPECT_DOUBLE_EQ(start.acceleration, 0.0);
  EXPECT_NEAR(finish.velocity, 0.0, 1e-12);
  EXPECT_NEAR(finish.acceleration, 0.0, 1e-12);
  EXPECT_NEAR(trajectory.sample(1.0).position, 0.5, 1e-12);
}

TEST(PolynomialTrajectory, SamplesClampOutsideDurationAndHandleNonPositiveDuration) {
  const CubicTrajectory cubic(1.0, 0.0, 2.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(cubic.sample(-1.0).position, 1.0);
  EXPECT_DOUBLE_EQ(cubic.sample(2.0).position, 2.0);
  const QuinticTrajectory invalid(1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0);
  const auto sample = invalid.sample(0.5);
  EXPECT_DOUBLE_EQ(sample.position, 2.0);
  EXPECT_DOUBLE_EQ(sample.velocity, 0.0);
  EXPECT_DOUBLE_EQ(sample.acceleration, 0.0);
}

TEST(PolynomialTrajectory, InteriorSamplesRemainFinite) {
  const CubicTrajectory cubic(-1.0, 0.2, 2.0, -0.1, 1.25);
  const QuinticTrajectory quintic(-1.0, 0.2, 0.3, 2.0, -0.1, 0.0, 1.25);
  const auto a = cubic.sample(0.6);
  const auto b = quintic.sample(0.6);
  EXPECT_TRUE(std::isfinite(a.position));
  EXPECT_TRUE(std::isfinite(a.velocity));
  EXPECT_TRUE(std::isfinite(a.acceleration));
  EXPECT_TRUE(std::isfinite(b.position));
  EXPECT_TRUE(std::isfinite(b.velocity));
  EXPECT_TRUE(std::isfinite(b.acceleration));
}
}  // namespace
