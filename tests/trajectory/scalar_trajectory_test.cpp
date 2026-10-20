#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/trajectory/scalar.hpp"

TEST(ScalarTrajectory, StepHasConstantTargetAndZeroDerivatives) {
  const mca::trajectory::StepTrajectory trajectory(2.0, -1.0);
  const auto before = trajectory.sample(-0.1);
  const auto at_zero = trajectory.sample(0.0);
  const auto after = trajectory.sample(1.0);
  EXPECT_DOUBLE_EQ(before.position, -1.0);
  EXPECT_DOUBLE_EQ(at_zero.position, 2.0);
  EXPECT_DOUBLE_EQ(after.position, 2.0);
  EXPECT_DOUBLE_EQ(after.velocity, 0.0);
  EXPECT_DOUBLE_EQ(after.acceleration, 0.0);
}

TEST(ScalarTrajectory, RampHasAnalyticalSlopeAndClampsAtEndpoints) {
  const mca::trajectory::RampTrajectory trajectory(0.0, 2.0, 4.0);
  const auto before = trajectory.sample(-1.0);
  const auto middle = trajectory.sample(2.0);
  const auto end = trajectory.sample(5.0);
  EXPECT_DOUBLE_EQ(before.position, 0.0);
  EXPECT_DOUBLE_EQ(before.velocity, 0.0);
  EXPECT_DOUBLE_EQ(middle.position, 1.0);
  EXPECT_DOUBLE_EQ(middle.velocity, 0.5);
  EXPECT_DOUBLE_EQ(middle.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(end.position, 2.0);
  EXPECT_DOUBLE_EQ(end.velocity, 0.0);
  EXPECT_DOUBLE_EQ(end.acceleration, 0.0);
}

TEST(ScalarTrajectory, RampWithNonPositiveDurationReturnsFinishState) {
  const mca::trajectory::RampTrajectory trajectory(1.0, 3.0, 0.0);
  const auto sample = trajectory.sample(0.5);
  EXPECT_DOUBLE_EQ(sample.position, 3.0);
  EXPECT_DOUBLE_EQ(sample.velocity, 0.0);
  EXPECT_DOUBLE_EQ(sample.acceleration, 0.0);
}

TEST(ScalarTrajectory, SineProvidesAnalyticalPositionVelocityAcceleration) {
  constexpr double kPi = 3.14159265358979323846;
  const mca::trajectory::SineTrajectory trajectory(2.0, 1.0);
  const auto sample = trajectory.sample(0.25);
  EXPECT_NEAR(sample.position, 2.0, 1e-12);
  EXPECT_NEAR(sample.velocity, 0.0, 1e-11);
  EXPECT_NEAR(sample.acceleration, -2.0 * 4.0 * kPi * kPi, 1e-10);
}

TEST(ScalarTrajectory, SinePreservesOffsetAndPhase) {
  constexpr double kPi = 3.14159265358979323846;
  const mca::trajectory::SineTrajectory trajectory(2.0, 0.5, 1.5, kPi / 2.0);
  const auto sample = trajectory.sample(0.0);
  EXPECT_NEAR(sample.position, 3.5, 1e-12);
  EXPECT_NEAR(sample.velocity, 0.0, 1e-12);
  EXPECT_NEAR(sample.acceleration, -2.0 * 1.0 * kPi * kPi, 1e-11);
}

TEST(ScalarTrajectory, SineSupportsNegativeFrequencyConsistently) {
  const mca::trajectory::SineTrajectory trajectory(1.0, -1.0);
  const auto sample = trajectory.sample(0.125);
  EXPECT_NEAR(sample.position, std::sin(-0.25 * 3.14159265358979323846), 1e-12);
  EXPECT_NEAR(sample.velocity, -2.0 * 3.14159265358979323846 *
                                   std::cos(-0.25 * 3.14159265358979323846), 1e-11);
}

TEST(ScalarTrajectory, FiniteParametersProduceFiniteSamples) {
  const mca::trajectory::StepTrajectory step(1.0, -1.0);
  const mca::trajectory::RampTrajectory ramp(-1.0, 2.0, 1.0);
  const mca::trajectory::SineTrajectory sine(1.0, 2.0, 0.0, 0.5);
  const auto a = step.sample(0.2);
  const auto b = ramp.sample(0.2);
  const auto c = sine.sample(0.2);
  EXPECT_TRUE(std::isfinite(a.position));
  EXPECT_TRUE(std::isfinite(b.position));
  EXPECT_TRUE(std::isfinite(b.velocity));
  EXPECT_TRUE(std::isfinite(c.position));
  EXPECT_TRUE(std::isfinite(c.velocity));
  EXPECT_TRUE(std::isfinite(c.acceleration));
}
