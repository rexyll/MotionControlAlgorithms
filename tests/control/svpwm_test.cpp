#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/control/svpwm.hpp"
#include "mca/transform/clarke_park.hpp"

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kSqrt3 = 1.7320508075688772935;

mca::control::Svpwm make_svpwm() {
  mca::control::Svpwm controller;
  ASSERT_TRUE(controller.configure({48.0, 0.001}).ok());
  return controller;
}

template <typename T>
void expect_near(T actual, T expected, double tolerance) {
  EXPECT_NEAR(static_cast<double>(actual), static_cast<double>(expected),
              tolerance);
}

TEST(Svpwm, RejectsInvalidConfiguration) {
  mca::control::Svpwm controller;
  EXPECT_FALSE(controller.configure({0.0, 0.001}).ok());
  EXPECT_FALSE(controller.configured());
  EXPECT_FALSE(controller.configure({48.0, 0.0}).ok());
  EXPECT_FALSE(controller.configure(
                         {std::numeric_limits<double>::quiet_NaN(), 0.001})
                   .ok());
}

TEST(Svpwm, RejectsInputBeforeConfigurationAndNonFiniteInput) {
  mca::control::Svpwm controller;
  mca::control::SvpwmOutput output;
  EXPECT_FALSE(controller.compute({0.0, 0.0}, output).ok());

  ASSERT_TRUE(controller.configure({48.0, 0.001}).ok());
  EXPECT_FALSE(controller
                   .compute({std::numeric_limits<double>::infinity(), 0.0},
                            output)
                   .ok());
}

TEST(Svpwm, ZeroVectorProducesCenteredDutyAndFullZeroVectorTime) {
  auto controller = make_svpwm();
  mca::control::SvpwmOutput output;
  ASSERT_TRUE(controller.compute({0.0, 0.0}, output).ok());

  expect_near(output.duty_a, 0.5, 1e-12);
  expect_near(output.duty_b, 0.5, 1e-12);
  expect_near(output.duty_c, 0.5, 1e-12);
  expect_near(output.t0_s, 0.001, 1e-12);
  EXPECT_DOUBLE_EQ(output.t1_s, 0.0);
  EXPECT_DOUBLE_EQ(output.t2_s, 0.0);
  EXPECT_EQ(output.sector, 0);
  EXPECT_FALSE(output.overmodulated);
}

TEST(Svpwm, SixSectorsAreDetected) {
  auto controller = make_svpwm();
  for (int expected_sector = 1; expected_sector <= 6; ++expected_sector) {
    const double angle = (static_cast<double>(expected_sector) - 0.5) * kPi / 3.0;
    mca::control::SvpwmOutput output;
    ASSERT_TRUE(controller.compute(
                              {12.0 * std::cos(angle), 12.0 * std::sin(angle)},
                              output)
                    .ok());
    EXPECT_EQ(output.sector, expected_sector);
  }
}

TEST(Svpwm, AxisAndSectorBoundaryDwellTimesAreConsistent) {
  auto controller = make_svpwm();
  mca::control::SvpwmOutput output;

  ASSERT_TRUE(controller.compute({8.0, 0.0}, output).ok());
  EXPECT_EQ(output.sector, 1);
  expect_near(output.t2_s, 0.0, 1e-12);
  expect_near(output.t1_s + output.t2_s + output.t0_s, 0.001, 1e-12);

  const double boundary = kPi / 3.0;
  ASSERT_TRUE(controller.compute(
                              {8.0 * std::cos(boundary), 8.0 * std::sin(boundary)},
                              output)
                  .ok());
  EXPECT_EQ(output.sector, 2);
  expect_near(output.t1_s, 0.0, 1e-12);
  expect_near(output.t1_s + output.t2_s + output.t0_s, 0.001, 1e-12);
}

TEST(Svpwm, DwellTimesAndDutiesStayWithinPhysicalBounds) {
  auto controller = make_svpwm();
  for (int i = 0; i < 360; ++i) {
    const double angle = static_cast<double>(i) * kPi / 180.0;
    mca::control::SvpwmOutput output;
    ASSERT_TRUE(controller.compute(
                              {20.0 * std::cos(angle), 20.0 * std::sin(angle)},
                              output)
                    .ok());
    EXPECT_GE(output.t1_s, 0.0);
    EXPECT_GE(output.t2_s, 0.0);
    EXPECT_GE(output.t0_s, 0.0);
    expect_near(output.t1_s + output.t2_s + output.t0_s, 0.001, 1e-12);
    EXPECT_GE(output.duty_a, 0.0);
    EXPECT_LE(output.duty_a, 1.0);
    EXPECT_GE(output.duty_b, 0.0);
    EXPECT_LE(output.duty_b, 1.0);
    EXPECT_GE(output.duty_c, 0.0);
    EXPECT_LE(output.duty_c, 1.0);
  }
}

TEST(Svpwm, LinearLimitIsClampedAndMarkedAsOvermodulation) {
  auto controller = make_svpwm();
  const double limit = mca::control::Svpwm::linear_voltage_limit(48.0);
  mca::control::SvpwmOutput output;

  ASSERT_TRUE(controller.compute({2.0 * limit, 0.0}, output).ok());
  EXPECT_TRUE(output.overmodulated);
  expect_near(output.requested_magnitude_v, 2.0 * limit, 1e-12);
  expect_near(output.applied_magnitude_v, limit, 1e-12);
  expect_near(output.t1_s + output.t2_s + output.t0_s, 0.001, 1e-12);
  EXPECT_GE(output.duty_a, 0.0);
  EXPECT_LE(output.duty_a, 1.0);
  EXPECT_GE(output.duty_b, 0.0);
  EXPECT_LE(output.duty_b, 1.0);
  EXPECT_GE(output.duty_c, 0.0);
  EXPECT_LE(output.duty_c, 1.0);
}

TEST(Svpwm, AppliedVectorMatchesRequestedVectorInLinearRegion) {
  auto controller = make_svpwm();
  for (int i = 0; i < 24; ++i) {
    const double angle = static_cast<double>(i) * kPi / 12.0 + 0.07;
    const double magnitude = 10.0;
    mca::control::SvpwmOutput output;
    ASSERT_TRUE(controller.compute(
                              {magnitude * std::cos(angle),
                               magnitude * std::sin(angle)},
                              output)
                    .ok());

    const double va = (output.duty_a - 0.5) * 48.0;
    const double vb = (output.duty_b - 0.5) * 48.0;
    const double vc = (output.duty_c - 0.5) * 48.0;
    const auto ab0 = mca::transform::clarke({va, vb, vc});
    expect_near(ab0.alpha, magnitude * std::cos(angle), 1e-10);
    expect_near(ab0.beta, magnitude * std::sin(angle), 1e-10);
  }
}

TEST(Svpwm, NegativeAnglesWrapIntoPositiveSectors) {
  auto controller = make_svpwm();
  mca::control::SvpwmOutput output;
  ASSERT_TRUE(controller.compute({8.0, -1.0}, output).ok());
  EXPECT_EQ(output.sector, 6);
}

}  // namespace
