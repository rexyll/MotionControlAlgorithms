#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mca/control/reference_governor.hpp"

namespace {

mca::control::ReferenceGovernor make_governor() {
  mca::control::ReferenceGovernor governor;
  mca::control::ReferenceGovernorConfig config;
  config.position_min = -2.0;
  config.position_max = 2.0;
  config.velocity_max = 1.0;
  config.acceleration_max = 2.0;
  EXPECT_TRUE(governor.configure(config).ok());
  EXPECT_TRUE(governor.reset(0.0).ok());
  return governor;
}

TEST(ReferenceGovernor, LimitsVelocityAndAcceleration) {
  auto governor = make_governor();
  mca::control::ReferenceGovernorOutput output;
  double previous_velocity = 0.0;
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(governor.update(1.5, 0.1, output).ok());
    EXPECT_LE(std::abs(output.velocity), 1.0 + 1.0e-12);
    EXPECT_LE(std::abs(output.acceleration), 2.0 + 1.0e-12);
    EXPECT_LE(std::abs(output.velocity - previous_velocity), 0.2 + 1.0e-12);
    previous_velocity = output.velocity;
  }
}

TEST(ReferenceGovernor, RespectsPositionBounds) {
  auto governor = make_governor();
  ASSERT_TRUE(governor.reset(1.9, 0.0).ok());
  mca::control::ReferenceGovernorOutput output;
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(governor.update(2.0, 0.1, output).ok());
    EXPECT_GE(output.position, -2.0);
    EXPECT_LE(output.position, 2.0);
    EXPECT_LE(std::abs(output.velocity), 1.0 + 1.0e-12);
    EXPECT_LE(std::abs(output.acceleration), 2.0 + 1.0e-12);
  }
}

TEST(ReferenceGovernor, ResetIsRequiredAfterConfiguration) {
  mca::control::ReferenceGovernor governor;
  mca::control::ReferenceGovernorConfig config;
  ASSERT_TRUE(governor.configure(config).ok());
  mca::control::ReferenceGovernorOutput output;
  EXPECT_EQ(governor.update(0.5, 0.01, output).code,
            mca::core::StatusCode::NotConfigured);
}

TEST(ReferenceGovernor, RejectsInvalidInputTransactionally) {
  auto governor = make_governor();
  mca::control::ReferenceGovernorOutput output;
  output.position = 123.0;
  EXPECT_EQ(governor.update(std::numeric_limits<double>::quiet_NaN(), 0.1, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_DOUBLE_EQ(output.position, 123.0);
  EXPECT_EQ(governor.update(0.5, 0.0, output).code,
            mca::core::StatusCode::InvalidArgument);
  EXPECT_EQ(governor.update(3.0, 0.1, output).code,
            mca::core::StatusCode::InvalidArgument);
}

TEST(ReferenceGovernor, ConvergesTowardTarget) {
  auto governor = make_governor();
  mca::control::ReferenceGovernorOutput output;
  for (int i = 0; i < 500; ++i) ASSERT_TRUE(governor.update(0.8, 0.01, output).ok());
  EXPECT_NEAR(output.position, 0.8, 0.02);
  EXPECT_NEAR(output.velocity, 0.0, 0.02);
}

}  // namespace
