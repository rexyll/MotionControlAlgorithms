#include <limits>

#include <gtest/gtest.h>

#include "mca/control/friction_compensation.hpp"

namespace {

mca::control::FrictionCompensationConfig make_config() {
  mca::control::FrictionCompensationConfig c;
  c.static_friction_nm = 0.8;
  c.coulomb_friction_nm = 0.5;
  c.viscous_friction_nm_per_rad_s = 0.1;
  c.velocity_deadband_rad_s = 0.2;
  c.compensation_min_nm = -10.0;
  c.compensation_max_nm = 10.0;
  return c;
}

TEST(FrictionCompensator, RejectsInvalidConfigurationAndInputs) {
  mca::control::FrictionCompensator compensator;
  auto config = make_config();
  config.static_friction_nm = -0.1;
  EXPECT_FALSE(compensator.configure(config).ok());

  config = make_config();
  ASSERT_TRUE(compensator.configure(config).ok());
  double output = 0.0;
  EXPECT_FALSE(compensator
                   .compute(std::numeric_limits<double>::quiet_NaN(), 1.0,
                            output)
                   .ok());
}

TEST(FrictionCompensator, CompensatesCoulombAndViscousFrictionInMotion) {
  mca::control::FrictionCompensator compensator(make_config());
  double output = 0.0;

  ASSERT_TRUE(compensator.compute(3.0, 0.0, output).ok());
  EXPECT_NEAR(output, 0.8, 1e-12);

  ASSERT_TRUE(compensator.compute(-3.0, 0.0, output).ok());
  EXPECT_NEAR(output, -0.8, 1e-12);
}

TEST(FrictionCompensator, UsesRequestedTorqueForBreakawayDirection) {
  mca::control::FrictionCompensator compensator(make_config());
  double output = 0.0;

  ASSERT_TRUE(compensator.compute(0.0, 2.0, output).ok());
  EXPECT_NEAR(output, 0.8, 1e-12);

  ASSERT_TRUE(compensator.compute(0.0, -2.0, output).ok());
  EXPECT_NEAR(output, -0.8, 1e-12);
}

TEST(FrictionCompensator, UsesOnlyViscousTermAtExactZeroRequestedTorque) {
  mca::control::FrictionCompensator compensator(make_config());
  double output = 0.0;
  ASSERT_TRUE(compensator.compute(0.0, 0.0, output).ok());
  EXPECT_DOUBLE_EQ(output, 0.0);
}

TEST(FrictionCompensator, AppliesOutputLimits) {
  auto config = make_config();
  config.compensation_min_nm = -0.6;
  config.compensation_max_nm = 0.6;
  mca::control::FrictionCompensator compensator(config);
  double output = 0.0;

  ASSERT_TRUE(compensator.compute(3.0, 0.0, output).ok());
  EXPECT_DOUBLE_EQ(output, 0.6);

  ASSERT_TRUE(compensator.compute(-3.0, 0.0, output).ok());
  EXPECT_DOUBLE_EQ(output, -0.6);
}

TEST(FrictionCompensator, DeadbandStillIncludesViscousContribution) {
  mca::control::FrictionCompensator compensator(make_config());
  double output = 0.0;

  ASSERT_TRUE(compensator.compute(0.1, 0.0, output).ok());
  EXPECT_NEAR(output, 0.01, 1e-12);
}

}  // namespace
