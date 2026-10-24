#include <gtest/gtest.h>

#include <limits>

#include "mca/plant/bldc_pmsm.hpp"

TEST(BldcPmsm, RejectsInvalidConfiguration) {
  mca::plant::BldcPmsm motor;
  mca::plant::BldcPmsmConfig config;
  config.pole_pairs = 0;
  EXPECT_FALSE(motor.configure(config).ok());
  EXPECT_FALSE(motor.configured());
}

TEST(BldcPmsm, AppliesThreePhaseVoltageAndProducesCurrent) {
  mca::plant::BldcPmsmConfig config;
  config.phase_resistance_ohm = 1.0;
  config.phase_inductance_h = 0.01;
  config.inertia_kg_m2 = 1.0;
  config.viscous_damping_nm_per_rad_s = 1.0;
  mca::plant::BldcPmsm motor(config);
  ASSERT_TRUE(motor.configured());

  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(motor.step(1.0, -1.0, 0.0, 0.0, 0.0001).ok());
  }

  EXPECT_GT(motor.current_a(), 0.0);
  EXPECT_LT(motor.current_b(), 0.0);
  // Phase-C is initially unexcited, but its current evolves from the
  // electrical state/back-EMF coupling; assert the physically relevant
  // small-current bound rather than exact zero.
  EXPECT_NEAR(motor.current_c(), 0.0, 1e-4);
  EXPECT_GT(motor.speed(), 0.0);
}

TEST(BldcPmsm, ElectricalAngleUsesPolePairCount) {
  mca::plant::BldcPmsmConfig config;
  config.pole_pairs = 5;
  mca::plant::BldcPmsm motor(config);
  ASSERT_TRUE(motor.reset(0.2, 0.0).ok());
  EXPECT_NEAR(motor.electrical_angle(), 1.0, 1e-12);
}

TEST(BldcPmsm, RejectsNonFiniteInputAndInvalidStep) {
  mca::plant::BldcPmsm motor(mca::plant::BldcPmsmConfig{});
  EXPECT_FALSE(motor.step(0.0, 0.0, 0.0, 0.0, 0.0).ok());
  EXPECT_FALSE(motor.step(0.0, 0.0, 0.0, 0.0,
                          std::numeric_limits<double>::quiet_NaN()).ok());
}

TEST(BldcPmsm, InvalidReconfigurationPreservesWorkingState) {
  mca::plant::BldcPmsm motor(mca::plant::BldcPmsmConfig{});
  ASSERT_TRUE(motor.reset(0.25, 0.5, 0.1, -0.2, 0.3).ok());
  const auto before_config = motor.config();
  mca::plant::BldcPmsmConfig invalid = before_config;
  invalid.pole_pairs = 0;

  EXPECT_EQ(motor.configure(invalid).code, mca::core::StatusCode::InvalidArgument);
  EXPECT_TRUE(motor.configured());
  EXPECT_EQ(motor.config().pole_pairs, before_config.pole_pairs);
  EXPECT_DOUBLE_EQ(motor.mechanical_angle(), 0.25);
  EXPECT_DOUBLE_EQ(motor.speed(), 0.5);
  EXPECT_DOUBLE_EQ(motor.current_a(), 0.1);
  EXPECT_DOUBLE_EQ(motor.current_b(), -0.2);
  EXPECT_DOUBLE_EQ(motor.current_c(), 0.3);
  EXPECT_TRUE(motor.step(0.0, 0.0, 0.0, 0.0, 0.001).ok());
}


TEST(BldcPmsm, ResetDerivedOverflowPreservesState) {
  mca::plant::BldcPmsm motor(mca::plant::BldcPmsmConfig{});
  ASSERT_TRUE(motor.reset(0.25, 0.5, 0.1, -0.2, 0.3).ok());

  const auto status = motor.reset(std::numeric_limits<double>::max(), 0.5,
                                  0.1, -0.2, 0.3);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(motor.mechanical_angle(), 0.25);
  EXPECT_DOUBLE_EQ(motor.speed(), 0.5);
  EXPECT_DOUBLE_EQ(motor.current_a(), 0.1);
  EXPECT_DOUBLE_EQ(motor.current_b(), -0.2);
  EXPECT_DOUBLE_EQ(motor.current_c(), 0.3);
  EXPECT_TRUE(motor.step(0.0, 0.0, 0.0, 0.0, 0.001).ok());
}

TEST(BldcPmsm, NumericalOverflowDoesNotCommitState) {
  mca::plant::BldcPmsmConfig config;
  config.phase_inductance_h = 1e-308;
  mca::plant::BldcPmsm motor(config);
  ASSERT_TRUE(motor.reset(0.25, 0.0, 0.0, 0.0, 0.0).ok());

  const auto status = motor.step(std::numeric_limits<double>::max(), 0.0,
                                 0.0, 0.0, 1.0);
  EXPECT_EQ(status.code, mca::core::StatusCode::NumericalError);
  EXPECT_DOUBLE_EQ(motor.mechanical_angle(), 0.25);
  EXPECT_DOUBLE_EQ(motor.speed(), 0.0);
  EXPECT_DOUBLE_EQ(motor.current_a(), 0.0);
  EXPECT_DOUBLE_EQ(motor.current_b(), 0.0);
  EXPECT_DOUBLE_EQ(motor.current_c(), 0.0);
  EXPECT_TRUE(motor.step(0.0, 0.0, 0.0, 0.0, 0.001).ok());
}
