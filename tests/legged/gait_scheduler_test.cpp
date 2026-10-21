#include "mca/legged/gait_scheduler.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace {
using mca::legged::GaitSchedulerOutput;
using mca::legged::GaitSchedulerParameters;
using mca::legged::evaluate_gait;

TEST(GaitScheduler, DefaultOffsetsProduceDiagonalPhasePairs) {
  GaitSchedulerParameters p;
  GaitSchedulerOutput output;
  // Sample after the cycle starts: with a 0.6 duty factor, phase 0.0 and
  // phase 0.5 are both in stance, so t=0 cannot distinguish the diagonal pair.
  ASSERT_TRUE(evaluate_gait(p, 0.075, output));
  ASSERT_EQ(output.legs.size(), 4U);
  EXPECT_TRUE(output.legs[0].in_stance);
  EXPECT_FALSE(output.legs[1].in_stance);
  EXPECT_FALSE(output.legs[2].in_stance);
  EXPECT_TRUE(output.legs[3].in_stance);
  EXPECT_NEAR(output.legs[0].cycle_phase, 0.15, 1e-12);
  EXPECT_NEAR(output.legs[1].cycle_phase, 0.65, 1e-12);
}

TEST(GaitScheduler, PhaseWrapsForPositiveAndNegativeTime) {
  GaitSchedulerParameters p;
  p.period_s = 2.0;
  p.duty_factor = 0.5;
  p.phase_offsets = {0.0};
  GaitSchedulerOutput output;
  ASSERT_TRUE(evaluate_gait(p, -0.5, output));
  EXPECT_NEAR(output.legs[0].cycle_phase, 0.75, 1e-12);
  EXPECT_FALSE(output.legs[0].in_stance);
  ASSERT_TRUE(evaluate_gait(p, 4.5, output));
  EXPECT_NEAR(output.legs[0].cycle_phase, 0.25, 1e-12);
  EXPECT_TRUE(output.legs[0].in_stance);
}

TEST(GaitScheduler, ComputesNormalizedStanceAndSwingProgress) {
  GaitSchedulerParameters p;
  p.period_s = 1.0;
  p.duty_factor = 0.6;
  p.phase_offsets = {0.3, 0.8};
  GaitSchedulerOutput output;
  ASSERT_TRUE(evaluate_gait(p, 0.0, output));
  ASSERT_EQ(output.legs.size(), 2U);
  EXPECT_TRUE(output.legs[0].in_stance);
  EXPECT_NEAR(output.legs[0].normalized_phase, 0.5, 1e-12);
  EXPECT_FALSE(output.legs[1].in_stance);
  EXPECT_NEAR(output.legs[1].normalized_phase, 0.5, 1e-12);
}

TEST(GaitScheduler, InvalidInputsPreservePreviousOutput) {
  GaitSchedulerParameters p;
  GaitSchedulerOutput output;
  ASSERT_TRUE(evaluate_gait(p, 0.1, output));
  const auto original = output.legs;
  std::string error;
  p.duty_factor = 1.0;
  EXPECT_FALSE(evaluate_gait(p, 0.1, output, &error));
  EXPECT_EQ(output.legs.size(), original.size());
  EXPECT_DOUBLE_EQ(output.legs[0].cycle_phase, original[0].cycle_phase);
  p.duty_factor = 0.6;
  p.phase_offsets = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(evaluate_gait(p, 0.1, output, &error));
  p.phase_offsets.clear();
  EXPECT_FALSE(evaluate_gait(p, 0.1, output, &error));
  EXPECT_EQ(output.legs.size(), original.size());
}

TEST(GaitScheduler, ExactStanceBoundaryStartsSwing) {
  GaitSchedulerParameters p;
  p.period_s = 1.0;
  p.duty_factor = 0.5;
  p.phase_offsets = {0.0};
  GaitSchedulerOutput output;
  ASSERT_TRUE(evaluate_gait(p, 0.5, output));
  EXPECT_FALSE(output.legs[0].in_stance);
  EXPECT_DOUBLE_EQ(output.legs[0].normalized_phase, 0.0);
  ASSERT_TRUE(evaluate_gait(p, 1.0, output));
  EXPECT_TRUE(output.legs[0].in_stance);
}
}  // namespace
