#include <limits>

#include <gtest/gtest.h>

#include "mca/simulation/step.hpp"

TEST(SimulationStep, ValidatesFiniteNonNegativeTimeAndPositiveDt) {
  EXPECT_TRUE((mca::simulation::SimulationStep{0.0, 0.01, 0}).valid());
  EXPECT_TRUE((mca::simulation::SimulationStep{1.0, 0.1, 42}).valid());
  EXPECT_FALSE((mca::simulation::SimulationStep{-0.1, 0.01, 0}).valid());
  EXPECT_FALSE((mca::simulation::SimulationStep{0.0, 0.0, 0}).valid());
  EXPECT_FALSE((mca::simulation::SimulationStep{0.0, -0.1, 0}).valid());
  EXPECT_FALSE((mca::simulation::SimulationStep{std::numeric_limits<double>::infinity(), 0.1, 0}).valid());
}

TEST(SimulationClock, AdvancesDeterministicallyFromConfiguredStep) {
  mca::simulation::SimulationClock clock;
  EXPECT_FALSE(clock.advance());
  ASSERT_TRUE(clock.configure(0.01));
  ASSERT_TRUE(clock.reset());

  const auto initial = clock.current_step();
  EXPECT_DOUBLE_EQ(initial.time_s, 0.0);
  EXPECT_DOUBLE_EQ(initial.dt_s, 0.01);
  EXPECT_EQ(initial.step_index, 0U);

  ASSERT_TRUE(clock.advance());
  const auto next = clock.current_step();
  EXPECT_DOUBLE_EQ(next.time_s, 0.01);
  EXPECT_DOUBLE_EQ(next.dt_s, 0.01);
  EXPECT_EQ(next.step_index, 1U);
}

TEST(SimulationClock, InvalidConfigurationPreservesExistingClock) {
  mca::simulation::SimulationClock clock;
  ASSERT_TRUE(clock.configure(0.02));
  ASSERT_TRUE(clock.advance());

  EXPECT_FALSE(clock.configure(0.0));
  EXPECT_DOUBLE_EQ(clock.dt(), 0.02);
  EXPECT_DOUBLE_EQ(clock.time(), 0.02);
  EXPECT_EQ(clock.step_index(), 1U);

  EXPECT_FALSE(clock.configure(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_DOUBLE_EQ(clock.dt(), 0.02);
  EXPECT_DOUBLE_EQ(clock.time(), 0.02);
  EXPECT_EQ(clock.step_index(), 1U);
}

TEST(SimulationClock, ResetRestoresDeterministicBaseline) {
  mca::simulation::SimulationClock clock;
  ASSERT_TRUE(clock.configure(0.05));
  ASSERT_TRUE(clock.advance());
  ASSERT_TRUE(clock.advance());
  ASSERT_TRUE(clock.reset());

  EXPECT_DOUBLE_EQ(clock.time(), 0.0);
  EXPECT_EQ(clock.step_index(), 0U);
  EXPECT_DOUBLE_EQ(clock.current_step().dt_s, 0.05);
}

TEST(SimulationClock, OverflowingAdvancePreservesState) {
  mca::simulation::SimulationClock clock;
  ASSERT_TRUE(clock.configure(std::numeric_limits<double>::max()));
  ASSERT_TRUE(clock.advance());
  const auto before = clock.current_step();

  EXPECT_FALSE(clock.advance());
  const auto after = clock.current_step();
  EXPECT_DOUBLE_EQ(after.time_s, before.time_s);
  EXPECT_DOUBLE_EQ(after.dt_s, before.dt_s);
  EXPECT_EQ(after.step_index, before.step_index);
}
