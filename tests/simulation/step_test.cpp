#include <limits>

#include <gtest/gtest.h>

#include "mca/experiment/single_axis.hpp"
#include "mca/simulation/lifecycle.hpp"
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

namespace {
class FakeSimulationParticipant final : public mca::simulation::ISimulationParticipant {
 public:
  bool reset() noexcept override {
    state_ = 0;
    expected_index_ = 0;
    return true;
  }

  bool step(const mca::simulation::SimulationStep& step) noexcept override {
    if (!mca::simulation::valid_step(step) || step.step_index != expected_index_) {
      return false;
    }
    state_ += 1;
    expected_index_ += 1;
    return true;
  }

  int state() const noexcept { return state_; }

 private:
  int state_{0};
  std::size_t expected_index_{0};
};
}  // namespace

TEST(SimulationLifecycle, ParticipantConsumesAuthoritativeExternalStep) {
  FakeSimulationParticipant participant;
  EXPECT_EQ(participant.state(), 0);
  EXPECT_TRUE(participant.reset());

  EXPECT_TRUE(participant.step({0.0, 0.01, 0}));
  EXPECT_TRUE(participant.step({0.01, 0.01, 1}));
  EXPECT_EQ(participant.state(), 2);
}

TEST(SimulationLifecycle, InvalidStepDoesNotAdvanceParticipant) {
  FakeSimulationParticipant participant;
  ASSERT_TRUE(participant.reset());
  ASSERT_TRUE(participant.step({0.0, 0.01, 0}));
  const int before = participant.state();

  EXPECT_FALSE(participant.step({0.02, 0.01, 2}));
  EXPECT_EQ(participant.state(), before);
  EXPECT_FALSE(participant.step({-0.01, 0.01, 1}));
  EXPECT_EQ(participant.state(), before);
}

TEST(SimulationLifecycle, ResetRestoresParticipantBaseline) {
  FakeSimulationParticipant participant;
  ASSERT_TRUE(participant.reset());
  ASSERT_TRUE(participant.step({0.0, 0.01, 0}));
  ASSERT_EQ(participant.state(), 1);

  ASSERT_TRUE(participant.reset());
  EXPECT_EQ(participant.state(), 0);
  EXPECT_TRUE(participant.step({0.0, 0.01, 0}));
  EXPECT_EQ(participant.state(), 1);
}

TEST(SimulationTimestep, SingleAxisUsesOneAuthoritativeDtForControllerAndPlant) {
  mca::experiment::SingleAxisExperiment experiment;
  mca::plant::SecondOrderPlant plant;
  ASSERT_TRUE(plant.configure(1.0, 0.1));

  mca::control::PidConfig pid;
  pid.kp = 2.0;
  pid.ki = 0.5;
  pid.kd = 0.0;

  mca::experiment::SingleAxisConfig config;
  config.dt = 0.01;
  config.steps = 2;
  config.reference = 1.0;
  ASSERT_TRUE(experiment.configure(config, plant, pid).ok());

  double effort0 = 0.0;
  double effort1 = 0.0;
  ASSERT_TRUE(experiment.step({0.0, 0.01, 0}, effort0).ok());
  ASSERT_TRUE(experiment.step({0.01, 0.01, 1}, effort1).ok());
  EXPECT_TRUE(std::isfinite(effort0));
  EXPECT_TRUE(std::isfinite(effort1));

  const double before = effort1;
  EXPECT_FALSE(experiment.step({0.02, 0.02, 2}, effort1).ok());
  EXPECT_DOUBLE_EQ(effort1, before);
}

TEST(SimulationTimestep, SingleAxisRejectsOutOfOrderExternalStepWithoutMutation) {
  mca::experiment::SingleAxisExperiment experiment;
  mca::plant::SecondOrderPlant plant;
  ASSERT_TRUE(plant.configure(1.0, 0.1));

  mca::control::PidConfig pid;
  pid.kp = 1.0;

  mca::experiment::SingleAxisConfig config;
  config.dt = 0.01;
  config.steps = 3;
  config.reference = 1.0;
  ASSERT_TRUE(experiment.configure(config, plant, pid).ok());

  double effort = 0.0;
  ASSERT_TRUE(experiment.step({0.0, 0.01, 0}, effort).ok());
  const double accepted_effort = effort;

  EXPECT_FALSE(experiment.step({0.02, 0.01, 2}, effort).ok());
  EXPECT_DOUBLE_EQ(effort, accepted_effort);
  EXPECT_TRUE(experiment.step({0.01, 0.01, 1}, effort).ok());
}
