#include <limits>

#include <gtest/gtest.h>

#include "mca/experiment/single_axis.hpp"
#include "mca/simulation/lifecycle.hpp"
#include "mca/simulation/replay.hpp"
#include "mca/simulation/ros2_adapter.hpp"
#include "mca/simulation/state_command.hpp"
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

TEST(SimulationStateCommand, FixedSizeValueTypesAreFiniteAndAllocationFree) {
  mca::simulation::SimulationState<double, 2> state;
  state.position = {1.0, -2.0};
  state.velocity = {0.5, -0.25};
  state.acceleration = {0.0, 3.0};
  EXPECT_TRUE(state.finite());

  state.velocity[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(state.finite());

  mca::simulation::ActuatorCommand<double, 2> command;
  command.effort = {4.0, -1.0};
  EXPECT_TRUE(command.finite());
  command.effort[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(command.finite());
}

TEST(SimulationStateCommand, ScalarAliasesExposeExplicitSingleAxisBoundary) {
  mca::simulation::ScalarSimulationState state;
  state.position[0] = 1.25;
  state.velocity[0] = 0.5;
  state.acceleration[0] = -0.1;

  mca::simulation::ScalarActuatorCommand command;
  command.effort[0] = 2.0;

  EXPECT_TRUE(state.finite());
  EXPECT_TRUE(command.finite());
  EXPECT_DOUBLE_EQ(state.position[0], 1.25);
  EXPECT_DOUBLE_EQ(command.effort[0], 2.0);
}


namespace {
class FixedReplayBuffer final
    : public mca::simulation::ISimulationLogSink<double, 1>,
      public mca::simulation::ISimulationReplaySource<double, 1> {
 public:
  using Record = mca::simulation::ScalarSimulationRecord;

  bool append(const Record& record) noexcept override {
    if (!record.valid() || size_ >= records_.size()) return false;
    if (size_ > 0 && record.step.step_index != records_[size_ - 1].step.step_index + 1) {
      return false;
    }
    records_[size_++] = record;
    return true;
  }

  bool reset() noexcept override {
    cursor_ = 0;
    return true;
  }

  bool next(Record& output) noexcept override {
    if (cursor_ >= size_) return false;
    output = records_[cursor_++];
    return true;
  }

  std::size_t size() const noexcept { return size_; }

 private:
  std::array<Record, 4> records_{};
  std::size_t size_{0};
  std::size_t cursor_{0};
};
}  // namespace

TEST(SimulationReplay, RecordValidatesStepStateAndCommandTogether) {
  mca::simulation::ScalarSimulationRecord record;
  record.step = {0.0, 0.01, 0};
  record.state.position[0] = 0.25;
  record.command.effort[0] = 1.5;
  EXPECT_TRUE(record.valid());

  record.command.effort[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(record.valid());
  record.command.effort[0] = 1.5;
  record.step.dt_s = 0.0;
  EXPECT_FALSE(record.valid());
}

TEST(SimulationReplay, SinkRejectsInvalidAndNonSequentialRecordsWithoutMutation) {
  FixedReplayBuffer buffer;
  mca::simulation::ScalarSimulationRecord first;
  first.step = {0.0, 0.01, 0};
  ASSERT_TRUE(buffer.append(first));
  EXPECT_EQ(buffer.size(), 1U);

  auto invalid = first;
  invalid.state.velocity[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(buffer.append(invalid));
  auto out_of_order = first;
  out_of_order.step = {0.02, 0.01, 2};
  EXPECT_FALSE(buffer.append(out_of_order));
  EXPECT_EQ(buffer.size(), 1U);
}

TEST(SimulationReplay, ReplayPreservesExactRecordsAndResetRestartsSequence) {
  FixedReplayBuffer buffer;
  mca::simulation::ScalarSimulationRecord first;
  first.step = {0.0, 0.01, 0};
  first.state.position[0] = 1.25;
  first.command.effort[0] = -0.5;
  auto second = first;
  second.step = {0.01, 0.01, 1};
  second.state.position[0] = 1.5;
  second.command.effort[0] = 0.75;
  ASSERT_TRUE(buffer.append(first));
  ASSERT_TRUE(buffer.append(second));
  ASSERT_TRUE(buffer.reset());

  mca::simulation::ScalarSimulationRecord output;
  ASSERT_TRUE(buffer.next(output));
  EXPECT_EQ(output.step.step_index, 0U);
  EXPECT_DOUBLE_EQ(output.state.position[0], 1.25);
  EXPECT_DOUBLE_EQ(output.command.effort[0], -0.5);
  ASSERT_TRUE(buffer.next(output));
  EXPECT_EQ(output.step.step_index, 1U);
  EXPECT_DOUBLE_EQ(output.state.position[0], 1.5);
  EXPECT_DOUBLE_EQ(output.command.effort[0], 0.75);

  const auto last = output;
  EXPECT_FALSE(buffer.next(output));
  EXPECT_EQ(output.step.step_index, last.step.step_index);
  EXPECT_DOUBLE_EQ(output.state.position[0], last.state.position[0]);

  ASSERT_TRUE(buffer.reset());
  ASSERT_TRUE(buffer.next(output));
  EXPECT_EQ(output.step.step_index, 0U);
}


namespace {
class FakeRos2SimulationAdapter final
    : public mca::simulation::IScalarRos2SimulationAdapter {
 public:
  bool publish_state(const mca::simulation::SimulationStep& step,
                     const State& state) noexcept override {
    if (!step.valid() || !state.finite()) return false;
    last_step = step;
    last_state = state;
    published = true;
    return true;
  }

  bool read_command(Command& output) noexcept override {
    if (!command_available) return false;
    output = command;
    return true;
  }

  mca::simulation::SimulationStep last_step{};
  State last_state{};
  Command command{};
  bool published{false};
  bool command_available{false};
};
}  // namespace

TEST(Ros2SimulationAdapter, BoundaryAcceptsCoreValuesWithoutRosDependencies) {
  FakeRos2SimulationAdapter adapter;
  mca::simulation::ScalarSimulationState state;
  state.position[0] = 0.125;
  ASSERT_TRUE(adapter.publish_state({0.02, 0.01, 2}, state));
  EXPECT_TRUE(adapter.published);
  EXPECT_EQ(adapter.last_step.step_index, 2U);
  EXPECT_DOUBLE_EQ(adapter.last_state.position[0], 0.125);
}

TEST(Ros2SimulationAdapter, RejectsInvalidStateAndPreservesCommandOnUnavailableRead) {
  FakeRos2SimulationAdapter adapter;
  mca::simulation::ScalarSimulationState invalid;
  invalid.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.publish_state({0.0, 0.01, 0}, invalid));
  EXPECT_FALSE(adapter.published);

  mca::simulation::ScalarActuatorCommand output;
  output.effort[0] = 4.0;
  EXPECT_FALSE(adapter.read_command(output));
  EXPECT_DOUBLE_EQ(output.effort[0], 4.0);

  adapter.command.effort[0] = -2.0;
  adapter.command_available = true;
  ASSERT_TRUE(adapter.read_command(output));
  EXPECT_DOUBLE_EQ(output.effort[0], -2.0);
}
