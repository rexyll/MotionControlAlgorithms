#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/cia402_pds_controller.hpp"

namespace {

using namespace mca::hardware;

TEST(Cia402ControlPlan, AdvancesOnlyFromObservedPdsState) {
  Cia402ControlRequest request{};
  request.communication_healthy = true;
  request.enable_operation = true;

  auto plan = plan_cia402_control(0x0040U, request);
  EXPECT_EQ(plan.controlword, 0x0006U);
  EXPECT_FALSE(plan.motion_permitted);

  plan = plan_cia402_control(0x0021U, request);
  EXPECT_EQ(plan.controlword, 0x0007U);
  EXPECT_FALSE(plan.motion_permitted);

  plan = plan_cia402_control(0x0023U, request);
  EXPECT_EQ(plan.controlword, 0x000FU);
  EXPECT_FALSE(plan.motion_permitted);

  plan = plan_cia402_control(0x0027U, request);
  EXPECT_EQ(plan.controlword, 0x000FU);
  EXPECT_TRUE(plan.operation_enabled);
  EXPECT_TRUE(plan.motion_permitted);
}

TEST(Cia402ControlPlan, CommunicationLossDisablesOutputAndMotion) {
  Cia402ControlRequest request{};
  request.enable_operation = true;
  request.communication_healthy = false;
  const auto plan = plan_cia402_control(0x0027U, request);
  EXPECT_EQ(plan.controlword, 0x0000U);
  EXPECT_FALSE(plan.motion_permitted);
}

TEST(Cia402ControlPlan, QuickStopUsesBitTwoClearPatternAndLatches) {
  Cia402ControlRequest request{};
  request.communication_healthy = true;
  request.enable_operation = true;
  request.quick_stop = true;

  auto plan = plan_cia402_control(0x0027U, request);
  EXPECT_EQ(plan.controlword, 0x0002U);
  EXPECT_FALSE(plan.motion_permitted);

  request.quick_stop = false;
  plan = plan_cia402_control(0x0007U, request);
  EXPECT_EQ(plan.controlword, 0x0002U);
  EXPECT_FALSE(plan.motion_permitted);

  request.release_quick_stop = true;
  plan = plan_cia402_control(0x0007U, request);
  EXPECT_EQ(plan.controlword, 0x000FU);
  EXPECT_FALSE(plan.motion_permitted);
}

TEST(Cia402ControlPlan, FaultDoesNotTriggerAutomaticReset) {
  Cia402ControlRequest request{};
  request.communication_healthy = true;
  request.enable_operation = true;
  const auto plan = plan_cia402_control(0x0008U, request);
  EXPECT_EQ(plan.controlword, 0x0000U);
  EXPECT_EQ(plan.observed_state, Cia402PdsState::Fault);
  EXPECT_FALSE(plan.motion_permitted);
}

TEST(Cia402ControlPlan, DisablingOperationDoesNotRequestEnable) {
  Cia402ControlRequest request{};
  request.communication_healthy = true;
  request.enable_operation = false;
  const auto plan = plan_cia402_control(0x0027U, request);
  EXPECT_EQ(plan.controlword, 0x0007U);
  EXPECT_FALSE(plan.motion_permitted);
}

TEST(Cia402MotionInterlock, RequiresEveryIndependentSafetyCondition) {
  Cia402MotionInterlock interlock{true, true, true, true, true, true};
  EXPECT_TRUE(cia402_motion_permitted(interlock));

  interlock.working_counter_complete = false;
  EXPECT_FALSE(cia402_motion_permitted(interlock));
  interlock.working_counter_complete = true;
  interlock.requested_mode_confirmed = false;
  EXPECT_FALSE(cia402_motion_permitted(interlock));
  interlock.requested_mode_confirmed = true;
  interlock.pds_operation_enabled = false;
  EXPECT_FALSE(cia402_motion_permitted(interlock));
  interlock.pds_operation_enabled = true;
  interlock.slave_online = false;
  EXPECT_FALSE(cia402_motion_permitted(interlock));
}


TEST(Cia402FaultResetSequencer, RequiresExplicitHealthyFaultRequest) {
  Cia402FaultResetSequencer sequencer;
  EXPECT_FALSE(sequencer.begin(0x0027U, true));
  EXPECT_FALSE(sequencer.begin(0x0008U, false));
  EXPECT_EQ(sequencer.phase(), Cia402FaultResetPhase::Idle);
  EXPECT_TRUE(sequencer.begin(0x0008U, true));
  EXPECT_EQ(sequencer.phase(), Cia402FaultResetPhase::PulsePending);
}

TEST(Cia402FaultResetSequencer, EmitsOneResetPulseAndWaitsForDisabledFeedback) {
  Cia402FaultResetSequencer sequencer(4U);
  ASSERT_TRUE(sequencer.begin(0x0008U, true));

  auto plan = sequencer.update(0x0008U, true);
  EXPECT_EQ(plan.controlword, 0x0080U);
  EXPECT_TRUE(plan.pulse_sent);
  EXPECT_FALSE(plan.complete);
  EXPECT_EQ(plan.phase, Cia402FaultResetPhase::AwaitingFaultClear);

  // Holding Fault feedback must not emit another reset pulse. The reset
  // command is a one-cycle rising-edge request, not a level held until clear.
  for (std::size_t cycle = 0U; cycle < 8U; ++cycle) {
    plan = sequencer.update(0x0008U, true);
    EXPECT_EQ(plan.controlword, 0x0000U);
    EXPECT_FALSE(plan.pulse_sent);
    EXPECT_FALSE(plan.complete);
    EXPECT_FALSE(plan.failed);
  }

  plan = sequencer.update(0x000FU, true);
  EXPECT_EQ(plan.controlword, 0x0000U);
  EXPECT_EQ(plan.phase, Cia402FaultResetPhase::AwaitingFaultClear);

  plan = sequencer.update(0x0040U, true);
  EXPECT_EQ(plan.controlword, 0x0000U);
  EXPECT_TRUE(plan.complete);
  EXPECT_FALSE(plan.failed);
  EXPECT_EQ(plan.phase, Cia402FaultResetPhase::Complete);
}

TEST(Cia402FaultResetSequencer, TimesOutAndRequiresExplicitRearm) {
  Cia402FaultResetSequencer sequencer(2U);
  ASSERT_TRUE(sequencer.begin(0x0008U, true));
  ASSERT_TRUE(sequencer.update(0x0008U, true).pulse_sent);

  auto plan = sequencer.update(0x0008U, true);
  EXPECT_FALSE(plan.failed);
  plan = sequencer.update(0x0008U, true);
  EXPECT_TRUE(plan.failed);
  EXPECT_EQ(plan.phase, Cia402FaultResetPhase::Failed);
  EXPECT_FALSE(sequencer.begin(0x0008U, true));

  sequencer.reset();
  EXPECT_TRUE(sequencer.begin(0x0008U, true));
}

TEST(Cia402FaultResetSequencer, CommunicationLossAbortsWithoutResetPulse) {
  Cia402FaultResetSequencer sequencer;
  ASSERT_TRUE(sequencer.begin(0x0008U, true));

  auto plan = sequencer.update(0x0008U, false);
  EXPECT_EQ(plan.controlword, 0x0000U);
  EXPECT_FALSE(plan.pulse_sent);
  EXPECT_TRUE(plan.failed);

  sequencer.reset();
  ASSERT_TRUE(sequencer.begin(0x0008U, true));
  ASSERT_TRUE(sequencer.update(0x0008U, true).pulse_sent);
  plan = sequencer.update(0x0008U, false);
  EXPECT_EQ(plan.controlword, 0x0000U);
  EXPECT_TRUE(plan.failed);
}

TEST(Cia402FaultResetSequencer, RejectsUnexpectedStateAfterResetPulse) {
  Cia402FaultResetSequencer sequencer;
  ASSERT_TRUE(sequencer.begin(0x0008U, true));
  ASSERT_TRUE(sequencer.update(0x0008U, true).pulse_sent);
  const auto plan = sequencer.update(0x0027U, true);
  EXPECT_EQ(plan.controlword, 0x0000U);
  EXPECT_TRUE(plan.failed);
}

TEST(Cia402FaultResetSequencer, RejectsInvalidTimeout) {
  Cia402FaultResetSequencer sequencer(0U);
  EXPECT_FALSE(sequencer.configured());
  EXPECT_FALSE(sequencer.begin(0x0008U, true));
}

}  // namespace
