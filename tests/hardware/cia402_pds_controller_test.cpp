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

}  // namespace
