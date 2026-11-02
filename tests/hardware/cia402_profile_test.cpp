#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include "mca/hardware/cia402_profile.hpp"

namespace {

using namespace mca::hardware;

TEST(Cia402PdsState, DecodesAllStandardStates) {
  EXPECT_EQ(decode_cia402_pds_state(0x0000U),
            Cia402PdsState::NotReadyToSwitchOn);
  EXPECT_EQ(decode_cia402_pds_state(0x0040U),
            Cia402PdsState::SwitchOnDisabled);
  EXPECT_EQ(decode_cia402_pds_state(0x0021U),
            Cia402PdsState::ReadyToSwitchOn);
  EXPECT_EQ(decode_cia402_pds_state(0x0023U), Cia402PdsState::SwitchedOn);
  EXPECT_EQ(decode_cia402_pds_state(0x0027U),
            Cia402PdsState::OperationEnabled);
  EXPECT_EQ(decode_cia402_pds_state(0x0007U),
            Cia402PdsState::QuickStopActive);
  EXPECT_EQ(decode_cia402_pds_state(0x000FU),
            Cia402PdsState::FaultReactionActive);
  EXPECT_EQ(decode_cia402_pds_state(0x0008U), Cia402PdsState::Fault);
}

TEST(Cia402PdsState, RejectsUnknownPatternsAndDecodesStatusFlags) {
  EXPECT_EQ(decode_cia402_pds_state(0x005FU), Cia402PdsState::Unknown);
  const auto status = decode_cia402_status(0x0ED7U);
  EXPECT_EQ(status.raw, 0x0ED7U);
  EXPECT_TRUE(status.warning);
  EXPECT_TRUE(status.remote);
  EXPECT_TRUE(status.target_reached);
  EXPECT_TRUE(status.internal_limit_active);
  EXPECT_TRUE(status.voltage_enabled);
  EXPECT_TRUE(status.quick_stop);
  EXPECT_TRUE(status.switch_on_disabled);
}

TEST(Cia402Controlword, ProducesStandardActionPatterns) {
  EXPECT_EQ(cia402_controlword(Cia402ControlAction::DisableVoltage), 0x0000U);
  EXPECT_EQ(cia402_controlword(Cia402ControlAction::Shutdown), 0x0006U);
  EXPECT_EQ(cia402_controlword(Cia402ControlAction::SwitchOn), 0x0007U);
  EXPECT_EQ(cia402_controlword(Cia402ControlAction::EnableOperation), 0x000FU);
  EXPECT_EQ(cia402_controlword(Cia402ControlAction::DisableOperation), 0x0007U);
  EXPECT_EQ(cia402_controlword(Cia402ControlAction::QuickStop), 0x0002U);
  EXPECT_EQ(cia402_controlword(Cia402ControlAction::FaultReset), 0x0080U);
}

TEST(Cia402Modes, EncodesAndDecodesStandardOperationModeValues) {
  const DriveOperationMode modes[] = {
      DriveOperationMode::ProfilePosition,
      DriveOperationMode::ProfileVelocity,
      DriveOperationMode::ProfileTorque,
      DriveOperationMode::Homing,
      DriveOperationMode::CyclicSynchronousPosition,
      DriveOperationMode::CyclicSynchronousVelocity,
      DriveOperationMode::CyclicSynchronousTorque};
  const std::int8_t codes[] = {1, 3, 4, 6, 8, 9, 10};
  for (std::size_t i = 0; i < 7U; ++i) {
    std::int8_t encoded = 0;
    EXPECT_TRUE(cia402_mode_code(modes[i], encoded));
    EXPECT_EQ(encoded, codes[i]);
    DriveOperationMode decoded = DriveOperationMode::ProfilePosition;
    EXPECT_TRUE(decode_cia402_mode_code(encoded, decoded));
    EXPECT_EQ(decoded, modes[i]);
  }
  std::int8_t ignored = 0;
  EXPECT_FALSE(cia402_mode_code(static_cast<DriveOperationMode>(0xFFFFU),
                                ignored));
  DriveOperationMode decoded = DriveOperationMode::ProfilePosition;
  EXPECT_FALSE(decode_cia402_mode_code(2, decoded));
}

AxisCapabilities make_axis() {
  AxisCapabilities axis;
  axis.supported_modes = control_mode_bit(DriveControlMode::Position) |
                         control_mode_bit(DriveControlMode::Velocity);
  axis.supported_operation_modes =
      operation_mode_bit(DriveOperationMode::ProfilePosition) |
      operation_mode_bit(DriveOperationMode::CyclicSynchronousPosition);
  return axis;
}

TEST(Cia402Modes, ValidatesCapabilityAndDriveSpecificSwitchingPolicy) {
  const auto axis = make_axis();
  EXPECT_TRUE(validate_cia402_mode_selection(
                  axis, DriveOperationMode::CyclicSynchronousPosition,
                  Cia402PdsState::OperationEnabled, false).ok());
  EXPECT_EQ(validate_cia402_mode_selection(
                axis, DriveOperationMode::ProfileVelocity,
                Cia402PdsState::SwitchOnDisabled, false).code,
            mca::core::StatusCode::ConstraintViolation);
  EXPECT_EQ(validate_cia402_mode_selection(
                axis, DriveOperationMode::ProfilePosition,
                Cia402PdsState::OperationEnabled, true).code,
            mca::core::StatusCode::RuntimeError);
  EXPECT_TRUE(validate_cia402_mode_selection(
                  axis, DriveOperationMode::ProfilePosition,
                  Cia402PdsState::SwitchOnDisabled, true).ok());
  EXPECT_EQ(validate_cia402_mode_selection(
                axis, static_cast<DriveOperationMode>(0xFFFFU),
                Cia402PdsState::SwitchOnDisabled, false).code,
            mca::core::StatusCode::InvalidArgument);
}

TEST(EthercatAlState, RemainsIndependentFromCia402PdsState) {
  EXPECT_EQ(decode_ethercat_al_state(0x01U), EthercatAlState::Init);
  EXPECT_EQ(decode_ethercat_al_state(0x02U), EthercatAlState::PreOperational);
  EXPECT_EQ(decode_ethercat_al_state(0x03U), EthercatAlState::Bootstrap);
  EXPECT_EQ(decode_ethercat_al_state(0x04U), EthercatAlState::SafeOperational);
  EXPECT_EQ(decode_ethercat_al_state(0x08U), EthercatAlState::Operational);
  EXPECT_EQ(decode_ethercat_al_state(0x14U), EthercatAlState::Error);
  EXPECT_EQ(decode_ethercat_al_state(0x00U), EthercatAlState::Unknown);

  const auto pds = decode_cia402_pds_state(0x0040U);
  const auto al = decode_ethercat_al_state(0x08U);
  EXPECT_EQ(pds, Cia402PdsState::SwitchOnDisabled);
  EXPECT_EQ(al, EthercatAlState::Operational);
}

}  // namespace
