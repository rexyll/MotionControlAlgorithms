#pragma once

#include <cstdint>

#include "mca/core/types.hpp"
#include "mca/hardware/device_metadata.hpp"

namespace mca::hardware {

// CiA 402 Power Drive System (PDS) state. This is deliberately independent
// from the EtherCAT Application Layer (AL) state below.
enum class Cia402PdsState : std::uint8_t {
  Unknown,
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault
};

struct Cia402Status {
  std::uint16_t raw{0};
  Cia402PdsState state{Cia402PdsState::Unknown};
  bool warning{false};
  bool remote{false};
  bool target_reached{false};
  bool internal_limit_active{false};
  bool voltage_enabled{false};
  bool quick_stop_not_active{false};
  bool switch_on_disabled{false};
};

constexpr Cia402PdsState decode_cia402_pds_state(
    std::uint16_t statusword) noexcept {
  // CiA 402 uses different masks for state patterns. Keep the mask paired
  // with each value; a single broad mask can alias valid states.
  if ((statusword & 0x004FU) == 0x0000U)
    return Cia402PdsState::NotReadyToSwitchOn;
  if ((statusword & 0x004FU) == 0x0040U)
    return Cia402PdsState::SwitchOnDisabled;
  if ((statusword & 0x006FU) == 0x0021U)
    return Cia402PdsState::ReadyToSwitchOn;
  if ((statusword & 0x006FU) == 0x0023U)
    return Cia402PdsState::SwitchedOn;
  if ((statusword & 0x006FU) == 0x0027U)
    return Cia402PdsState::OperationEnabled;
  if ((statusword & 0x006FU) == 0x0007U)
    return Cia402PdsState::QuickStopActive;
  if ((statusword & 0x004FU) == 0x000FU)
    return Cia402PdsState::FaultReactionActive;
  if ((statusword & 0x004FU) == 0x0008U)
    return Cia402PdsState::Fault;
  return Cia402PdsState::Unknown;
}

constexpr Cia402Status decode_cia402_status(
    std::uint16_t statusword) noexcept {
  return {statusword,
          decode_cia402_pds_state(statusword),
          (statusword & 0x0080U) != 0U,
          (statusword & 0x0200U) != 0U,
          (statusword & 0x0400U) != 0U,
          (statusword & 0x0800U) != 0U,
          (statusword & 0x0010U) != 0U,
          (statusword & 0x0020U) != 0U,
          (statusword & 0x0040U) != 0U};
}

enum class Cia402ControlAction : std::uint8_t {
  DisableVoltage,
  Shutdown,
  SwitchOn,
  EnableOperation,
  DisableOperation,
  QuickStop,
  FaultReset
};

// These are standard controlword patterns, not a state-transition executor.
// The caller must observe statusword feedback and handle manufacturer-specific
// interlocks before issuing the next action.
constexpr std::uint16_t cia402_controlword(
    Cia402ControlAction action) noexcept {
  switch (action) {
    case Cia402ControlAction::DisableVoltage: return 0x0000U;
    case Cia402ControlAction::Shutdown: return 0x0006U;
    case Cia402ControlAction::SwitchOn: return 0x0007U;
    case Cia402ControlAction::EnableOperation: return 0x000FU;
    case Cia402ControlAction::DisableOperation: return 0x0007U;
    case Cia402ControlAction::QuickStop: return 0x0002U;
    case Cia402ControlAction::FaultReset: return 0x0080U;
  }
  return 0U;
}

// State-aware guard for common PDS transitions. Device-specific interlocks,
// quick-stop option codes and fault-reset timing remain profile responsibilities.
constexpr bool cia402_control_action_allowed(
    Cia402PdsState state, Cia402ControlAction action) noexcept {
  switch (action) {
    case Cia402ControlAction::DisableVoltage:
      return state != Cia402PdsState::Unknown;
    case Cia402ControlAction::Shutdown:
      return state == Cia402PdsState::SwitchOnDisabled ||
             state == Cia402PdsState::ReadyToSwitchOn ||
             state == Cia402PdsState::SwitchedOn ||
             state == Cia402PdsState::OperationEnabled ||
             state == Cia402PdsState::QuickStopActive;
    case Cia402ControlAction::SwitchOn:
      return state == Cia402PdsState::ReadyToSwitchOn;
    case Cia402ControlAction::EnableOperation:
      return state == Cia402PdsState::SwitchedOn;
    case Cia402ControlAction::DisableOperation:
      return state == Cia402PdsState::OperationEnabled;
    case Cia402ControlAction::QuickStop:
      return state == Cia402PdsState::OperationEnabled;
    case Cia402ControlAction::FaultReset:
      return state == Cia402PdsState::Fault;
  }
  return false;
}

// Standard CiA 402 Modes of Operation values (object 0x6060/0x6061).
constexpr bool cia402_mode_code(DriveOperationMode mode,
                                std::int8_t& code) noexcept {
  switch (mode) {
    case DriveOperationMode::ProfilePosition: code = 1; return true;
    case DriveOperationMode::ProfileVelocity: code = 3; return true;
    case DriveOperationMode::ProfileTorque: code = 4; return true;
    case DriveOperationMode::Homing: code = 6; return true;
    case DriveOperationMode::CyclicSynchronousPosition: code = 8; return true;
    case DriveOperationMode::CyclicSynchronousVelocity: code = 9; return true;
    case DriveOperationMode::CyclicSynchronousTorque: code = 10; return true;
  }
  return false;
}

constexpr bool decode_cia402_mode_code(std::int8_t code,
                                       DriveOperationMode& mode) noexcept {
  switch (code) {
    case 1: mode = DriveOperationMode::ProfilePosition; return true;
    case 3: mode = DriveOperationMode::ProfileVelocity; return true;
    case 4: mode = DriveOperationMode::ProfileTorque; return true;
    case 6: mode = DriveOperationMode::Homing; return true;
    case 8: mode = DriveOperationMode::CyclicSynchronousPosition; return true;
    case 9: mode = DriveOperationMode::CyclicSynchronousVelocity; return true;
    case 10: mode = DriveOperationMode::CyclicSynchronousTorque; return true;
    default: return false;
  }
}

// Mode changes are validated against per-axis discovery. Some drives require
// Switch On Disabled for mode changes; others permit selected changes while
// enabled. That policy is supplied by the model/profile, never guessed here.
inline core::Status validate_cia402_mode_selection(
    const AxisCapabilities& axis, DriveOperationMode requested,
    Cia402PdsState current_state,
    bool mode_change_requires_switch_on_disabled) noexcept {
  std::int8_t ignored = 0;
  if (!axis.valid() || !cia402_mode_code(requested, ignored)) {
    return {core::StatusCode::InvalidArgument};
  }
  if (!axis.supports(requested)) {
    return {core::StatusCode::ConstraintViolation};
  }
  if (mode_change_requires_switch_on_disabled &&
      current_state != Cia402PdsState::SwitchOnDisabled) {
    return {core::StatusCode::RuntimeError};
  }
  return {};
}

// EtherCAT AL state is a separate state machine from CiA 402 PDS.
enum class EthercatAlState : std::uint8_t {
  Unknown,
  Init,
  PreOperational,
  Bootstrap,
  SafeOperational,
  Operational,
  Error
};

constexpr EthercatAlState decode_ethercat_al_state(
    std::uint8_t al_status) noexcept {
  if ((al_status & 0x10U) != 0U) return EthercatAlState::Error;
  switch (al_status & 0x0FU) {
    case 0x01U: return EthercatAlState::Init;
    case 0x02U: return EthercatAlState::PreOperational;
    case 0x03U: return EthercatAlState::Bootstrap;
    case 0x04U: return EthercatAlState::SafeOperational;
    case 0x08U: return EthercatAlState::Operational;
    default: return EthercatAlState::Unknown;
  }
}

}  // namespace mca::hardware
