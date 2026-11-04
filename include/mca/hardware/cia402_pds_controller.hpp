#pragma once

#include <cstdint>

#include "mca/hardware/cia402_profile.hpp"

namespace mca::hardware {

// Inputs are sampled from the latest completed EtherCAT process-data cycle.
// A caller must not pass cached "healthy" values after an exchange failure.
struct Cia402ControlRequest {
  bool communication_healthy{false};
  bool enable_operation{false};
  bool quick_stop{false};
  // Releasing Quick Stop is deliberately a separate, explicit request so a
  // recovered bus cannot silently restart a drive.
  bool release_quick_stop{false};
};

struct Cia402ControlPlan {
  std::uint16_t controlword{0x0000U};
  Cia402PdsState observed_state{Cia402PdsState::Unknown};
  bool operation_enabled{false};
  bool motion_permitted{false};
};

// Plans one controlword from observed statusword feedback. This is not a
// replacement for a vendor drive profile: option-code behavior and additional
// interlocks remain device-specific. Fault reset is intentionally never
// automatic; a separate, deliberate reset workflow must be implemented by
// the application after fault diagnostics.
constexpr Cia402ControlPlan plan_cia402_control(
    std::uint16_t statusword, const Cia402ControlRequest& request) noexcept {
  const auto state = decode_cia402_pds_state(statusword);
  Cia402ControlPlan plan{};
  plan.observed_state = state;

  if (!request.communication_healthy) return plan;

  if (request.quick_stop) {
    if (state == Cia402PdsState::OperationEnabled ||
        state == Cia402PdsState::QuickStopActive) {
      plan.controlword = cia402_controlword(Cia402ControlAction::QuickStop);
    }
    return plan;
  }

  switch (state) {
    case Cia402PdsState::SwitchOnDisabled:
      if (request.enable_operation)
        plan.controlword = cia402_controlword(Cia402ControlAction::Shutdown);
      break;
    case Cia402PdsState::ReadyToSwitchOn:
      if (request.enable_operation)
        plan.controlword = cia402_controlword(Cia402ControlAction::SwitchOn);
      break;
    case Cia402PdsState::SwitchedOn:
      if (request.enable_operation)
        plan.controlword = cia402_controlword(Cia402ControlAction::EnableOperation);
      break;
    case Cia402PdsState::OperationEnabled:
      if (request.enable_operation) {
        plan.controlword = cia402_controlword(Cia402ControlAction::EnableOperation);
        plan.operation_enabled = true;
        plan.motion_permitted = true;
      } else {
        plan.controlword = cia402_controlword(Cia402ControlAction::DisableOperation);
      }
      break;
    case Cia402PdsState::QuickStopActive:
      if (request.release_quick_stop && request.enable_operation) {
        plan.controlword = cia402_controlword(Cia402ControlAction::EnableOperation);
      } else {
        plan.controlword = cia402_controlword(Cia402ControlAction::QuickStop);
      }
      break;
    case Cia402PdsState::NotReadyToSwitchOn:
    case Cia402PdsState::FaultReactionActive:
    case Cia402PdsState::Fault:
    case Cia402PdsState::Unknown:
      // Keep voltage disabled. Never attempt an implicit fault reset.
      break;
  }
  return plan;
}


enum class Cia402FaultResetPhase : std::uint8_t {
  Idle,
  PulsePending,
  AwaitingFaultClear,
  Complete,
  Failed
};

struct Cia402FaultResetPlan {
  std::uint16_t controlword{0U};
  Cia402FaultResetPhase phase{Cia402FaultResetPhase::Idle};
  bool pulse_sent{false};
  bool complete{false};
  bool failed{false};
};

// Explicit, bounded Fault Reset workflow. The application must route the
// sequencer's controlword to the drive for every step; it emits 0x0080 for
// exactly one cycle, then clears bit 7 and waits for Switch On Disabled
// feedback. It never enables the drive after a successful reset.
class Cia402FaultResetSequencer final {
 public:
  explicit constexpr Cia402FaultResetSequencer(
      std::uint32_t feedback_timeout_cycles = 100U) noexcept
      : feedback_timeout_cycles_(feedback_timeout_cycles) {}

  [[nodiscard]] constexpr bool configured() const noexcept {
    return feedback_timeout_cycles_ > 0U;
  }

  [[nodiscard]] constexpr Cia402FaultResetPhase phase() const noexcept {
    return phase_;
  }

  // Reset can only be explicitly armed from an observed Fault state while
  // communication is healthy. Failed workflows require reset() before retry.
  constexpr bool begin(std::uint16_t statusword,
                       bool communication_healthy) noexcept {
    if (!configured() || !communication_healthy ||
        decode_cia402_pds_state(statusword) != Cia402PdsState::Fault ||
        (phase_ != Cia402FaultResetPhase::Idle &&
         phase_ != Cia402FaultResetPhase::Complete)) {
      return false;
    }
    phase_ = Cia402FaultResetPhase::PulsePending;
    waited_cycles_ = 0U;
    return true;
  }

  constexpr Cia402FaultResetPlan update(
      std::uint16_t statusword, bool communication_healthy) noexcept {
    Cia402FaultResetPlan plan{};
    if (phase_ == Cia402FaultResetPhase::PulsePending) {
      if (!communication_healthy ||
          decode_cia402_pds_state(statusword) != Cia402PdsState::Fault) {
        phase_ = Cia402FaultResetPhase::Failed;
      } else {
        plan.controlword = cia402_controlword(Cia402ControlAction::FaultReset);
        plan.pulse_sent = true;
        phase_ = Cia402FaultResetPhase::AwaitingFaultClear;
      }
    } else if (phase_ == Cia402FaultResetPhase::AwaitingFaultClear) {
      if (!communication_healthy) {
        phase_ = Cia402FaultResetPhase::Failed;
      } else {
        const auto state = decode_cia402_pds_state(statusword);
        if (state == Cia402PdsState::SwitchOnDisabled) {
          phase_ = Cia402FaultResetPhase::Complete;
        } else if (state != Cia402PdsState::Fault &&
                   state != Cia402PdsState::FaultReactionActive) {
          phase_ = Cia402FaultResetPhase::Failed;
        } else {
          ++waited_cycles_;
          if (waited_cycles_ >= feedback_timeout_cycles_) {
            phase_ = Cia402FaultResetPhase::Failed;
          }
        }
      }
    }
    plan.phase = phase_;
    plan.complete = phase_ == Cia402FaultResetPhase::Complete;
    plan.failed = phase_ == Cia402FaultResetPhase::Failed;
    return plan;
  }

  constexpr void reset() noexcept {
    phase_ = Cia402FaultResetPhase::Idle;
    waited_cycles_ = 0U;
  }

 private:
  std::uint32_t feedback_timeout_cycles_{100U};
  std::uint32_t waited_cycles_{0U};
  Cia402FaultResetPhase phase_{Cia402FaultResetPhase::Idle};
};

struct Cia402MotionInterlock {
  bool ethercat_operational{false};
  bool link_up{false};
  bool slave_online{false};
  bool working_counter_complete{false};
  bool pds_operation_enabled{false};
  bool requested_mode_confirmed{false};
};

// Motion setpoints are valid only when every independent transport, topology,
// PDS and mode-feedback condition is true in the same cycle.
constexpr bool cia402_motion_permitted(
    const Cia402MotionInterlock& interlock) noexcept {
  return interlock.ethercat_operational && interlock.link_up &&
         interlock.slave_online && interlock.working_counter_complete &&
         interlock.pds_operation_enabled &&
         interlock.requested_mode_confirmed;
}

}  // namespace mca::hardware
