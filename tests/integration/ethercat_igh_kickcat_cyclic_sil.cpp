#include "mca/hardware/ethercat_igh_adapter.hpp"
#include "mca/hardware/cia402_pdo_bindings.hpp"
#include "mca/hardware/cia402_pds_controller.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <iostream>
#include <thread>

namespace {

using Adapter = mca::hardware::EthercatIghAdapter<16U, 4U, 128U>;
using Policy = mca::hardware::EthercatPdoMappingPolicy;
using mca::hardware::Cia402PdsState;

constexpr std::size_t kMaximumCycles = 5'000U;
constexpr std::size_t kStableCyclesRequired = 10U;
constexpr std::uint8_t kCspMode = 8U;

bool transport_healthy(const Adapter& adapter, std::size_t expected_slaves) {
  const auto& health = adapter.runtime_health();
  if (!health.valid() || !health.link_up ||
      health.slaves_responding != expected_slaves ||
      !adapter.domain_working_counter_complete(1U) ||
      adapter.observed_wkc() == 0U) {
    return false;
  }
  for (std::size_t i = 0; i < health.slave_count; ++i) {
    if (!health.slaves[i].configured || !health.slaves[i].online ||
        !health.slaves[i].operational) return false;
  }
  return true;
}

bool read_u16(Adapter& adapter, std::size_t index, std::uint16_t& value) {
  std::uint64_t raw = 0U;
  if (!adapter.read_pdo(index, raw).ok() || raw > 0xFFFFU) return false;
  value = static_cast<std::uint16_t>(raw);
  return true;
}

bool read_i32_bits(Adapter& adapter, std::size_t index, std::uint32_t& bits) {
  std::uint64_t raw = 0U;
  if (!adapter.read_pdo(index, raw).ok() || raw > 0xFFFFFFFFULL) return false;
  bits = static_cast<std::uint32_t>(raw);
  return true;
}

bool write_value(Adapter& adapter, std::size_t index, std::uint64_t value) {
  return adapter.write_pdo(index, value).ok();
}

bool environment_flag(const char* name) {
  const char* value = std::getenv(name);
  return value != nullptr && std::string(value) == "1";
}

std::size_t environment_size(const char* name, std::size_t fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') return fallback;
  char* end = nullptr;
  const auto parsed = std::strtoul(value, &end, 10);
  if (end == value || *end != '\0') return fallback;
  return static_cast<std::size_t>(parsed);
}

}  // namespace

int main() {
  const bool fault_sil_requested = environment_flag("MCA_ECAT_FAULT_SIL");
  Adapter::Config config{};
  config.master_index = 0U;
  config.cycle_time_ns = 1'000'000U;
  config.send_interval_us = 1'000U;
  Adapter adapter(config);

  if (!adapter.request_master().ok()) {
    std::cerr << "FAIL: request_master\n";
    return 1;
  }
  if (!adapter.discover().ok()) {
    std::cerr << "FAIL: discover\n";
    return 1;
  }

  const auto& topology = adapter.topology();
  if (topology.slave_count != 1U) {
    std::cerr << "FAIL: CiA 402 SIL requires exactly one drive slave; found "
              << topology.slave_count << '\n';
    return 1;
  }
  std::cout << "Discovered " << topology.slave_count << " slave(s)\n";
  const auto& drive = topology.slaves[0];
  std::cout << "  ring_position=" << drive.ring_position
            << " vendor=0x" << std::hex << drive.vendor_id
            << " product=0x" << drive.product_code << std::dec << '\n';

  Adapter::PdoMappingProfile mapping{};
  if (!adapter.discover_pdo_mapping(mapping).ok()) {
    std::cerr << "FAIL: discover_pdo_mapping (capacity or unsupported map)\n";
    return 1;
  }

  std::array<Adapter::PdoBinding, 128U> bindings{};
  std::size_t binding_count = 0U;
  for (std::size_t i = 0; i < mapping.size(); ++i) {
    const auto& entry = mapping[i];
    if (entry.index == 0U || entry.bit_length > 64U) continue;
    if (binding_count >= bindings.size()) {
      std::cerr << "FAIL: process-image binding capacity exceeded\n";
      return 1;
    }
    bindings[binding_count++] = {
        entry.ring_position, entry.vendor_id, entry.product_code,
        entry.index, entry.subindex, entry.bit_length, 1U, entry.direction};
  }
  if (binding_count == 0U) {
    std::cerr << "FAIL: no scalar PDO entries available for process image\n";
    return 1;
  }

  mca::hardware::Cia402PdoBindings cia402{};
  if (!mca::hardware::resolve_cia402_pdo_bindings(
          bindings, binding_count, drive.ring_position, drive.vendor_id,
          drive.product_code, cia402)) {
    std::cerr << "FAIL: discovered PDO bindings do not match typed CiA 402 profile\n";
    return 1;
  }
  std::cout << "CiA402 binding indices: controlword=" << cia402.controlword
            << " target_position=" << cia402.target_position
            << " target_velocity=" << cia402.target_velocity
            << " mode=" << cia402.mode_of_operation
            << " statusword=" << cia402.statusword
            << " actual_position=" << cia402.actual_position
            << " actual_velocity=" << cia402.actual_velocity
            << " mode_display=" << cia402.mode_display << '\n';
  std::cout << "Discovered PDO entries=" << mapping.size()
            << " scalar bindings=" << binding_count << '\n';

  if (!adapter.configure(bindings, binding_count, mapping,
                         Policy::UseDiscoveredMapping).ok()) {
    std::cerr << "FAIL: UseDiscoveredMapping configure\n";
    return 1;
  }
  if (!adapter.activate().ok()) {
    std::cerr << "FAIL: activate\n";
    return 1;
  }

  // Start with a deterministic, non-motion output image. IgH may need
  // several cycles after activation before the first complete domain WKC and
  // slave OP feedback arrive. Treat that bounded startup convergence as
  // expected; do not require the very first exchange to be healthy.
  const bool outputs_ok =
      write_value(adapter, cia402.controlword, 0U) &&
      write_value(adapter, cia402.target_position, 0U) &&
      write_value(adapter, cia402.target_velocity, 0U) &&
      write_value(adapter, cia402.mode_of_operation, kCspMode);
  if (!outputs_ok) {
    std::cerr << "FAIL: unable to initialize safe PDO output image\n";
    (void)adapter.shutdown();
    return 1;
  }

  constexpr std::size_t kStartupCyclesMaximum = 5'000U;
  bool startup_transport_ready = false;
  for (std::size_t startup_cycle = 0U;
       startup_cycle < kStartupCyclesMaximum; ++startup_cycle) {
    const auto previous_sequence = adapter.cycle_sequence();
    const auto receive_status = adapter.receive_process();
    // Refresh the safe output image after receive/process, before queue/send.
    const bool safe_outputs_ok =
        write_value(adapter, cia402.controlword, 0U) &&
        write_value(adapter, cia402.target_position, 0U) &&
        write_value(adapter, cia402.target_velocity, 0U) &&
        write_value(adapter, cia402.mode_of_operation, kCspMode);
    const auto send_status = adapter.queue_send();
    if (adapter.cycle_sequence() == previous_sequence) {
      std::cerr << "FAIL: IgH did not send startup cyclic frame\n";
      break;
    }
    if (!safe_outputs_ok) {
      std::cerr << "FAIL: unable to refresh safe startup PDO image\n";
      break;
    }
    if (transport_healthy(adapter, topology.slave_count)) {
      startup_transport_ready = true;
      std::cout << "Startup transport ready after "
                << (startup_cycle + 1U) << " cycle(s)\n";
      break;
    }
    if ((!receive_status.ok() || !send_status.ok()) &&
        adapter.domain_working_counter_complete(1U)) {
      std::cerr << "FAIL: cyclic exchange failed after domain WKC became complete\n";
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!startup_transport_ready) {
    const auto& health = adapter.runtime_health();
    std::cerr << "FAIL: initial healthy EtherCAT exchange not established"
              << " master_state_valid=" << health.master_state_valid
              << " link_up=" << health.link_up
              << " slaves_responding=" << health.slaves_responding
              << " slave_count=" << health.slave_count
              << " runtime_valid=" << health.valid()
              << " wkc=" << adapter.observed_wkc()
              << " domain_complete="
              << adapter.domain_working_counter_complete(1U);
    for (std::size_t i = 0U; i < health.slave_count; ++i) {
      const auto& slave = health.slaves[i];
      std::cerr << " slave[" << i << "] configured=" << slave.configured
                << " state_valid=" << slave.state_valid
                << " online=" << slave.online
                << " operational=" << slave.operational
                << " al_state=0x" << std::hex
                << static_cast<unsigned int>(slave.al_state) << std::dec
                << " state_error=" << slave.state_error;
    }
    std::cerr << '\n';
    (void)adapter.shutdown();
    return 1;
  }

  std::size_t enabled_stable_cycles = 0U;
  std::size_t cycles = 1U;
  std::uint16_t last_statusword = 0U;
  std::uint8_t last_mode_display = 0U;
  std::uint32_t hold_position_bits = 0U;
  bool have_hold_position = false;
  bool state_machine_passed = false;
  bool feedback_ok = true;

  for (; cycles < kMaximumCycles; ++cycles) {
    // Pace the first and subsequent state-machine exchanges at the 1 ms
    // simulator cycle. The startup convergence loop may have just completed
    // an exchange, so an immediate back-to-back frame is not deterministic.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto receive_status = adapter.receive_process();
    if (!transport_healthy(adapter, topology.slave_count)) {
      (void)adapter.queue_send();
      feedback_ok = false;
      std::cerr << "FAIL: EtherCAT health/WKC interlock lost during PDS sequence\n";
      break;
    }

    if (!read_u16(adapter, cia402.statusword, last_statusword) ||
        !read_i32_bits(adapter, cia402.actual_position, hold_position_bits)) {
      (void)adapter.queue_send();
      feedback_ok = false;
      std::cerr << "FAIL: unable to read CiA 402 status/position feedback\n";
      break;
    }
    if (!receive_status.ok()) {
      (void)adapter.queue_send();
      feedback_ok = false;
      std::cerr << "FAIL: receive/process phase reported an EtherCAT cycle error\n";
      break;
    }
    have_hold_position = true;
    const auto observed = mca::hardware::decode_cia402_pds_state(last_statusword);
    if (observed == Cia402PdsState::Fault ||
        observed == Cia402PdsState::FaultReactionActive ||
        observed == Cia402PdsState::Unknown ||
        observed == Cia402PdsState::NotReadyToSwitchOn) {
      (void)adapter.queue_send();
      feedback_ok = false;
      std::cerr << "FAIL: unexpected PDS state, statusword=0x"
                << std::hex << last_statusword << std::dec << '\n';
      break;
    }

    mca::hardware::Cia402ControlRequest cycle_request{};
    cycle_request.communication_healthy = true;
    cycle_request.enable_operation = true;
    const auto plan =
        mca::hardware::plan_cia402_control(last_statusword, cycle_request);

    std::uint64_t mode_raw = 0U;
    if (!adapter.read_pdo(cia402.mode_display, mode_raw).ok() ||
        mode_raw > 0xFFU) {
      (void)adapter.queue_send();
      feedback_ok = false;
      std::cerr << "FAIL: unable to read mode display\n";
      break;
    }
    last_mode_display = static_cast<std::uint8_t>(mode_raw);

    // Hold the current actual position throughout enable sequencing. This
    // test intentionally proves no commanded displacement.
    const bool writes_ok =
        write_value(adapter, cia402.controlword, plan.controlword) &&
        write_value(adapter, cia402.target_position, hold_position_bits) &&
        write_value(adapter, cia402.target_velocity, 0U) &&
        write_value(adapter, cia402.mode_of_operation, kCspMode);
    if (!writes_ok) {
      (void)adapter.queue_send();
      feedback_ok = false;
      std::cerr << "FAIL: PDO write failed"
                << " controlword=0x" << std::hex << plan.controlword
                << std::dec << " target_position_bits=" << hold_position_bits
                << '\n';
      break;
    }
    const auto cycle_status = adapter.queue_send();
    if (!cycle_status.ok()) {
      const auto& health = adapter.runtime_health();
      feedback_ok = false;
      std::cerr << "FAIL: cyclic exchange failed"
                << " status_code=" << static_cast<unsigned int>(cycle_status.code)
                << " master_state_valid=" << health.master_state_valid
                << " link_up=" << health.link_up
                << " slaves_responding=" << health.slaves_responding
                << " wkc=" << adapter.observed_wkc()
                << " domain_complete="
                << adapter.domain_working_counter_complete(1U);
      for (std::size_t i = 0U; i < health.slave_count; ++i) {
        const auto& slave = health.slaves[i];
        std::cerr << " slave[" << i << "] online=" << slave.online
                  << " operational=" << slave.operational
                  << " al_state=0x" << std::hex
                  << static_cast<unsigned int>(slave.al_state) << std::dec
                  << " state_error=" << slave.state_error;
      }
      std::cerr << '\n';
      break;
    }

    if (plan.operation_enabled && plan.motion_permitted &&
        last_mode_display == kCspMode) {
      ++enabled_stable_cycles;
      if (enabled_stable_cycles >= kStableCyclesRequired) {
        state_machine_passed = true;
        ++cycles;
        break;
      }
    } else {
      enabled_stable_cycles = 0U;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  bool fault_sil_passed = !fault_sil_requested;
  if (fault_sil_requested && feedback_ok && state_machine_passed) {
    bool saw_fault_reaction = false;
    bool saw_fault = false;
    std::size_t fault_reaction_cycles_seen = 0U;
    const std::size_t expected_fault_reaction_cycles =
        environment_size("MCA_ECAT_EXPECTED_FAULT_REACTION_CYCLES", 3U);

    // Maintain Operation Enabled until the simulator injects its configured
    // fault. After Fault Reaction Active is observed, only safe outputs are sent.
    for (std::size_t i = 0U; i < 2'000U; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      const auto receive_status = adapter.receive_process();
      std::uint16_t statusword = 0U;
      std::uint32_t actual_position_bits = 0U;
      if (!receive_status.ok() ||
          !transport_healthy(adapter, topology.slave_count) ||
          !read_u16(adapter, cia402.statusword, statusword) ||
          !read_i32_bits(adapter, cia402.actual_position,
                         actual_position_bits)) {
        feedback_ok = false;
        (void)adapter.queue_send();
        std::cerr << "FAIL: transport/feedback lost during injected-fault wait\n";
        break;
      }

      const auto state =
          mca::hardware::decode_cia402_pds_state(statusword);
      if (state == Cia402PdsState::FaultReactionActive) {
        saw_fault_reaction = true;
        ++fault_reaction_cycles_seen;
      } else if (state == Cia402PdsState::Fault) {
        saw_fault = true;
      } else if (saw_fault_reaction) {
        feedback_ok = false;
        (void)adapter.queue_send();
        std::cerr << "FAIL: unexpected PDS state after Fault Reaction Active, "
                  << "statusword=0x" << std::hex << statusword << std::dec
                  << '\n';
        break;
      } else if (state != Cia402PdsState::OperationEnabled) {
        feedback_ok = false;
        (void)adapter.queue_send();
        std::cerr << "FAIL: drive left Operation Enabled before injected fault, "
                  << "statusword=0x" << std::hex << statusword << std::dec
                  << '\n';
        break;
      }

      const std::uint16_t controlword =
          state == Cia402PdsState::OperationEnabled && !saw_fault_reaction
              ? 0x000FU
              : 0x0000U;
      const bool writes_ok =
          write_value(adapter, cia402.controlword, controlword) &&
          write_value(adapter, cia402.target_velocity, 0U) &&
          write_value(adapter, cia402.target_position, actual_position_bits) &&
          write_value(adapter, cia402.mode_of_operation, kCspMode);
      const auto send_status = adapter.queue_send();
      std::cout << "FAULT_TRACE cycle=" << adapter.cycle_sequence()
                << " statusword=0x" << std::hex << statusword
                << " controlword=0x" << controlword << std::dec
                << " wkc=" << adapter.observed_wkc()
                << " domain_complete="
                << adapter.domain_working_counter_complete(1U) << '\n';
      if (!writes_ok || !send_status.ok()) {
        feedback_ok = false;
        std::cerr << "FAIL: PDO send failed during injected-fault wait\n";
        break;
      }
      if (saw_fault) {
        last_statusword = statusword;
        break;
      }
    }

    if (feedback_ok && (!saw_fault_reaction || !saw_fault ||
                        fault_reaction_cycles_seen !=
                            expected_fault_reaction_cycles)) {
      feedback_ok = false;
      std::cerr << "FAIL: simulator did not expose the expected Fault Reaction Active "
                << "duration followed by Fault; reaction_cycles="
                << fault_reaction_cycles_seen
                << " expected=" << expected_fault_reaction_cycles << '\n';
    }

    if (feedback_ok && saw_fault) {
      mca::hardware::Cia402FaultResetSequencer reset_sequencer(100U);
      if (!reset_sequencer.begin(last_statusword, true)) {
        feedback_ok = false;
        std::cerr << "FAIL: explicit Fault Reset request was rejected\n";
      } else {
        // A fresh receive phase is mandatory before the reset PDO write. Pace
        // this exchange exactly like the main cyclic loop: the simulator needs
        // one 1 ms cycle to process the preceding controlword=0 frame and return
        // the next process image. An unpaced receive can sample before that
        // response is available and falsely reject a still-latched Fault.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const auto reset_receive_status = adapter.receive_process();
        std::uint16_t confirmed_fault_statusword = 0U;
        const bool reset_transport_healthy =
            transport_healthy(adapter, topology.slave_count);
        const bool reset_statusword_read =
            read_u16(adapter, cia402.statusword,
                     confirmed_fault_statusword);
        const auto confirmed_fault_state =
            mca::hardware::decode_cia402_pds_state(
                confirmed_fault_statusword);
        if (!reset_receive_status.ok() || !reset_transport_healthy ||
            !reset_statusword_read ||
            confirmed_fault_state != Cia402PdsState::Fault) {
          const auto& reset_health = adapter.runtime_health();
          feedback_ok = false;
          (void)adapter.queue_send();
          std::cerr << "FAIL: Fault state was not confirmed immediately before reset"
                    << " receive_ok=" << reset_receive_status.ok()
                    << " statusword_read=" << reset_statusword_read
                    << " statusword=0x" << std::hex
                    << confirmed_fault_statusword << std::dec
                    << " decoded_state="
                    << static_cast<unsigned int>(confirmed_fault_state)
                    << " transport_healthy=" << reset_transport_healthy
                    << " runtime_valid=" << reset_health.valid()
                    << " link_up=" << reset_health.link_up
                    << " slaves_responding=" << reset_health.slaves_responding
                    << " wkc=" << adapter.observed_wkc()
                    << " domain_complete="
                    << adapter.domain_working_counter_complete(1U)
                    << '\n';
        } else {
          last_statusword = confirmed_fault_statusword;
          const auto reset_pulse =
              reset_sequencer.update(confirmed_fault_statusword, true);
          const bool reset_pulse_sent = reset_pulse.pulse_sent &&
                                        reset_pulse.controlword == 0x0080U;
          std::size_t reset_pulse_count =
              reset_pulse.pulse_sent ? 1U : 0U;
          const auto pulse_position =
              have_hold_position ? hold_position_bits : 0U;
          const bool pulse_write_ok =
              write_value(adapter, cia402.controlword, reset_pulse.controlword) &&
              write_value(adapter, cia402.target_velocity, 0U) &&
              write_value(adapter, cia402.target_position, pulse_position) &&
              write_value(adapter, cia402.mode_of_operation, kCspMode);
          const auto pulse_send_status = adapter.queue_send();
          std::cout << "FAULT_RESET controlword=0x" << std::hex
                    << reset_pulse.controlword << std::dec
                    << " pulse_sent=" << reset_pulse.pulse_sent << '\n';

          bool reset_complete = false;
          if (!pulse_write_ok || !pulse_send_status.ok() ||
              !reset_pulse_sent) {
            feedback_ok = false;
            std::cerr << "FAIL: one-cycle Fault Reset pulse could not be sent\n";
          } else {
            for (std::size_t i = 0U; i < 20U; ++i) {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
              const auto receive_status = adapter.receive_process();
              std::uint16_t statusword = 0U;
              std::uint32_t actual_position_bits = 0U;
              if (!receive_status.ok() ||
                  !transport_healthy(adapter, topology.slave_count) ||
                  !read_u16(adapter, cia402.statusword, statusword) ||
                  !read_i32_bits(adapter, cia402.actual_position,
                                 actual_position_bits)) {
                feedback_ok = false;
                (void)adapter.queue_send();
                std::cerr << "FAIL: transport/feedback lost while awaiting reset feedback\n";
                break;
              }
              const auto plan = reset_sequencer.update(statusword, true);
              if (plan.pulse_sent) ++reset_pulse_count;
              const bool writes_ok =
                  write_value(adapter, cia402.controlword, plan.controlword) &&
                  write_value(adapter, cia402.target_velocity, 0U) &&
                  write_value(adapter, cia402.target_position,
                              actual_position_bits) &&
                  write_value(adapter, cia402.mode_of_operation, kCspMode);
              const auto send_status = adapter.queue_send();
              std::cout << "FAULT_RESET_TRACE cycle=" << adapter.cycle_sequence()
                        << " statusword=0x" << std::hex << statusword
                        << " controlword=0x" << plan.controlword << std::dec
                        << " phase=" << static_cast<unsigned int>(plan.phase)
                        << " wkc=" << adapter.observed_wkc() << '\n';
              if (!writes_ok || !send_status.ok() || plan.failed) {
                feedback_ok = false;
                std::cerr << "FAIL: Fault Reset sequence failed\n";
                break;
              }
              if (plan.complete) {
                reset_complete =
                    mca::hardware::decode_cia402_pds_state(statusword) ==
                    Cia402PdsState::SwitchOnDisabled;
                last_statusword = statusword;
                break;
              }
            }
          }
          fault_sil_passed = feedback_ok && reset_pulse_count == 1U &&
                             reset_complete;
          if (!fault_sil_passed) {
            feedback_ok = false;
            std::cerr << "FAIL: reset did not produce exactly one 0x0080 pulse "
                         "and Switch On Disabled feedback; pulse_count="
                      << reset_pulse_count << '\n';
          } else {
            std::cout << "PASS: wire-level CiA402 Fault Reaction Active -> Fault "
                         "-> explicit one-cycle Fault Reset -> Switch On Disabled; "
                         "drive was not re-enabled.\n";
          }
        }
      }
    }
  } else if (fault_sil_requested) {
    feedback_ok = false;
    std::cerr << "FAIL: fault SIL requires baseline Operation Enabled acceptance\n";
  }

  // Remove voltage with receive -> output update -> queue/send ordering.
  bool disabled_feedback = false;
  for (std::size_t i = 0; i < 100U; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto receive_status = adapter.receive_process();
    const bool health_ok = transport_healthy(adapter, topology.slave_count);
    std::uint16_t statusword = 0U;
    const bool status_ok = read_u16(adapter, cia402.statusword, statusword);
    const bool disable_written =
        write_value(adapter, cia402.controlword, 0U) &&
        write_value(adapter, cia402.target_velocity, 0U) &&
        (have_hold_position
             ? write_value(adapter, cia402.target_position, hold_position_bits)
             : write_value(adapter, cia402.target_position, 0U));
    const auto send_status = adapter.queue_send();
    if (!health_ok || !status_ok || !disable_written ||
        !receive_status.ok() || !send_status.ok()) break;
    if (mca::hardware::decode_cia402_pds_state(statusword) ==
        Cia402PdsState::SwitchOnDisabled) {
      disabled_feedback = true;
      break;
    }
  }

  const auto final_health = adapter.runtime_health();
  const bool transport_passed = transport_healthy(adapter, topology.slave_count);
  std::cout << "cycles=" << adapter.cycle_sequence()
            << " enabled_stable_cycles=" << enabled_stable_cycles
            << " statusword=0x" << std::hex << last_statusword
            << " mode_display=0x" << static_cast<unsigned int>(last_mode_display)
            << std::dec
            << " wkc=" << adapter.observed_wkc()
            << " domain_complete=" << adapter.domain_working_counter_complete(1U)
            << " disabled_feedback=" << disabled_feedback << '\n';
  for (std::size_t i = 0; i < final_health.slave_count; ++i) {
    const auto& slave = final_health.slaves[i];
    std::cout << "slave ring_position=" << slave.ring_position
              << " online=" << slave.online
              << " operational=" << slave.operational
              << " al_state=0x" << std::hex
              << static_cast<unsigned int>(slave.al_state) << std::dec << '\n';
  }

  const auto shutdown_status = adapter.shutdown();
  if (!shutdown_status.ok()) {
    std::cerr << "FAIL: shutdown\n";
    return 1;
  }
  if (!feedback_ok || !state_machine_passed || !disabled_feedback ||
      !transport_passed || !fault_sil_passed) {
    std::cerr << "FAIL: CiA 402 state-machine closed-loop acceptance not reached\n";
    return 1;
  }
  std::cout << "PASS: EtherCAT transport healthy; CiA 402 reached Operation Enabled "
               "from statusword feedback in CSP mode, held position unchanged, "
               "then returned to Switch On Disabled.\n";
  if (fault_sil_requested) {
    std::cout << "NOTE: Injected fault validates the patched KickCAT model only; "
                 "vendor fault codes, emergency objects, STO and HIL safety remain unvalidated.\n";
  }
  std::cout << "NOTE: KickCAT echoes mode display; this does not validate a real "
               "drive's mode acceptance, motion response, DC timing or HIL safety.\n";
  return 0;
}
