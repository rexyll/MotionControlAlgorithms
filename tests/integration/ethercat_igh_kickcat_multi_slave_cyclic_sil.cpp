#include "mca/hardware/ethercat_igh_adapter.hpp"
#include "mca/hardware/cia402_pdo_bindings.hpp"
#include "mca/hardware/cia402_pds_controller.hpp"
#include "mca/hardware/ethercat_igh_health_monitor.hpp"

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

constexpr std::size_t kDriveCount = 2U;
constexpr std::size_t kStableCyclesRequired = 10U;
constexpr std::size_t kStartupCyclesMaximum = 5'000U;
constexpr std::size_t kStateCyclesMaximum = 5'000U;
constexpr std::uint8_t kCspMode = 8U;

struct Drive {
  std::uint16_t ring_position{0U};
  std::uint16_t domain_id{0U};
  mca::hardware::Cia402PdoBindings pdo{};
  std::uint32_t held_position_bits{0U};
  std::uint16_t last_statusword{0U};
  std::uint8_t last_mode_display{0U};
  std::size_t enabled_stable_cycles{0U};
};

bool read_u16(Adapter& adapter, std::size_t binding,
              std::uint16_t& value) {
  std::uint64_t raw = 0U;
  if (!adapter.read_pdo(binding, raw).ok() || raw > 0xFFFFU) return false;
  value = static_cast<std::uint16_t>(raw);
  return true;
}

bool read_i32_bits(Adapter& adapter, std::size_t binding,
                   std::uint32_t& bits) {
  std::uint64_t raw = 0U;
  if (!adapter.read_pdo(binding, raw).ok() || raw > 0xFFFFFFFFULL)
    return false;
  bits = static_cast<std::uint32_t>(raw);
  return true;
}

bool write_value(Adapter& adapter, std::size_t binding,
                 std::uint64_t value) {
  return adapter.write_pdo(binding, value).ok();
}

bool transport_healthy(const Adapter& adapter) {
  const auto& health = adapter.runtime_health();
  if (!health.valid() || !health.link_up ||
      health.slaves_responding != kDriveCount ||
      adapter.observed_wkc() == 0U)
    return false;

  for (std::size_t i = 0U; i < kDriveCount; ++i) {
    if (!health.slaves[i].configured || !health.slaves[i].state_valid ||
        !health.slaves[i].online || !health.slaves[i].operational ||
        !adapter.domain_working_counter_complete(
            static_cast<std::uint16_t>(i + 1U)))
      return false;
  }
  return true;
}

void print_transport_diagnostics(const Adapter& adapter,
                                 const mca::core::Status& receive) {
  const auto& health = adapter.runtime_health();
  std::cerr << "TRANSPORT_DIAGNOSTIC"
            << " receive_ok=" << receive.ok()
            << " runtime_valid=" << health.valid()
            << " master_state_valid=" << health.master_state_valid
            << " master_state_error=" << health.master_state_error
            << " link_up=" << health.link_up
            << " responding=" << health.slaves_responding
            << " observed_wkc=" << adapter.observed_wkc()
            << " cycle_sequence=" << adapter.cycle_sequence();
  for (std::size_t i = 0U; i < kDriveCount; ++i) {
    const auto& slave = health.slaves[i];
    const auto domain = static_cast<std::uint16_t>(i + 1U);
    std::cerr << " slave[" << i << "]"
              << "{ring=" << slave.ring_position
              << ",configured=" << slave.configured
              << ",state_valid=" << slave.state_valid
              << ",state_error=" << slave.state_error
              << ",online=" << slave.online
              << ",operational=" << slave.operational
              << ",al_state=" << static_cast<unsigned int>(slave.al_state)
              << ",domain_wkc=" << adapter.domain_working_counter(domain)
              << ",domain_complete="
              << adapter.domain_working_counter_complete(domain) << "}";
  }
  std::cerr << '\n';
}

bool write_safe_outputs(Adapter& adapter,
                        const std::array<Drive, kDriveCount>& drives) {
  bool ok = true;
  for (const auto& drive : drives) {
    ok = write_value(adapter, drive.pdo.controlword, 0U) && ok;
    ok = write_value(adapter, drive.pdo.target_position, 0U) && ok;
    ok = write_value(adapter, drive.pdo.target_velocity, 0U) && ok;
    ok = write_value(adapter, drive.pdo.mode_of_operation, kCspMode) && ok;
  }
  return ok;
}

}  // namespace

int main() {
  unsigned long cycle_time_us = 1'000UL;
  if (const char* cycle_text = std::getenv("MCA_ECAT_CYCLE_TIME_US")) {
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(cycle_text, &end, 10);
    if (end == cycle_text || *end != '\0' ||
        parsed < 100UL || parsed > 100'000UL) {
      std::cerr << "FAIL: MCA_ECAT_CYCLE_TIME_US must be in [100, 100000]\n";
      return 1;
    }
    cycle_time_us = parsed;
  }

  Adapter::Config config{};
  config.master_index = 0U;
  config.cycle_time_ns = static_cast<std::uint32_t>(cycle_time_us * 1'000UL);
  config.send_interval_us = static_cast<std::uint32_t>(cycle_time_us);
  Adapter adapter(config);

  if (!adapter.request_master().ok() || !adapter.discover().ok()) {
    std::cerr << "FAIL: IgH master request/discovery\n";
    return 1;
  }

  const auto& topology = adapter.topology();
  if (topology.slave_count != kDriveCount) {
    std::cerr << "FAIL: multi-slave SIL requires exactly " << kDriveCount
              << " slaves; found " << topology.slave_count << '\n';
    return 1;
  }
  std::cout << "Discovered " << topology.slave_count << " drive slaves\n";

  Adapter::PdoMappingProfile mapping{};
  if (!adapter.discover_pdo_mapping(mapping).ok()) {
    std::cerr << "FAIL: discovered multi-slave PDO mapping\n";
    return 1;
  }

  std::array<Adapter::PdoBinding, 128U> bindings{};
  std::size_t binding_count = 0U;
  for (std::size_t i = 0U; i < mapping.size(); ++i) {
    const auto& entry = mapping[i];
    if (entry.index == 0U || entry.bit_length > 64U) continue;

    std::size_t topology_slot = topology.slave_count;
    for (std::size_t slot = 0U; slot < topology.slave_count; ++slot) {
      if (topology.slaves[slot].ring_position == entry.ring_position) {
        topology_slot = slot;
        break;
      }
    }
    if (topology_slot >= topology.slave_count ||
        binding_count >= bindings.size()) {
      std::cerr << "FAIL: PDO entry references an unknown slave or exceeds capacity\n";
      return 1;
    }

    const std::uint16_t domain_id =
        static_cast<std::uint16_t>(topology_slot + 1U);
    bindings[binding_count++] = {
        entry.ring_position, entry.vendor_id, entry.product_code,
        entry.index, entry.subindex, entry.bit_length, domain_id,
        entry.direction};
  }
  if (binding_count == 0U) {
    std::cerr << "FAIL: no scalar PDO entries discovered\n";
    return 1;
  }

  std::array<Drive, kDriveCount> drives{};
  for (std::size_t slot = 0U; slot < kDriveCount; ++slot) {
    const auto& slave = topology.slaves[slot];
    auto& drive = drives[slot];
    drive.ring_position = slave.ring_position;
    drive.domain_id = static_cast<std::uint16_t>(slot + 1U);
    if (!mca::hardware::resolve_cia402_pdo_bindings(
            bindings, binding_count, slave.ring_position, slave.vendor_id,
            slave.product_code, drive.pdo)) {
      std::cerr << "FAIL: ring position " << slave.ring_position
                << " does not expose a complete CiA 402 CSP PDO set\n";
      return 1;
    }
    std::cout << "Drive slot=" << slot
              << " ring_position=" << drive.ring_position
              << " domain=" << drive.domain_id
              << " bindings={cw:" << drive.pdo.controlword
              << ",sw:" << drive.pdo.statusword
              << ",target_pos:" << drive.pdo.target_position
              << ",actual_pos:" << drive.pdo.actual_position << "}\n";
  }

  if (!adapter.configure(bindings, binding_count, mapping,
                         Policy::UseDiscoveredMapping).ok()) {
    std::cerr << "FAIL: configure independent per-slave PDO domains\n";
    return 1;
  }
  if (!adapter.activate().ok()) {
    std::cerr << "FAIL: activate multi-slave IgH master\n";
    return 1;
  }

  mca::hardware::EthercatIghSlaveHealthMonitor<16U, 4U, 128U>
      slave_health({1U, 3U});

  if (!write_safe_outputs(adapter, drives)) {
    std::cerr << "FAIL: initialize safe output images\n";
    (void)adapter.shutdown();
    return 1;
  }

  // IgH may need multiple cyclic exchanges to move the slaves from PREOP
  // through SAFEOP to OP. An incomplete WKC makes receive_process() and
  // queue_send() report RuntimeError, but queue_send() still queues and sends
  // the safe output image. Do not abort convergence on that expected state.
  bool startup_ready = false;
  bool startup_output_write_failed = false;
  bool startup_frame_send_failed = false;
  for (std::size_t cycle = 0U; cycle < kStartupCyclesMaximum; ++cycle) {
    std::this_thread::sleep_for(std::chrono::microseconds(cycle_time_us));
    const auto previous_sequence = adapter.cycle_sequence();
    const auto receive = adapter.receive_process();
    const auto health_status = slave_health.update(adapter);
    const bool outputs_ok = write_safe_outputs(adapter, drives);
    const auto send = adapter.queue_send();

    if (adapter.cycle_sequence() == previous_sequence) {
      startup_frame_send_failed = true;
      std::cerr << "FAIL: IgH did not send startup cyclic frame\n";
      break;
    }
    if (!outputs_ok) {
      startup_output_write_failed = true;
      std::cerr << "FAIL: unable to refresh safe startup PDO image\n";
      break;
    }
    if (receive.ok() && send.ok() && health_status.ok() &&
        transport_healthy(adapter) && slave_health.snapshot().valid()) {
      startup_ready = true;
      std::cout << "Startup transport and per-slave health ready after "
                << (cycle + 1U) << " cycles\n";
      break;
    }

    const bool all_domains_complete =
        adapter.domain_working_counter_complete(1U) &&
        adapter.domain_working_counter_complete(2U);
    if ((!receive.ok() || !send.ok()) && all_domains_complete) {
      startup_frame_send_failed = !send.ok();
      std::cerr << "FAIL: cyclic exchange failed after both domain WKC "
                   "became complete\n";
      break;
    }
  }
  if (!startup_ready) {
    const auto& runtime = adapter.runtime_health();
    std::cerr << "FAIL: two-slave transport did not reach healthy OP/WKC"
              << " runtime_valid=" << runtime.valid()
              << " link_up=" << runtime.link_up
              << " responding=" << runtime.slaves_responding
              << " wkc=" << adapter.observed_wkc();
    for (std::size_t i = 0U; i < runtime.slave_count; ++i) {
      const auto& slave = runtime.slaves[i];
      std::cerr << " slave[" << i << "] ring=" << slave.ring_position
                << " online=" << slave.online
                << " operational=" << slave.operational
                << " state_valid=" << slave.state_valid;
    }
    std::cerr << " domain[1] wkc=" << adapter.domain_working_counter(1U)
              << " complete=" << adapter.domain_working_counter_complete(1U)
              << " domain[2] wkc=" << adapter.domain_working_counter(2U)
              << " complete=" << adapter.domain_working_counter_complete(2U)
              << " output_write_failed=" << startup_output_write_failed
              << " frame_send_failed=" << startup_frame_send_failed;
    std::cerr << '\n';
    (void)adapter.shutdown();
    return 1;
  }

  bool all_enabled = false;
  for (std::size_t cycle = 0U; cycle < kStateCyclesMaximum; ++cycle) {
    std::this_thread::sleep_for(std::chrono::microseconds(cycle_time_us));
    const auto receive = adapter.receive_process();
    if (!receive.ok() || !transport_healthy(adapter)) {
      // Preserve the exact first failing cycle before queue_send() advances the
      // adapter again. An incomplete WKC remains a hard acceptance failure.
      print_transport_diagnostics(adapter, receive);
      std::cerr << "ENABLE_SEQUENCE_LAST_FEEDBACK";
      for (const auto& drive : drives) {
        std::cerr << " ring=" << drive.ring_position
                  << " last_statusword=0x" << std::hex
                  << drive.last_statusword << std::dec
                  << " enabled_stable_cycles=" << drive.enabled_stable_cycles;
      }
      std::cerr << '\n';
      (void)adapter.queue_send();
      std::cerr << "FAIL: transport/WKC lost during two-drive enable sequence"
                << " receive_ok=" << receive.ok()
                << " transport_healthy=" << transport_healthy(adapter)
                << '\n';
      break;
    }
    if (!slave_health.update(adapter).ok()) {
      (void)adapter.queue_send();
      std::cerr << "FAIL: per-slave health monitor update\n";
      break;
    }

    bool writes_ok = true;
    for (auto& drive : drives) {
      if (!read_u16(adapter, drive.pdo.statusword, drive.last_statusword) ||
          !read_i32_bits(adapter, drive.pdo.actual_position,
                         drive.held_position_bits)) {
        writes_ok = false;
        break;
      }
      const auto observed =
          mca::hardware::decode_cia402_pds_state(drive.last_statusword);
      if (observed == Cia402PdsState::Fault ||
          observed == Cia402PdsState::FaultReactionActive ||
          observed == Cia402PdsState::Unknown ||
          observed == Cia402PdsState::NotReadyToSwitchOn) {
        std::cerr << "FAIL: unexpected PDS state on ring "
                  << drive.ring_position << " statusword=0x"
                  << std::hex << drive.last_statusword << std::dec << '\n';
        writes_ok = false;
        break;
      }

      mca::hardware::Cia402ControlRequest request{};
      request.communication_healthy = true;
      request.enable_operation = true;
      const auto plan =
          mca::hardware::plan_cia402_control(drive.last_statusword, request);
      std::uint64_t mode_raw = 0U;
      if (!adapter.read_pdo(drive.pdo.mode_display, mode_raw).ok() ||
          mode_raw > 0xFFU) {
        std::cerr << "FAIL: mode display read on ring "
                  << drive.ring_position << '\n';
        writes_ok = false;
        break;
      }
      drive.last_mode_display = static_cast<std::uint8_t>(mode_raw);

      writes_ok = write_value(adapter, drive.pdo.controlword, plan.controlword) &&
                  write_value(adapter, drive.pdo.target_position,
                              drive.held_position_bits) &&
                  write_value(adapter, drive.pdo.target_velocity, 0U) &&
                  write_value(adapter, drive.pdo.mode_of_operation, kCspMode);
      if (!writes_ok) break;

      if (plan.operation_enabled && plan.motion_permitted &&
          drive.last_mode_display == kCspMode) {
        ++drive.enabled_stable_cycles;
      } else {
        drive.enabled_stable_cycles = 0U;
      }
    }

    const auto send = adapter.queue_send();
    if (!writes_ok || !send.ok()) {
      std::cerr << "FAIL: drive PDO enable/update/send\n";
      break;
    }

    all_enabled = true;
    for (const auto& drive : drives) {
      if (drive.enabled_stable_cycles < kStableCyclesRequired)
        all_enabled = false;
    }
    if (all_enabled) break;
  }

  if (!all_enabled) {
    std::cerr << "FAIL: both drives did not reach stable Operation Enabled\n";
    (void)adapter.shutdown();
    return 1;
  }

  for (const auto& drive : drives) {
    std::cout << "PASS: ring_position=" << drive.ring_position
              << " Operation Enabled stable_cycles="
              << drive.enabled_stable_cycles
              << " statusword=0x" << std::hex << drive.last_statusword
              << " mode_display=0x"
              << static_cast<unsigned int>(drive.last_mode_display)
              << std::dec << " domain_wkc="
              << adapter.domain_working_counter(drive.domain_id) << '\n';
  }

  // Optional wire-level independent fault-isolation case. The simulator
  // fixture injects a fault into exactly one configured ring position; the
  // other drive must remain Operation Enabled with its own complete domain.
  const char* multi_fault_flag = std::getenv("MCA_ECAT_MULTI_FAULT_SIL");
  if (multi_fault_flag != nullptr && std::string(multi_fault_flag) == "1") {
    const char* target_text = std::getenv("MCA_ECAT_MULTI_FAULT_RING_POSITION");
    const char* reaction_text = std::getenv("MCA_ECAT_EXPECTED_FAULT_REACTION_CYCLES");
    if (target_text == nullptr || reaction_text == nullptr) {
      std::cerr << "FAIL: multi-fault mode requires target ring position and "
                   "expected FRA duration\n";
      (void)adapter.shutdown();
      return 1;
    }
    char* target_end = nullptr;
    char* reaction_end = nullptr;
    const unsigned long target_value = std::strtoul(target_text, &target_end, 10);
    const unsigned long reaction_value = std::strtoul(reaction_text, &reaction_end, 10);
    if (target_end == target_text || *target_end != '\0' ||
        reaction_end == reaction_text || *reaction_end != '\0' ||
        reaction_value == 0UL || reaction_value > 1000UL) {
      std::cerr << "FAIL: invalid multi-fault target or FRA duration\n";
      (void)adapter.shutdown();
      return 1;
    }
    std::size_t target_slot = kDriveCount;
    for (std::size_t slot = 0U; slot < drives.size(); ++slot) {
      if (drives[slot].ring_position == target_value) target_slot = slot;
    }
    if (target_slot >= kDriveCount) {
      std::cerr << "FAIL: requested fault ring position was not discovered\n";
      (void)adapter.shutdown();
      return 1;
    }
    const std::size_t peer_slot = 1U - target_slot;
    auto& target = drives[target_slot];
    auto& peer = drives[peer_slot];
    bool saw_reaction = false;
    bool saw_fault = false;
    std::size_t reaction_cycles = 0U;
    std::cout << "MULTI_FAULT target_ring=" << target.ring_position
              << " peer_ring=" << peer.ring_position
              << " expected_reaction_cycles=" << reaction_value << '\n';

    for (std::size_t cycle = 0U; cycle < 2'000U; ++cycle) {
      std::this_thread::sleep_for(std::chrono::microseconds(cycle_time_us));
      const auto receive = adapter.receive_process();
      if (!receive.ok() || !transport_healthy(adapter)) {
        print_transport_diagnostics(adapter, receive);
        (void)adapter.queue_send();
        std::cerr << "FAIL: transport/domain health lost during independent fault wait\n";
        (void)adapter.shutdown();
        return 1;
      }
      bool reads_ok = true;
      for (auto& drive : drives) {
        reads_ok = read_u16(adapter, drive.pdo.statusword, drive.last_statusword) &&
                   read_i32_bits(adapter, drive.pdo.actual_position,
                                 drive.held_position_bits) && reads_ok;
      }
      if (!reads_ok) {
        (void)adapter.queue_send();
        std::cerr << "FAIL: PDO feedback read failed during independent fault wait\n";
        (void)adapter.shutdown();
        return 1;
      }
      const auto target_state =
          mca::hardware::decode_cia402_pds_state(target.last_statusword);
      const auto peer_state =
          mca::hardware::decode_cia402_pds_state(peer.last_statusword);
      if (peer_state != Cia402PdsState::OperationEnabled ||
          !adapter.domain_working_counter_complete(peer.domain_id) ||
          !adapter.domain_working_counter_complete(target.domain_id)) {
        (void)adapter.queue_send();
        std::cerr << "FAIL: unaffected peer left Operation Enabled or a per-drive "
                     "domain WKC became incomplete; peer_statusword=0x"
                  << std::hex << peer.last_statusword << std::dec
                  << " peer_domain_wkc=" << adapter.domain_working_counter(peer.domain_id)
                  << " target_domain_wkc=" << adapter.domain_working_counter(target.domain_id)
                  << '\n';
        (void)adapter.shutdown();
        return 1;
      }
      if (target_state == Cia402PdsState::FaultReactionActive) {
        saw_reaction = true;
        ++reaction_cycles;
      } else if (target_state == Cia402PdsState::Fault) {
        saw_fault = true;
      } else if (saw_reaction) {
        (void)adapter.queue_send();
        std::cerr << "FAIL: target left FRA without reporting Fault\n";
        (void)adapter.shutdown();
        return 1;
      } else if (target_state != Cia402PdsState::OperationEnabled) {
        (void)adapter.queue_send();
        std::cerr << "FAIL: target left Operation Enabled before injected fault\n";
        (void)adapter.shutdown();
        return 1;
      }

      const bool writes_ok =
          write_value(adapter, target.pdo.controlword,
                      target_state == Cia402PdsState::OperationEnabled && !saw_reaction
                          ? 0x000FU : 0U) &&
          write_value(adapter, target.pdo.target_position, target.held_position_bits) &&
          write_value(adapter, target.pdo.target_velocity, 0U) &&
          write_value(adapter, target.pdo.mode_of_operation, kCspMode) &&
          write_value(adapter, peer.pdo.controlword, 0x000FU) &&
          write_value(adapter, peer.pdo.target_position, peer.held_position_bits) &&
          write_value(adapter, peer.pdo.target_velocity, 0U) &&
          write_value(adapter, peer.pdo.mode_of_operation, kCspMode);
      const auto send = adapter.queue_send();
      std::cout << "MULTI_FAULT_TRACE cycle=" << adapter.cycle_sequence()
                << " target_statusword=0x" << std::hex << target.last_statusword
                << " peer_statusword=0x" << peer.last_statusword << std::dec
                << " target_domain_wkc=" << adapter.domain_working_counter(target.domain_id)
                << " peer_domain_wkc=" << adapter.domain_working_counter(peer.domain_id)
                << '\n';
      if (!writes_ok || !send.ok()) {
        std::cerr << "FAIL: PDO write/send failed during independent fault wait\n";
        (void)adapter.shutdown();
        return 1;
      }
      if (saw_fault) break;
    }
    if (!saw_reaction || !saw_fault ||
        reaction_cycles != static_cast<std::size_t>(reaction_value)) {
      std::cerr << "FAIL: target FRA/Fault sequence mismatch; saw_reaction="
                << saw_reaction << " saw_fault=" << saw_fault
                << " reaction_cycles=" << reaction_cycles
                << " expected=" << reaction_value << '\n';
      (void)adapter.shutdown();
      return 1;
    }

    // Confirm a fresh Fault feedback sample immediately before arming reset.
    std::this_thread::sleep_for(std::chrono::microseconds(cycle_time_us));
    const auto confirm_receive = adapter.receive_process();
    std::uint16_t confirmed_fault = 0U;
    std::uint16_t confirmed_peer = 0U;
    if (!confirm_receive.ok() || !transport_healthy(adapter) ||
        !read_u16(adapter, target.pdo.statusword, confirmed_fault) ||
        !read_u16(adapter, peer.pdo.statusword, confirmed_peer) ||
        mca::hardware::decode_cia402_pds_state(confirmed_fault) != Cia402PdsState::Fault ||
        mca::hardware::decode_cia402_pds_state(confirmed_peer) != Cia402PdsState::OperationEnabled) {
      (void)adapter.queue_send();
      std::cerr << "FAIL: target Fault and peer Operation Enabled not confirmed before reset\n";
      (void)adapter.shutdown();
      return 1;
    }

    mca::hardware::Cia402FaultResetSequencer reset_sequencer(100U);
    if (!reset_sequencer.begin(confirmed_fault, true)) {
      std::cerr << "FAIL: independent target Fault Reset could not be armed\n";
      (void)adapter.shutdown();
      return 1;
    }
    const auto pulse = reset_sequencer.update(confirmed_fault, true);
    const bool pulse_written =
        pulse.pulse_sent && pulse.controlword == 0x0080U &&
        write_value(adapter, target.pdo.controlword, pulse.controlword) &&
        write_value(adapter, target.pdo.target_position, target.held_position_bits) &&
        write_value(adapter, target.pdo.target_velocity, 0U) &&
        write_value(adapter, target.pdo.mode_of_operation, kCspMode) &&
        write_value(adapter, peer.pdo.controlword, 0x000FU) &&
        write_value(adapter, peer.pdo.target_position, peer.held_position_bits) &&
        write_value(adapter, peer.pdo.target_velocity, 0U) &&
        write_value(adapter, peer.pdo.mode_of_operation, kCspMode);
    const auto pulse_send = adapter.queue_send();
    std::size_t pulse_count = pulse.pulse_sent ? 1U : 0U;
    if (!pulse_written || !pulse_send.ok()) {
      std::cerr << "FAIL: one-cycle target Fault Reset pulse failed\n";
      (void)adapter.shutdown();
      return 1;
    }

    bool reset_complete = false;
    for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
      std::this_thread::sleep_for(std::chrono::microseconds(cycle_time_us));
      const auto receive = adapter.receive_process();
      std::uint16_t target_status = 0U;
      std::uint16_t peer_status = 0U;
      std::uint32_t target_position = 0U;
      std::uint32_t peer_position = 0U;
      if (!receive.ok() || !transport_healthy(adapter) ||
          !read_u16(adapter, target.pdo.statusword, target_status) ||
          !read_u16(adapter, peer.pdo.statusword, peer_status) ||
          !read_i32_bits(adapter, target.pdo.actual_position, target_position) ||
          !read_i32_bits(adapter, peer.pdo.actual_position, peer_position) ||
          mca::hardware::decode_cia402_pds_state(peer_status) != Cia402PdsState::OperationEnabled ||
          !adapter.domain_working_counter_complete(peer.domain_id) ||
          !adapter.domain_working_counter_complete(target.domain_id)) {
        (void)adapter.queue_send();
        std::cerr << "FAIL: peer/domain health lost while awaiting target reset feedback\n";
        (void)adapter.shutdown();
        return 1;
      }
      const auto plan = reset_sequencer.update(target_status, true);
      if (plan.pulse_sent) ++pulse_count;
      const bool writes_ok =
          write_value(adapter, target.pdo.controlword, plan.controlword) &&
          write_value(adapter, target.pdo.target_position, target_position) &&
          write_value(adapter, target.pdo.target_velocity, 0U) &&
          write_value(adapter, target.pdo.mode_of_operation, kCspMode) &&
          write_value(adapter, peer.pdo.controlword, 0x000FU) &&
          write_value(adapter, peer.pdo.target_position, peer_position) &&
          write_value(adapter, peer.pdo.target_velocity, 0U) &&
          write_value(adapter, peer.pdo.mode_of_operation, kCspMode);
      const auto send = adapter.queue_send();
      std::cout << "MULTI_FAULT_RESET_TRACE cycle=" << adapter.cycle_sequence()
                << " target_statusword=0x" << std::hex << target_status
                << " peer_statusword=0x" << peer_status << std::dec
                << " target_controlword=0x" << std::hex << plan.controlword
                << std::dec << " phase=" << static_cast<unsigned int>(plan.phase)
                << " peer_domain_wkc=" << adapter.domain_working_counter(peer.domain_id)
                << '\n';
      if (!writes_ok || !send.ok() || plan.failed) {
        std::cerr << "FAIL: target reset sequence or peer preservation failed\n";
        (void)adapter.shutdown();
        return 1;
      }
      if (plan.complete) {
        reset_complete =
            mca::hardware::decode_cia402_pds_state(target_status) ==
            Cia402PdsState::SwitchOnDisabled;
        target.last_statusword = target_status;
        break;
      }
    }
    if (!reset_complete || pulse_count != 1U) {
      std::cerr << "FAIL: target did not return Disabled with exactly one reset pulse; "
                << "pulse_count=" << pulse_count << '\n';
      (void)adapter.shutdown();
      return 1;
    }
    std::cout << "PASS: independent fault isolation target_ring="
              << target.ring_position << " peer_ring=" << peer.ring_position
              << " FRA_cycles=" << reaction_cycles
              << " reset_pulses=" << pulse_count
              << " target_statusword=0x" << std::hex << target.last_statusword
              << std::dec << " peer_remained_operation_enabled=1\n";
  }

  bool all_disabled = false;
  for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
    std::this_thread::sleep_for(std::chrono::microseconds(cycle_time_us));
    const auto receive = adapter.receive_process();
    if (!receive.ok() || !transport_healthy(adapter)) {
      (void)adapter.queue_send();
      break;
    }

    bool writes_ok = true;
    all_disabled = true;
    for (auto& drive : drives) {
      std::uint16_t statusword = 0U;
      std::uint32_t actual_position = 0U;
      if (!read_u16(adapter, drive.pdo.statusword, statusword) ||
          !read_i32_bits(adapter, drive.pdo.actual_position, actual_position)) {
        writes_ok = false;
        break;
      }
      drive.last_statusword = statusword;
      writes_ok = write_value(adapter, drive.pdo.controlword, 0U) &&
                  write_value(adapter, drive.pdo.target_velocity, 0U) &&
                  write_value(adapter, drive.pdo.target_position,
                              actual_position) &&
                  write_value(adapter, drive.pdo.mode_of_operation, kCspMode);
      if (!writes_ok) break;
      if (mca::hardware::decode_cia402_pds_state(statusword) !=
          Cia402PdsState::SwitchOnDisabled)
        all_disabled = false;
    }

    const auto send = adapter.queue_send();
    if (!writes_ok || !send.ok()) break;
    if (all_disabled) break;
  }

  bool final_health_ok = transport_healthy(adapter);
  const auto& final_runtime = adapter.runtime_health();
  std::cout << "Final multi-slave health: wkc=" << adapter.observed_wkc()
            << " link_up=" << final_runtime.link_up
            << " responding=" << final_runtime.slaves_responding
            << " disabled_feedback=" << all_disabled << '\n';
  for (std::size_t slot = 0U; slot < drives.size(); ++slot) {
    const auto& drive = drives[slot];
    std::cout << "slave ring_position=" << drive.ring_position
              << " domain=" << drive.domain_id
              << " domain_wkc=" << adapter.domain_working_counter(drive.domain_id)
              << " complete="
              << adapter.domain_working_counter_complete(drive.domain_id)
              << " online=" << final_runtime.slaves[slot].online
              << " operational=" << final_runtime.slaves[slot].operational
              << " statusword=0x" << std::hex << drive.last_statusword
              << std::dec << '\n';
  }

  const auto shutdown = adapter.shutdown();
  if (!shutdown.ok()) {
    std::cerr << "FAIL: shutdown\n";
    return 1;
  }
  if (!final_health_ok || !all_disabled) {
    std::cerr << "FAIL: multi-slave CiA 402 baseline acceptance\n";
    return 1;
  }

  std::cout << "PASS: two independent EtherCAT slave domains reached "
               "Operation Enabled and each returned to Switch On Disabled.\n";
  const char* multi_fault_mode = std::getenv("MCA_ECAT_MULTI_FAULT_SIL");
  if (multi_fault_mode != nullptr && std::string(multi_fault_mode) == "1") {
    std::cout << "NOTE: This is two-slave KickCAT fault-isolation SIL only; "
                 "vendor semantics, DC timing and physical HIL remain unvalidated.\n";
  } else {
    std::cout << "NOTE: This is two-slave KickCAT baseline SIL only; independent "
                 "per-drive fault injection, vendor semantics, DC timing and "
                 "physical HIL remain unvalidated.\n";
  }
  return 0;
}
