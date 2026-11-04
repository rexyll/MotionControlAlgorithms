#include "mca/hardware/ethercat_igh_adapter.hpp"
#include "mca/hardware/cia402_pdo_bindings.hpp"
#include "mca/hardware/cia402_pds_controller.hpp"
#include "mca/hardware/ethercat_igh_health_monitor.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
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
  Adapter::Config config{};
  config.master_index = 0U;
  config.cycle_time_ns = 1'000'000U;
  config.send_interval_us = 1'000U;
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

  bool startup_ready = false;
  for (std::size_t cycle = 0U; cycle < kStartupCyclesMaximum; ++cycle) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto receive = adapter.receive_process();
    const auto health_status = slave_health.update(adapter);
    const bool outputs_ok = write_safe_outputs(adapter, drives);
    const auto send = adapter.queue_send();
    if (!outputs_ok || !send.ok()) {
      std::cerr << "FAIL: startup safe-output exchange\n";
      break;
    }
    if (receive.ok() && health_status.ok() && transport_healthy(adapter) &&
        slave_health.snapshot().valid()) {
      startup_ready = true;
      std::cout << "Startup transport and per-slave health ready after "
                << (cycle + 1U) << " cycles\n";
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
    std::cerr << '\n';
    (void)adapter.shutdown();
    return 1;
  }

  bool all_enabled = false;
  for (std::size_t cycle = 0U; cycle < kStateCyclesMaximum; ++cycle) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto receive = adapter.receive_process();
    if (!receive.ok() || !transport_healthy(adapter)) {
      (void)adapter.queue_send();
      std::cerr << "FAIL: transport/WKC lost during two-drive enable sequence\n";
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
      if (!adapter.read_pdo(drive.pdo.mode_display, mode_raw) ||
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

  bool all_disabled = false;
  for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
  std::cout << "NOTE: This is two-slave KickCAT baseline SIL only; independent "
               "per-drive fault injection, vendor semantics, DC timing and "
               "physical HIL remain unvalidated.\n";
  return 0;
}
