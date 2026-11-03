#include "mca/hardware/ethercat_igh_adapter.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <thread>

int main() {
  using Adapter = mca::hardware::EthercatIghAdapter<16U, 4U, 128U>;
  using Direction = mca::hardware::EthercatPdoDirection;
  using Policy = mca::hardware::EthercatPdoMappingPolicy;

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
  if (topology.slave_count == 0U || topology.slave_count > 16U) {
    std::cerr << "FAIL: unexpected slave count " << topology.slave_count << '\n';
    return 1;
  }
  std::cout << "Discovered " << topology.slave_count << " slave(s)\n";
  for (std::size_t i = 0; i < topology.slave_count; ++i) {
    const auto& slave = topology.slaves[i];
    std::cout << "  ring_position=" << slave.ring_position
              << " vendor=0x" << std::hex << slave.vendor_id
              << " product=0x" << slave.product_code << std::dec << '\n';
  }

  Adapter::PdoMappingProfile mapping{};
  if (!adapter.discover_pdo_mapping(mapping).ok()) {
    std::cerr << "FAIL: discover_pdo_mapping (capacity or unsupported map)\n";
    return 1;
  }

  std::array<Adapter::PdoBinding, 128U> bindings{};
  std::size_t binding_count = 0U;
  for (std::size_t i = 0; i < mapping.size(); ++i) {
    const auto& entry = mapping[i];
    // Keep padding and >64-bit entries in the full configured map, but do not
    // expose them through the uint64_t scalar process-image API.
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

  std::size_t stable_cycles = 0U;
  constexpr std::size_t kMaximumCycles = 5'000U;
  for (std::size_t cycle = 0; cycle < kMaximumCycles; ++cycle) {
    bool values_ok = true;
    for (std::size_t i = 0; i < binding_count; ++i) {
      if (bindings[i].direction == Direction::Output) {
        if (!adapter.write_pdo(i, 0U).ok()) values_ok = false;
      }
    }

    const auto cycle_status = adapter.exchange();
    const auto& health = adapter.runtime_health();
    bool all_operational = health.valid() && health.link_up &&
        health.slaves_responding == topology.slave_count;
    for (std::size_t i = 0; i < health.slave_count; ++i) {
      if (health.slaves[i].configured)
        all_operational = all_operational && health.slaves[i].operational;
    }

    bool inputs_readable = true;
    for (std::size_t i = 0; i < binding_count; ++i) {
      if (bindings[i].direction == Direction::Input) {
        std::uint64_t value = 0U;
        if (!adapter.read_pdo(i, value).ok()) inputs_readable = false;
      }
    }

    const bool wkc_complete =
        adapter.domain_working_counter_complete(1U) &&
        adapter.observed_wkc() > 0U;
    if (cycle_status.ok() && values_ok && inputs_readable &&
        all_operational && wkc_complete) {
      ++stable_cycles;
      if (stable_cycles >= 10U) break;
    } else {
      stable_cycles = 0U;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const auto& final_health = adapter.runtime_health();
  const bool passed = stable_cycles >= 10U &&
      adapter.domain_working_counter_complete(1U) &&
      adapter.observed_wkc() > 0U;
  std::cout << "cycles=" << adapter.cycle_sequence()
            << " stable_cycles=" << stable_cycles
            << " observed_wkc=" << adapter.observed_wkc()
            << " domain_complete="
            << adapter.domain_working_counter_complete(1U) << '\n';
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
  if (!passed) {
    std::cerr << "FAIL: cyclic PDO/WKC/OP acceptance not reached\n";
    return 1;
  }
  std::cout << "PASS: IgH adapter discovered mapping, verified it, exchanged "
               "cyclic PDO data with complete WKC and all slaves OP.\n";
  return 0;
}
