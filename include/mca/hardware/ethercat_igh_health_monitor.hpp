#pragma once

#include <cstdint>

#include "mca/hardware/ethercat_health_monitor.hpp"
#include "mca/hardware/ethercat_igh_adapter.hpp"

namespace mca::hardware {

// Bridges the IgH-specific runtime snapshot into the protocol-neutral bus
// monitor. This is intentionally called by the supervisory loop, not from
// inside EthercatIghAdapter::exchange().
template <std::size_t MaxSlaves, std::size_t MaxDomains, std::size_t MaxEntries>
class EthercatIghHealthMonitor final {
 public:
  using Adapter = EthercatIghAdapter<MaxSlaves, MaxDomains, MaxEntries>;

  explicit EthercatIghHealthMonitor(
      const EthercatBusHealthConfig& config = {}) noexcept
      : monitor_(config) {}

  [[nodiscard]] const EthercatBusHealthSnapshot& snapshot() const noexcept {
    return monitor_.snapshot();
  }

  [[nodiscard]] EthercatBusHealthMonitor& monitor() noexcept {
    return monitor_;
  }

  core::Status update(const Adapter& adapter,
                      bool latched_fault = false) noexcept {
    const auto& runtime = adapter.runtime_health();
    const auto& topology = adapter.topology();
    const auto& layout = adapter.layout();

    EthercatBusHealthSample sample{};
    sample.sequence = runtime.sequence;
    sample.diagnostics_valid =
        runtime.valid() && topology.valid() &&
        layout.valid(topology.slave_count);
    sample.link_up = runtime.link_up;
    sample.expected_slaves =
        static_cast<std::uint32_t>(topology.slave_count);
    sample.responding_slaves = runtime.slaves_responding;
    sample.all_slaves_online = sample.diagnostics_valid;
    sample.all_slaves_operational = sample.diagnostics_valid;

    if (sample.diagnostics_valid) {
      for (std::size_t i = 0; i < runtime.slave_count; ++i) {
        const auto& slave = runtime.slaves[i];
        if (!slave.configured || !slave.state_valid || !slave.online)
          sample.all_slaves_online = false;
        if (!slave.configured || !slave.state_valid || !slave.operational)
          sample.all_slaves_operational = false;
      }

      if (layout.domain_count == 0U) {
        sample.working_counter = EthercatWorkingCounterState::Unknown;
      } else {
        bool all_domains_complete = true;
        for (std::size_t i = 0; i < layout.domain_count; ++i) {
          if (!adapter.domain_working_counter_complete(layout.domains[i].id)) {
            all_domains_complete = false;
            break;
          }
        }
        sample.working_counter =
            all_domains_complete ? EthercatWorkingCounterState::Complete
                                 : EthercatWorkingCounterState::Incomplete;
      }
    }
    sample.latched_fault = latched_fault;
    return monitor_.update(sample);
  }

  core::Status clear_latched_fault() noexcept {
    return monitor_.clear_latched_fault();
  }

  void reset() noexcept { monitor_.reset(); }

 private:
  EthercatBusHealthMonitor monitor_;
};

}  // namespace mca::hardware
