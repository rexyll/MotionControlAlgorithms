#pragma once

#include <cstdint>

#include "mca/hardware/ethercat_health_monitor.hpp"
#include "mca/hardware/ethercat_igh_adapter.hpp"

namespace mca::hardware {

// Bridges the IgH-specific runtime snapshot into the protocol-neutral bus
// monitor. Invoke from the supervisory loop, outside the cyclic exchange.
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
    // A complete master/slave read is insufficient if the adapter's
    // configured topology and the sampled slave set disagree.
    sample.diagnostics_valid =
        runtime.valid() && topology.valid() &&
        runtime.slave_count == topology.slave_count &&
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


struct EthercatIghSlaveHealthConfig {
  std::uint32_t lost_after_cycles{1U};
  std::uint32_t recovery_after_cycles{3U};

  [[nodiscard]] bool valid() const noexcept {
    return lost_after_cycles > 0U && recovery_after_cycles > 0U;
  }
};

// Produces independent transport/AL/WKC health for every PDO-configured slave.
// A domain must be owned by exactly one topology slot; otherwise its WKC cannot
// be attributed to an individual slave and this monitor rejects the layout.
template <std::size_t MaxSlaves, std::size_t MaxDomains,
          std::size_t MaxEntries>
class EthercatIghSlaveHealthMonitor final {
 public:
  using Adapter = EthercatIghAdapter<MaxSlaves, MaxDomains, MaxEntries>;
  using Topology = typename Adapter::Topology;
  using Snapshot = EthercatSlaveHealthSnapshot<MaxSlaves>;

  explicit EthercatIghSlaveHealthMonitor(
      const EthercatIghSlaveHealthConfig& config = {}) noexcept
      : config_(config) {}

  [[nodiscard]] const Snapshot& snapshot() const noexcept { return snapshot_; }
  [[nodiscard]] bool configured() const noexcept { return config_.valid(); }

  core::Status update(const Adapter& adapter) noexcept {
    if (!configured()) {
      snapshot_ = {};
      return {core::StatusCode::InvalidArgument};
    }

    const auto& topology = adapter.topology();
    const auto& layout = adapter.layout();
    const auto& runtime = adapter.runtime_health();
    if (!topology.valid() || !layout.valid(topology.slave_count) ||
        runtime.sequence == 0U ||
        runtime.slave_count != topology.slave_count ||
        !runtime.master_state_valid) {
      snapshot_ = {};
      return {core::StatusCode::RuntimeError};
    }

    reset_tracking_if_topology_changed(topology);

    constexpr std::size_t kNoOwner = MaxSlaves;
    std::array<std::size_t, MaxDomains> domain_owner{};
    domain_owner.fill(kNoOwner);
    std::array<bool, MaxSlaves> has_domain{};

    for (std::size_t entry_index = 0U;
         entry_index < layout.entry_count; ++entry_index) {
      const auto& entry = layout.entries[entry_index];
      const std::size_t domain_index = find_domain(layout, entry.domain_id);
      if (domain_index >= layout.domain_count) {
        snapshot_ = {};
        return {core::StatusCode::ConstraintViolation};
      }
      const std::size_t owner = entry.slave_position;
      if (domain_owner[domain_index] != kNoOwner &&
          domain_owner[domain_index] != owner) {
        snapshot_ = {};
        return {core::StatusCode::ConstraintViolation};
      }
      domain_owner[domain_index] = owner;
      has_domain[owner] = true;
    }

    snapshot_ = {};
    snapshot_.sequence = runtime.sequence;
    for (std::size_t topology_index = 0U;
         topology_index < topology.slave_count; ++topology_index) {
      const auto* observed = find_runtime_slave(
          runtime, topology.slaves[topology_index].ring_position);
      if (observed == nullptr || !observed->configured) continue;
      if (!has_domain[topology_index]) {
        snapshot_ = {};
        return {core::StatusCode::ConstraintViolation};
      }

      auto& tracked = tracking_[topology_index];
      tracked.used = true;
      tracked.ring_position = topology.slaves[topology_index].ring_position;

      if (!observed->state_valid) {
        tracked.state = EthercatSlaveHealthState::Unknown;
        tracked.bad_cycles = 0U;
        tracked.good_cycles = 0U;
        append_snapshot(tracked);
        continue;
      }

      bool healthy = runtime.link_up && observed->online &&
                     observed->operational;
      for (std::size_t d = 0U; d < layout.domain_count; ++d) {
        if (domain_owner[d] == topology_index &&
            !adapter.domain_working_counter_complete(layout.domains[d].id))
          healthy = false;
      }
      update_state(tracked, healthy);
      append_snapshot(tracked);
    }

    if (snapshot_.slave_count == 0U) {
      snapshot_ = {};
      return {core::StatusCode::ConstraintViolation};
    }
    return {};
  }

  // Call only after the application has independently confirmed a drive fault.
  [[nodiscard]] bool latch_fault(std::uint16_t ring_position) noexcept {
    auto* tracked = find_tracked(ring_position);
    if (tracked == nullptr ||
        !ethercat_slave_health_transition_allowed(
            tracked->state, EthercatSlaveHealthState::Fault))
      return false;
    tracked->state = EthercatSlaveHealthState::Fault;
    tracked->good_cycles = 0U;
    tracked->bad_cycles = 0U;
    refresh_snapshot_state(ring_position, tracked->state);
    return true;
  }

  // This clears only the monitor's software latch. The caller must first
  // complete the vendor/CiA402 reset procedure and verify safe feedback.
  [[nodiscard]] bool clear_fault(std::uint16_t ring_position) noexcept {
    auto* tracked = find_tracked(ring_position);
    if (tracked == nullptr ||
        tracked->state != EthercatSlaveHealthState::Fault ||
        !ethercat_slave_health_transition_allowed(
            tracked->state, EthercatSlaveHealthState::Recovering))
      return false;
    tracked->state = EthercatSlaveHealthState::Recovering;
    tracked->good_cycles = 0U;
    tracked->bad_cycles = 0U;
    refresh_snapshot_state(ring_position, tracked->state);
    return true;
  }

  void reset() noexcept {
    tracking_ = {};
    tracked_topology_ = {};
    snapshot_ = {};
  }

 private:
  struct TrackedSlave {
    std::uint16_t ring_position{0U};
    EthercatSlaveHealthState state{EthercatSlaveHealthState::Unknown};
    std::uint32_t bad_cycles{0U};
    std::uint32_t good_cycles{0U};
    bool used{false};
  };

  static std::size_t find_domain(
      const typename Adapter::Layout& layout,
      std::uint16_t domain_id) noexcept {
    for (std::size_t i = 0U; i < layout.domain_count; ++i)
      if (layout.domains[i].id == domain_id) return i;
    return layout.domain_count;
  }

  static const typename Adapter::SlaveRuntimeHealth* find_runtime_slave(
      const typename Adapter::RuntimeHealth& runtime,
      std::uint16_t ring_position) noexcept {
    for (std::size_t i = 0U; i < runtime.slave_count; ++i)
      if (runtime.slaves[i].ring_position == ring_position)
        return &runtime.slaves[i];
    return nullptr;
  }

  void reset_tracking_if_topology_changed(
      const Topology& topology) noexcept {
    if (ethercat_topology_identity_matches(tracked_topology_, topology))
      return;
    tracking_ = {};
    tracked_topology_ = topology;
  }

  void update_state(TrackedSlave& tracked, bool healthy) noexcept {
    if (tracked.state == EthercatSlaveHealthState::Fault) return;

    if (healthy) {
      tracked.bad_cycles = 0U;
      if (tracked.state == EthercatSlaveHealthState::Unknown) {
        tracked.state = EthercatSlaveHealthState::Present;
        tracked.good_cycles = 0U;
      } else if (tracked.state == EthercatSlaveHealthState::Lost) {
        tracked.state = EthercatSlaveHealthState::Recovering;
        tracked.good_cycles = 1U;
        if (tracked.good_cycles >= config_.recovery_after_cycles)
          tracked.state = EthercatSlaveHealthState::Present;
      } else if (tracked.state == EthercatSlaveHealthState::Recovering) {
        increment_saturated(tracked.good_cycles);
        if (tracked.good_cycles >= config_.recovery_after_cycles)
          tracked.state = EthercatSlaveHealthState::Present;
      } else {
        tracked.good_cycles = 0U;
      }
      return;
    }

    tracked.good_cycles = 0U;
    if (tracked.state == EthercatSlaveHealthState::Recovering) {
      tracked.state = EthercatSlaveHealthState::Lost;
      tracked.bad_cycles = 1U;
    } else if (tracked.state == EthercatSlaveHealthState::Unknown ||
               tracked.state == EthercatSlaveHealthState::Present) {
      increment_saturated(tracked.bad_cycles);
      if (tracked.bad_cycles >= config_.lost_after_cycles)
        tracked.state = EthercatSlaveHealthState::Lost;
    }
  }

  static void increment_saturated(std::uint32_t& value) noexcept {
    if (value != UINT32_MAX) ++value;
  }

  void append_snapshot(const TrackedSlave& tracked) noexcept {
    if (snapshot_.slave_count >= MaxSlaves) return;
    auto& item = snapshot_.slaves[snapshot_.slave_count++];
    item.ring_position = tracked.ring_position;
    item.state = tracked.state;
  }

  TrackedSlave* find_tracked(std::uint16_t ring_position) noexcept {
    for (auto& tracked : tracking_)
      if (tracked.used && tracked.ring_position == ring_position)
        return &tracked;
    return nullptr;
  }

  void refresh_snapshot_state(std::uint16_t ring_position,
                              EthercatSlaveHealthState state) noexcept {
    for (std::size_t i = 0U; i < snapshot_.slave_count; ++i) {
      if (snapshot_.slaves[i].ring_position == ring_position) {
        snapshot_.slaves[i].state = state;
        return;
      }
    }
  }

  EthercatIghSlaveHealthConfig config_{};
  Topology tracked_topology_{};
  std::array<TrackedSlave, MaxSlaves> tracking_{};
  Snapshot snapshot_{};
};

}  // namespace mca::hardware
