#pragma once

#include <cstdint>

#include "mca/core/types.hpp"
#include "mca/hardware/ethercat_master.hpp"

namespace mca::hardware {

enum class EthercatBusHealthState : std::uint8_t {
  Unknown,
  Healthy,
  Degraded,
  Lost,
  Recovering,
  Fault
};

struct EthercatBusHealthConfig {
  std::uint32_t lost_after_cycles{3U};
  std::uint32_t recovery_after_cycles{5U};

  [[nodiscard]] bool valid() const noexcept {
    return lost_after_cycles > 0U && recovery_after_cycles > 0U;
  }
};

// Normalized input to the bus-health monitor. Adapters translate their
// protocol-specific snapshots into this fixed-size, allocation-free sample.
struct EthercatBusHealthSample {
  std::uint64_t sequence{0U};
  bool diagnostics_valid{false};
  bool link_up{false};
  std::uint32_t expected_slaves{0U};
  std::uint32_t responding_slaves{0U};
  bool all_slaves_online{false};
  bool all_slaves_operational{false};
  EthercatWorkingCounterState working_counter{
      EthercatWorkingCounterState::Unknown};
  bool latched_fault{false};
};

struct EthercatBusHealthSnapshot {
  EthercatBusHealthState state{EthercatBusHealthState::Unknown};
  std::uint64_t sequence{0U};
  std::uint32_t consecutive_bad_cycles{0U};
  std::uint32_t consecutive_good_cycles{0U};
  bool sample_valid{false};
};

// Stateful debounce is deliberately outside the cyclic adapter. update()
// performs bounded integer work only and never allocates or blocks.
class EthercatBusHealthMonitor final {
 public:
  explicit EthercatBusHealthMonitor(
      const EthercatBusHealthConfig& config = {}) noexcept
      : config_(config) {}

  [[nodiscard]] const EthercatBusHealthSnapshot& snapshot() const noexcept {
    return snapshot_;
  }

  [[nodiscard]] bool configured() const noexcept { return config_.valid(); }

  core::Status update(const EthercatBusHealthSample& sample) noexcept {
    if (!config_.valid() || sample.sequence == 0U ||
        sample.sequence <= last_sequence_) {
      return {core::StatusCode::InvalidArgument};
    }

    last_sequence_ = sample.sequence;
    snapshot_.sequence = sample.sequence;
    snapshot_.sample_valid = sample.diagnostics_valid;
    if (!sample.diagnostics_valid) {
      // An invalid sample breaks both debounce streaks. Recovery must be
      // confirmed from fresh consecutive samples after diagnostics return.
      snapshot_.consecutive_bad_cycles = 0U;
      snapshot_.consecutive_good_cycles = 0U;
      snapshot_.state = EthercatBusHealthState::Unknown;
      return {};
    }

    if (sample.latched_fault) {
      snapshot_.state = EthercatBusHealthState::Fault;
      snapshot_.consecutive_bad_cycles = 0U;
      snapshot_.consecutive_good_cycles = 0U;
      fault_latched_ = true;
      return {};
    }
    if (fault_latched_) {
      snapshot_.state = EthercatBusHealthState::Fault;
      return {};
    }

    const bool topology_lost =
        !sample.link_up || sample.expected_slaves == 0U ||
        sample.responding_slaves != sample.expected_slaves ||
        !sample.all_slaves_online;
    const bool degraded =
        !sample.all_slaves_operational ||
        sample.working_counter != EthercatWorkingCounterState::Complete;

    if (topology_lost || degraded) {
      increment_saturated(snapshot_.consecutive_bad_cycles);
      snapshot_.consecutive_good_cycles = 0U;
      snapshot_.state =
          snapshot_.consecutive_bad_cycles >= config_.lost_after_cycles
              ? EthercatBusHealthState::Lost
              : EthercatBusHealthState::Degraded;
      return {};
    }

    snapshot_.consecutive_bad_cycles = 0U;
    if (snapshot_.state == EthercatBusHealthState::Healthy) {
      snapshot_.consecutive_good_cycles =
          config_.recovery_after_cycles;
      return {};
    }

    increment_saturated(snapshot_.consecutive_good_cycles);
    snapshot_.state =
        snapshot_.consecutive_good_cycles >= config_.recovery_after_cycles
            ? EthercatBusHealthState::Healthy
            : EthercatBusHealthState::Recovering;
    return {};
  }

  // A latched safety/drive fault is cleared only by an explicit supervisory
  // action after the underlying source has confirmed the fault is inactive.
  core::Status clear_latched_fault() noexcept {
    if (!fault_latched_) return {core::StatusCode::RuntimeError};
    fault_latched_ = false;
    snapshot_.state = EthercatBusHealthState::Recovering;
    snapshot_.consecutive_bad_cycles = 0U;
    snapshot_.consecutive_good_cycles = 0U;
    return {};
  }

  void reset() noexcept {
    snapshot_ = {};
    last_sequence_ = 0U;
    fault_latched_ = false;
  }

 private:
  static void increment_saturated(std::uint32_t& value) noexcept {
    if (value != UINT32_MAX) ++value;
  }

  EthercatBusHealthConfig config_{};
  EthercatBusHealthSnapshot snapshot_{};
  std::uint64_t last_sequence_{0U};
  bool fault_latched_{false};
};

}  // namespace mca::hardware
