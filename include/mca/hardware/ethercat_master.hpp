#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mca/core/types.hpp"
#include "mca/hardware/cia402_profile.hpp"

namespace mca::hardware {

enum class EthercatMasterState : std::uint8_t {
  Offline,
  Discovered,
  Configured,
  Active,
  Recovering,
  Fault
};

enum class EthercatPdoDirection : std::uint8_t {
  Input,
  Output
};

enum class EthercatWorkingCounterState : std::uint8_t {
  Unknown,
  Zero,
  Incomplete,
  Complete,
  Unexpected
};

enum class EthercatDcState : std::uint8_t {
  Unsupported,
  NotConfigured,
  Synchronizing,
  Locked,
  Lost,
  Degraded
};

enum class EthercatCycleTimingState : std::uint8_t {
  Unknown,
  OnTime,
  Late,
  Missed
};

struct EthercatSlaveInfo {
  std::uint16_t station_position{0};
  std::uint32_t vendor_id{0};
  std::uint32_t product_code{0};
  std::uint32_t revision{0};
  std::uint32_t serial_number{0};
  EthercatAlState al_state{EthercatAlState::Unknown};
  std::uint16_t al_status_code{0};
};

template <std::size_t MaxSlaves>
struct EthercatTopologySnapshot {
  static_assert(MaxSlaves > 0U, "EtherCAT topology capacity must be positive");

  std::array<EthercatSlaveInfo, MaxSlaves> slaves{};
  std::size_t slave_count{0U};
  std::uint64_t sequence{0U};

  [[nodiscard]] bool valid() const noexcept {
    if (slave_count == 0U || slave_count > MaxSlaves || sequence == 0U)
      return false;
    for (std::size_t i = 0; i < slave_count; ++i) {
      for (std::size_t j = 0; j < i; ++j) {
        if (slaves[i].station_position == slaves[j].station_position)
          return false;
      }
    }
    return true;
  }
};

enum class EthercatSlaveHealthState : std::uint8_t {
  Unknown,
  Present,
  Lost,
  Recovering,
  Fault
};

struct EthercatSlaveHealth {
  std::uint16_t station_position{0U};
  EthercatSlaveHealthState state{EthercatSlaveHealthState::Unknown};
};

template <std::size_t MaxSlaves>
struct EthercatSlaveHealthSnapshot {
  static_assert(MaxSlaves > 0U, "EtherCAT health capacity must be positive");

  std::array<EthercatSlaveHealth, MaxSlaves> slaves{};
  std::size_t slave_count{0U};
  std::uint64_t sequence{0U};

  [[nodiscard]] bool valid() const noexcept {
    if (slave_count == 0U || slave_count > MaxSlaves || sequence == 0U)
      return false;
    for (std::size_t i = 0; i < slave_count; ++i) {
      if (slaves[i].state == EthercatSlaveHealthState::Unknown)
        return false;
      for (std::size_t j = 0; j < i; ++j) {
        if (slaves[i].station_position == slaves[j].station_position)
          return false;
      }
    }
    return true;
  }
};

// A rediscovered bus may reuse a PDO layout only when the complete ordered
// station identity set is unchanged. AL state/status and snapshot sequence are
// intentionally excluded; station position and identity fields are not.
template <std::size_t MaxSlaves>
constexpr bool ethercat_topology_identity_matches(
    const EthercatTopologySnapshot<MaxSlaves>& configured,
    const EthercatTopologySnapshot<MaxSlaves>& rediscovered) noexcept {
  if (!configured.valid() || !rediscovered.valid() ||
      configured.slave_count != rediscovered.slave_count)
    return false;
  for (std::size_t i = 0; i < configured.slave_count; ++i) {
    const auto& expected = configured.slaves[i];
    const EthercatSlaveInfo* actual = nullptr;
    for (std::size_t j = 0; j < rediscovered.slave_count; ++j) {
      if (rediscovered.slaves[j].station_position == expected.station_position) {
        actual = &rediscovered.slaves[j];
        break;
      }
    }
    if (actual == nullptr || expected.vendor_id != actual->vendor_id ||
        expected.product_code != actual->product_code ||
        expected.revision != actual->revision ||
        expected.serial_number != actual->serial_number)
      return false;
  }
  return true;
}

constexpr bool ethercat_slave_health_transition_allowed(
    EthercatSlaveHealthState from, EthercatSlaveHealthState to) noexcept {
  if (from == to) return true;
  switch (from) {
    case EthercatSlaveHealthState::Unknown:
      return to == EthercatSlaveHealthState::Present ||
             to == EthercatSlaveHealthState::Lost ||
             to == EthercatSlaveHealthState::Fault;
    case EthercatSlaveHealthState::Present:
      return to == EthercatSlaveHealthState::Lost ||
             to == EthercatSlaveHealthState::Fault;
    case EthercatSlaveHealthState::Lost:
      return to == EthercatSlaveHealthState::Recovering ||
             to == EthercatSlaveHealthState::Fault;
    case EthercatSlaveHealthState::Recovering:
      return to == EthercatSlaveHealthState::Present ||
             to == EthercatSlaveHealthState::Lost ||
             to == EthercatSlaveHealthState::Fault;
    case EthercatSlaveHealthState::Fault:
      return to == EthercatSlaveHealthState::Recovering;
  }
  return false;
}

struct EthercatPdoEntry {
  std::uint16_t slave_position{0U};
  std::uint16_t domain_id{0U};
  std::uint16_t index{0U};
  std::uint8_t subindex{0U};
  std::uint8_t bit_length{0U};
  EthercatPdoDirection direction{EthercatPdoDirection::Input};
  std::uint32_t bit_offset{0U};
};

struct EthercatProcessDataDomain {
  std::uint16_t id{0U};
  std::uint32_t input_bit_size{0U};
  std::uint32_t output_bit_size{0U};
  // Expected WKC is supplied by the master adapter after mapping. It is not
  // derivable from PDO bit lengths alone because datagram/slave access matters.
  std::uint16_t expected_working_counter{0U};
  std::uint32_t cycle_time_ns{0U};
  std::int32_t sync0_shift_ns{0};
  bool distributed_clocks_enabled{false};
};

template <std::size_t MaxDomains, std::size_t MaxEntries>
struct EthercatProcessDataLayout {
  static_assert(MaxDomains > 0U, "EtherCAT domain capacity must be positive");
  static_assert(MaxEntries > 0U, "EtherCAT PDO capacity must be positive");

  std::array<EthercatProcessDataDomain, MaxDomains> domains{};
  std::array<EthercatPdoEntry, MaxEntries> entries{};
  std::size_t domain_count{0U};
  std::size_t entry_count{0U};

  [[nodiscard]] bool valid(std::size_t slave_count) const noexcept {
    if (slave_count == 0U || domain_count == 0U || entry_count == 0U ||
        domain_count > MaxDomains || entry_count > MaxEntries)
      return false;

    for (std::size_t i = 0; i < domain_count; ++i) {
      const auto& domain = domains[i];
      if (domain.input_bit_size == 0U && domain.output_bit_size == 0U)
        return false;
      if (domain.distributed_clocks_enabled && domain.cycle_time_ns == 0U)
        return false;
      for (std::size_t j = 0; j < i; ++j) {
        if (domain.id == domains[j].id) return false;
      }
    }

    for (std::size_t i = 0; i < entry_count; ++i) {
      const auto& entry = entries[i];
      if (entry.slave_position >= slave_count || entry.index == 0U ||
          entry.bit_length == 0U || entry.bit_length > 64U ||
          (entry.direction != EthercatPdoDirection::Input &&
           entry.direction != EthercatPdoDirection::Output))
        return false;
      const EthercatProcessDataDomain* domain = nullptr;
      for (std::size_t j = 0; j < domain_count; ++j) {
        if (domains[j].id == entry.domain_id) {
          domain = &domains[j];
          break;
        }
      }
      if (domain == nullptr) return false;
      const std::uint32_t limit =
          entry.direction == EthercatPdoDirection::Input
              ? domain->input_bit_size
              : domain->output_bit_size;
      if (entry.bit_offset > limit ||
          static_cast<std::uint32_t>(entry.bit_length) > limit - entry.bit_offset)
        return false;

      for (std::size_t j = 0; j < i; ++j) {
        const auto& prior = entries[j];
        if (prior.domain_id != entry.domain_id ||
            prior.direction != entry.direction)
          continue;
        const std::uint32_t entry_end =
            entry.bit_offset + static_cast<std::uint32_t>(entry.bit_length);
        const std::uint32_t prior_end =
            prior.bit_offset + static_cast<std::uint32_t>(prior.bit_length);
        if (entry.bit_offset < prior_end && prior.bit_offset < entry_end)
          return false;
      }
    }
    return true;
  }
};

constexpr EthercatWorkingCounterState classify_ethercat_working_counter(
    std::uint16_t expected, std::uint16_t observed) noexcept {
  if (expected == 0U) return EthercatWorkingCounterState::Unknown;
  if (observed == 0U) return EthercatWorkingCounterState::Zero;
  if (observed < expected) return EthercatWorkingCounterState::Incomplete;
  if (observed == expected) return EthercatWorkingCounterState::Complete;
  return EthercatWorkingCounterState::Unexpected;
}

// EtherCAT AL state is independent from the CiA 402 PDS state. This helper
// describes the master-neutral state-transition contract; adapters remain
// responsible for issuing the protocol-specific state requests and observing
// the resulting AL status/error information.
constexpr bool ethercat_al_transition_allowed(EthercatAlState from,
                                               EthercatAlState to) noexcept {
  if (from == to) return true;
  switch (from) {
    case EthercatAlState::Init:
      return to == EthercatAlState::PreOperational ||
             to == EthercatAlState::Bootstrap || to == EthercatAlState::Error;
    case EthercatAlState::PreOperational:
      return to == EthercatAlState::SafeOperational ||
             to == EthercatAlState::Init || to == EthercatAlState::Error;
    case EthercatAlState::SafeOperational:
      return to == EthercatAlState::Operational ||
             to == EthercatAlState::PreOperational ||
             to == EthercatAlState::Init || to == EthercatAlState::Error;
    case EthercatAlState::Operational:
      return to == EthercatAlState::SafeOperational ||
             to == EthercatAlState::PreOperational ||
             to == EthercatAlState::Init || to == EthercatAlState::Error;
    case EthercatAlState::Bootstrap:
      return to == EthercatAlState::Init || to == EthercatAlState::Error;
    case EthercatAlState::Error:
      return to == EthercatAlState::Init;
    case EthercatAlState::Unknown:
      return false;
  }
  return false;
}

// DC health is deliberately modeled separately from AL and CiA 402 state.
// Unsupported is terminal for a configured domain; a configured clock starts
// synchronization, reaches Locked only after the adapter reports convergence,
// and can degrade or be lost without implying an AL transition.
constexpr bool ethercat_dc_transition_allowed(EthercatDcState from,
                                               EthercatDcState to) noexcept {
  if (from == to) return true;
  switch (from) {
    case EthercatDcState::Unsupported:
      return false;
    case EthercatDcState::NotConfigured:
      return to == EthercatDcState::Synchronizing;
    case EthercatDcState::Synchronizing:
      return to == EthercatDcState::Locked ||
             to == EthercatDcState::Lost ||
             to == EthercatDcState::Degraded ||
             to == EthercatDcState::NotConfigured;
    case EthercatDcState::Locked:
      return to == EthercatDcState::Synchronizing ||
             to == EthercatDcState::Degraded || to == EthercatDcState::Lost ||
             to == EthercatDcState::NotConfigured;
    case EthercatDcState::Lost:
      return to == EthercatDcState::Synchronizing ||
             to == EthercatDcState::NotConfigured;
    case EthercatDcState::Degraded:
      return to == EthercatDcState::Locked ||
             to == EthercatDcState::Synchronizing ||
             to == EthercatDcState::Lost || to == EthercatDcState::NotConfigured;
  }
  return false;
}

// The timing contract is master-neutral: the adapter reports the deadline and
// completion timestamps it observed, plus the number of schedule slots missed
// since the previous exchange. No protocol-specific lateness threshold is
// invented here; equality with the deadline is on-time, completion after it is
// late, and one or more missed slots makes the cycle missed.
constexpr EthercatCycleTimingState classify_ethercat_cycle_timing(
    std::uint64_t deadline_timestamp_ns, std::uint64_t completion_timestamp_ns,
    std::uint32_t missed_cycles_since_previous_exchange) noexcept {
  if (deadline_timestamp_ns == 0U || completion_timestamp_ns == 0U)
    return EthercatCycleTimingState::Unknown;
  if (missed_cycles_since_previous_exchange != 0U)
    return EthercatCycleTimingState::Missed;
  if (completion_timestamp_ns <= deadline_timestamp_ns)
    return EthercatCycleTimingState::OnTime;
  return EthercatCycleTimingState::Late;
}

enum class EthercatSdoOperation : std::uint8_t { Read, Write };

enum class EthercatMailboxResultState : std::uint8_t {
  Pending,
  Completed,
  Aborted,
  TimedOut,
  Failed
};

template <std::size_t MaxBytes>
struct EthercatSdoRequest {
  static_assert(MaxBytes > 0U, "EtherCAT SDO capacity must be positive");

  std::uint64_t request_id{0U};
  std::uint16_t slave_position{0U};
  std::uint16_t index{0U};
  std::uint8_t subindex{0U};
  EthercatSdoOperation operation{EthercatSdoOperation::Read};
  std::array<std::uint8_t, MaxBytes> payload{};
  std::size_t payload_size{0U};
  std::size_t max_response_size{0U};
  std::uint64_t deadline_timestamp_ns{0U};

  [[nodiscard]] bool valid() const noexcept {
    if (request_id == 0U || index == 0U || deadline_timestamp_ns == 0U ||
        payload_size > MaxBytes || max_response_size > MaxBytes)
      return false;
    if (operation == EthercatSdoOperation::Write)
      return payload_size > 0U && max_response_size == 0U;
    if (operation == EthercatSdoOperation::Read)
      return payload_size == 0U && max_response_size > 0U;
    return false;
  }
};

template <std::size_t MaxBytes>
struct EthercatSdoResult {
  static_assert(MaxBytes > 0U, "EtherCAT SDO capacity must be positive");

  std::uint64_t request_id{0U};
  EthercatMailboxResultState state{EthercatMailboxResultState::Pending};
  std::array<std::uint8_t, MaxBytes> payload{};
  std::size_t payload_size{0U};
  std::uint32_t abort_code{0U};

  [[nodiscard]] bool valid() const noexcept {
    if (request_id == 0U || payload_size > MaxBytes) return false;
    if (state == EthercatMailboxResultState::Completed)
      return abort_code == 0U;
    if (state == EthercatMailboxResultState::Aborted)
      return abort_code != 0U && payload_size == 0U;
    if (state == EthercatMailboxResultState::Pending ||
        state == EthercatMailboxResultState::TimedOut ||
        state == EthercatMailboxResultState::Failed)
      return payload_size == 0U;
    return false;
  }
};

// Mailbox work is submitted/polled independently from the cyclic master.
// Implementations must use a fixed-capacity queue, reject duplicate live
// request IDs, enforce deadlines, and return promptly when the queue is full.
// No mailbox operation may be called from or awaited by exchange().
template <std::size_t MaxBytes>
class IEthercatMailbox {
 public:
  using Request = EthercatSdoRequest<MaxBytes>;
  using Result = EthercatSdoResult<MaxBytes>;

  virtual ~IEthercatMailbox() = default;
  virtual core::Status enqueue(const Request& request) noexcept = 0;
  // A successful poll may report Pending. A missing/unknown request ID is an
  // InvalidArgument; output is unchanged on failure.
  virtual core::Status poll(std::uint64_t request_id,
                            Result& output) noexcept = 0;
};

struct EthercatDomainCycleStatus {
  std::uint16_t domain_id{0U};
  std::uint16_t expected_working_counter{0U};
  std::uint16_t observed_working_counter{0U};
  EthercatWorkingCounterState working_counter_state{
      EthercatWorkingCounterState::Unknown};
  std::uint64_t cycle_sequence{0U};
  std::uint64_t timestamp_ns{0U};
  std::uint64_t deadline_timestamp_ns{0U};
  std::uint64_t completion_timestamp_ns{0U};
  std::uint32_t missed_cycles_since_previous_exchange{0U};
  EthercatCycleTimingState timing_state{EthercatCycleTimingState::Unknown};

  [[nodiscard]] bool valid() const noexcept {
    return cycle_sequence != 0U && timestamp_ns != 0U &&
           timing_state == classify_ethercat_cycle_timing(
                                deadline_timestamp_ns,
                                completion_timestamp_ns,
                                missed_cycles_since_previous_exchange) &&
           working_counter_state == classify_ethercat_working_counter(
                                        expected_working_counter,
                                        observed_working_counter);
  }
};

struct EthercatClockStatus {
  EthercatDcState state{EthercatDcState::Unsupported};
  std::uint64_t application_time_ns{0U};
  std::uint64_t reference_time_ns{0U};
  std::int64_t phase_error_ns{0};
  std::uint32_t cycle_time_ns{0U};
};

template <std::size_t MaxBytes>
struct EthercatProcessDataImage {
  static_assert(MaxBytes > 0U, "EtherCAT process image capacity must be positive");

  std::array<std::uint8_t, MaxBytes> bytes{};
  std::size_t size{0U};

  [[nodiscard]] bool valid() const noexcept { return size <= MaxBytes; }
};

template <std::size_t MaxSlaves, std::size_t MaxDomains,
          std::size_t MaxEntries, std::size_t MaxProcessDataBytes>
class IEthercatMaster {
 public:
  using Topology = EthercatTopologySnapshot<MaxSlaves>;
  using Layout = EthercatProcessDataLayout<MaxDomains, MaxEntries>;
  using ProcessImage = EthercatProcessDataImage<MaxProcessDataBytes>;

  virtual ~IEthercatMaster() = default;

  virtual core::Status discover(Topology& output) noexcept = 0;
  virtual core::Status configure(const Layout& layout) noexcept = 0;
  virtual core::Status activate() noexcept = 0;
  virtual core::Status deactivate() noexcept = 0;

  // One bounded cyclic exchange for a single configured domain. On failure,
  // input and cycle_status must remain unchanged. Mailbox/SDO work is separate
  // and must never block this operation.
  virtual core::Status exchange(std::uint16_t domain_id,
                                const ProcessImage& output,
                                ProcessImage& input,
                                EthercatDomainCycleStatus& cycle_status) noexcept = 0;
  virtual core::Status clock_status(EthercatClockStatus& output) const noexcept = 0;
  [[nodiscard]] virtual EthercatMasterState state() const noexcept = 0;
};

}  // namespace mca::hardware
