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
    if (slave_count == 0U || domain_count == 0U ||
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
          entry.bit_length == 0U || entry.bit_length > 64U)
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

struct EthercatDomainCycleStatus {
  std::uint16_t domain_id{0U};
  std::uint16_t expected_working_counter{0U};
  std::uint16_t observed_working_counter{0U};
  EthercatWorkingCounterState working_counter_state{
      EthercatWorkingCounterState::Unknown};
  std::uint64_t cycle_sequence{0U};
  std::uint64_t timestamp_ns{0U};

  [[nodiscard]] bool valid() const noexcept {
    return cycle_sequence != 0U && timestamp_ns != 0U &&
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
