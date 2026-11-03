#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mca/hardware/ethercat_adapter_state.hpp"

namespace mca::hardware {

// A bounded, allocation-free declaration of the complete PDO mapping expected
// by the application. Entries must be supplied in wire order: slave, Sync
// Manager, PDO, then PDO entry. This is a configuration profile, not a claim
// that the physical slave currently exposes this mapping.
template <std::size_t MaxEntries>
class EthercatPdoMappingProfile final {
 public:
  struct Entry {
    std::uint16_t station_position{0U};
    std::uint32_t vendor_id{0U};
    std::uint32_t product_code{0U};
    std::uint8_t sync_manager{0U};
    std::uint16_t pdo_index{0U};
    std::uint16_t index{0U};
    std::uint8_t subindex{0U};
    std::uint8_t bit_length{0U};
    EthercatPdoDirection direction{EthercatPdoDirection::Input};
  };

  [[nodiscard]] bool add(const Entry& entry) noexcept {
    if (count_ >= MaxEntries || !entry_valid(entry)) return false;
    for (std::size_t i = 0; i < count_; ++i) {
      const auto& prior = entries_[i];
      if (prior.station_position == entry.station_position &&
          prior.index == entry.index && prior.subindex == entry.subindex)
        return false;
    }
    if (count_ != 0U && !ordered_after(entries_[count_ - 1U], entry))
      return false;
    entries_[count_++] = entry;
    return true;
  }

  [[nodiscard]] bool valid() const noexcept {
    if (count_ == 0U) return false;
    for (std::size_t i = 0; i < count_; ++i) {
      if (!entry_valid(entries_[i])) return false;
      for (std::size_t j = 0; j < i; ++j) {
        if (entries_[j].station_position == entries_[i].station_position &&
            entries_[j].index == entries_[i].index &&
            entries_[j].subindex == entries_[i].subindex)
          return false;
      }
      if (i != 0U && !ordered_after(entries_[i - 1U], entries_[i]))
        return false;
    }
    return true;
  }

  [[nodiscard]] std::size_t size() const noexcept { return count_; }
  [[nodiscard]] const Entry& operator[](std::size_t index) const noexcept {
    return entries_[index];
  }

 private:
  static bool entry_valid(const Entry& e) noexcept {
    return e.pdo_index != 0U && e.index != 0U && e.bit_length != 0U &&
           e.bit_length <= 64U &&
           (e.direction == EthercatPdoDirection::Input ||
            e.direction == EthercatPdoDirection::Output);
  }

  // Enforce monotonic group order and one direction per Sync Manager. A
  // previously closed SM/PDO group may not reappear later in the profile.
  bool ordered_after(const Entry& previous, const Entry& next) const noexcept {
    if (next.station_position < previous.station_position) return false;
    if (next.station_position > previous.station_position) return true;
    if (next.vendor_id != previous.vendor_id ||
        next.product_code != previous.product_code) return false;

    if (next.sync_manager < previous.sync_manager) return false;
    if (next.sync_manager > previous.sync_manager) return true;
    if (next.direction != previous.direction) return false;

    if (next.pdo_index == previous.pdo_index) return true;
    for (std::size_t i = 0; i + 1U < count_; ++i) {
      const auto& seen = entries_[i];
      if (seen.station_position == next.station_position &&
          seen.sync_manager == next.sync_manager &&
          seen.pdo_index == next.pdo_index)
        return false;
    }
    return true;
  }

  std::array<Entry, MaxEntries> entries_{};
  std::size_t count_{0U};
};

}  // namespace mca::hardware
