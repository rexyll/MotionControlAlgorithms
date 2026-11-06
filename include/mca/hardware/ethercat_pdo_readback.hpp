#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mca/hardware/ethercat_adapter_state.hpp"

namespace mca::hardware {

// One CoE 0x1C12/0x1C13 assignment -> PDO -> mapping-entry observation.
// raw_mapping_entry is the exact 32-bit value read from 0x1600..0x1BFF:
// index[31:16], subindex[15:8], bit length[7:0]. Keeping the raw word
// preserves legal padding entries (index=0, subindex=0, nonzero width).
struct EthercatPdoReadbackRecord {
  std::uint16_t ring_position{0U};
  std::uint32_t vendor_id{0U};
  std::uint32_t product_code{0U};
  std::uint8_t sync_manager{0U};
  EthercatPdoDirection direction{EthercatPdoDirection::Input};
  std::uint16_t assignment_object{0U};  // 0x1C12 RxPDO or 0x1C13 TxPDO
  std::uint16_t assignment_position{0U};
  std::uint16_t pdo_index{0U};
  std::uint16_t pdo_position{0U};
  std::uint16_t entry_position{0U};
  std::uint32_t raw_mapping_entry{0U};

  [[nodiscard]] constexpr std::uint16_t object_index() const noexcept {
    return static_cast<std::uint16_t>(raw_mapping_entry >> 16U);
  }
  [[nodiscard]] constexpr std::uint8_t object_subindex() const noexcept {
    return static_cast<std::uint8_t>((raw_mapping_entry >> 8U) & 0xFFU);
  }
  [[nodiscard]] constexpr std::uint8_t bit_length() const noexcept {
    return static_cast<std::uint8_t>(raw_mapping_entry & 0xFFU);
  }
  [[nodiscard]] constexpr bool padding() const noexcept {
    return object_index() == 0U && object_subindex() == 0U;
  }
};

enum class EthercatPdoParityResult : std::uint8_t {
  Match,
  Empty,
  CapacityExceeded,
  InvalidRecord,
  CardinalityMismatch,
  RecordMismatch
};

// Bounded, allocation-free ordered CoE mapping snapshot. A row is one mapping
// entry; PDO/assignment ordering is represented explicitly by the position
// fields and is never normalized by sorting.
template <std::size_t MaxRecords>
class EthercatPdoReadback final {
 public:
  [[nodiscard]] bool add(const EthercatPdoReadbackRecord& record) noexcept {
    if (count_ >= MaxRecords) return false;
    records_[count_++] = record;
    return true;
  }

  [[nodiscard]] std::size_t size() const noexcept { return count_; }
  [[nodiscard]] const EthercatPdoReadbackRecord& operator[](
      std::size_t index) const noexcept { return records_[index]; }

  [[nodiscard]] bool valid() const noexcept {
    if (count_ == 0U) return false;
    for (std::size_t i = 0U; i < count_; ++i) {
      const auto& r = records_[i];
      if (r.assignment_object != 0x1C12U &&
          r.assignment_object != 0x1C13U) return false;
      if ((r.assignment_object == 0x1C12U &&
           r.direction != EthercatPdoDirection::Output) ||
          (r.assignment_object == 0x1C13U &&
           r.direction != EthercatPdoDirection::Input)) return false;
      if (r.pdo_index == 0U || r.bit_length() == 0U) return false;
      if (r.object_index() == 0U && r.object_subindex() != 0U) return false;

      if (i != 0U && !ordered_after(records_[i - 1U], r)) return false;
      for (std::size_t j = 0U; j < i; ++j) {
        const auto& prior = records_[j];
        if (prior.ring_position == r.ring_position &&
            prior.assignment_object == r.assignment_object &&
            prior.assignment_position == r.assignment_position &&
            prior.pdo_position == r.pdo_position) {
          if (prior.pdo_index != r.pdo_index) return false;
          // Duplicate real objects in one PDO are ambiguous. Padding entries
          // are intentionally repeatable and retain their exact bit widths.
          if (!r.padding() && prior.object_index() == r.object_index() &&
              prior.object_subindex() == r.object_subindex()) return false;
        }
        if (prior.ring_position == r.ring_position &&
            prior.assignment_object == r.assignment_object &&
            prior.assignment_position == r.assignment_position &&
            prior.pdo_position != r.pdo_position &&
            prior.pdo_index == r.pdo_index) return false;
      }
    }
    return true;
  }

  template <std::size_t OtherCapacity>
  [[nodiscard]] EthercatPdoParityResult compare(
      const EthercatPdoReadback<OtherCapacity>& observed) const noexcept {
    if (count_ == 0U || observed.size() == 0U)
      return EthercatPdoParityResult::Empty;
    if (!valid() || !observed.valid())
      return EthercatPdoParityResult::InvalidRecord;
    if (count_ != observed.size())
      return EthercatPdoParityResult::CardinalityMismatch;
    for (std::size_t i = 0U; i < count_; ++i) {
      if (!same_record(records_[i], observed[i]))
        return EthercatPdoParityResult::RecordMismatch;
    }
    return EthercatPdoParityResult::Match;
  }

 private:
  static constexpr bool same_record(const EthercatPdoReadbackRecord& a,
                                    const EthercatPdoReadbackRecord& b) noexcept {
    return a.ring_position == b.ring_position &&
        a.vendor_id == b.vendor_id && a.product_code == b.product_code &&
        a.sync_manager == b.sync_manager && a.direction == b.direction &&
        a.assignment_object == b.assignment_object &&
        a.assignment_position == b.assignment_position &&
        a.pdo_index == b.pdo_index && a.pdo_position == b.pdo_position &&
        a.entry_position == b.entry_position &&
        a.raw_mapping_entry == b.raw_mapping_entry;
  }

  static constexpr bool ordered_after(const EthercatPdoReadbackRecord& a,
                                      const EthercatPdoReadbackRecord& b) noexcept {
    if (b.ring_position < a.ring_position) return false;
    if (b.ring_position == a.ring_position &&
        (b.vendor_id != a.vendor_id || b.product_code != a.product_code))
      return false;
    if (b.ring_position == a.ring_position &&
        b.assignment_object == a.assignment_object &&
        b.assignment_position == a.assignment_position &&
        b.pdo_position == a.pdo_position) {
      return b.entry_position == static_cast<std::uint16_t>(a.entry_position + 1U);
    }
    if (b.ring_position == a.ring_position &&
        b.assignment_object == a.assignment_object &&
        b.assignment_position == a.assignment_position &&
        b.pdo_position > a.pdo_position) {
      return b.pdo_position == static_cast<std::uint16_t>(a.pdo_position + 1U) &&
          b.entry_position == 0U;
    }
    if (b.ring_position == a.ring_position &&
        b.assignment_object == a.assignment_object &&
        b.assignment_position > a.assignment_position) {
      return b.assignment_position ==
          static_cast<std::uint16_t>(a.assignment_position + 1U) &&
          b.pdo_position == 0U && b.entry_position == 0U;
    }
    if (b.ring_position > a.ring_position)
      return b.assignment_position == 0U && b.pdo_position == 0U &&
          b.entry_position == 0U;
    return false;
  }

  std::array<EthercatPdoReadbackRecord, MaxRecords> records_{};
  std::size_t count_{0U};
};

}  // namespace mca::hardware
