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
      if (i == 0U && (r.assignment_position != 0U ||
          r.pdo_position != 0U || r.entry_position != 0U)) return false;
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
            (prior.sync_manager != r.sync_manager ||
             prior.direction != r.direction)) return false;
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
    if (b.ring_position == a.ring_position &&
        b.assignment_object != a.assignment_object) {
      return a.assignment_object == 0x1C12U &&
          b.assignment_object == 0x1C13U &&
          b.assignment_position == 0U && b.pdo_position == 0U &&
          b.entry_position == 0U;
    }
    if (b.ring_position > a.ring_position)
      return b.assignment_position == 0U && b.pdo_position == 0U &&
          b.entry_position == 0U;
    return false;
  }

  std::array<EthercatPdoReadbackRecord, MaxRecords> records_{};
  std::size_t count_{0U};
};


// Two-layer representation of CoE PDO configuration. Assignment objects
// (0x1C12/0x1C13) are independent from the mapping objects (0x1600..0x1BFF).
// This avoids inferring assignment boundaries from flattened mapping rows.
struct EthercatPdoAssignmentRecord {
  std::uint16_t ring_position{0U};
  std::uint32_t vendor_id{0U};
  std::uint32_t product_code{0U};
  std::uint16_t assignment_object{0U};
  std::uint16_t assignment_position{0U};
  std::uint16_t pdo_index{0U};
};

struct EthercatPdoMappingRecord {
  std::uint16_t ring_position{0U};
  std::uint32_t vendor_id{0U};
  std::uint32_t product_code{0U};
  std::uint16_t assignment_object{0U};
  std::uint16_t assignment_position{0U};
  std::uint16_t pdo_index{0U};
  std::uint8_t sync_manager{0U};
  EthercatPdoDirection direction{EthercatPdoDirection::Input};
  std::uint16_t entry_position{0U};
  std::uint32_t raw_mapping_entry{0U};
};

enum class EthercatPdoSnapshotResult : std::uint8_t {
  Match, Empty, CapacityExceeded, InvalidAssignment, InvalidMapping,
  AssignmentMismatch, MappingMismatch, CardinalityMismatch
};

template <std::size_t MaxAssignments, std::size_t MaxEntries>
class EthercatPdoSnapshot final {
 public:
  [[nodiscard]] bool add_assignment(
      const EthercatPdoAssignmentRecord& record) noexcept {
    if (assignment_count_ >= MaxAssignments) return false;
    assignments_[assignment_count_++] = record;
    return true;
  }

  [[nodiscard]] bool add_mapping(
      const EthercatPdoMappingRecord& record) noexcept {
    if (mapping_count_ >= MaxEntries) return false;
    mappings_[mapping_count_++] = record;
    return true;
  }

  [[nodiscard]] std::size_t assignment_count() const noexcept {
    return assignment_count_;
  }
  [[nodiscard]] std::size_t mapping_count() const noexcept {
    return mapping_count_;
  }
  [[nodiscard]] const EthercatPdoAssignmentRecord& assignment(
      std::size_t i) const noexcept { return assignments_[i]; }
  [[nodiscard]] const EthercatPdoMappingRecord& mapping(
      std::size_t i) const noexcept { return mappings_[i]; }

  [[nodiscard]] bool assignments_valid() const noexcept {
    if (assignment_count_ == 0U) return false;
    for (std::size_t i = 0U; i < assignment_count_; ++i) {
      const auto& r = assignments_[i];
      if ((r.assignment_object != 0x1C12U &&
           r.assignment_object != 0x1C13U) || r.pdo_index == 0U)
        return false;
      if (i == 0U) {
        if (r.assignment_position != 0U) return false;
      } else {
        const auto& p = assignments_[i - 1U];
        if (r.ring_position < p.ring_position) return false;
        if (r.ring_position == p.ring_position) {
          if (r.vendor_id != p.vendor_id || r.product_code != p.product_code)
            return false;
          if (r.assignment_object == p.assignment_object) {
            if (r.assignment_position !=
                static_cast<std::uint16_t>(p.assignment_position + 1U))
              return false;
          } else if (p.assignment_object != 0x1C12U ||
                     r.assignment_object != 0x1C13U ||
                     r.assignment_position != 0U) {
            return false;
          }
        } else if (r.assignment_position != 0U) {
          return false;
        }
      }
      for (std::size_t j = 0U; j < i; ++j) {
        const auto& p = assignments_[j];
        if (p.ring_position == r.ring_position &&
            p.assignment_object == r.assignment_object &&
            p.pdo_index == r.pdo_index) return false;
      }
    }
    return true;
  }

  [[nodiscard]] bool mappings_valid() const noexcept {
    if (mapping_count_ == 0U) return true;
    if (!assignments_valid()) return false;
    for (std::size_t i = 0U; i < mapping_count_; ++i) {
      const auto& r = mappings_[i];
      if (r.assignment_object != 0x1C12U &&
          r.assignment_object != 0x1C13U) return false;
      if ((r.assignment_object == 0x1C12U &&
           r.direction != EthercatPdoDirection::Output) ||
          (r.assignment_object == 0x1C13U &&
           r.direction != EthercatPdoDirection::Input)) return false;
      const auto index = r.raw_mapping_entry >> 16U;
      const auto sub = (r.raw_mapping_entry >> 8U) & 0xFFU;
      const auto width = r.raw_mapping_entry & 0xFFU;
      if (width == 0U || (index == 0U && sub != 0U)) return false;
      bool assignment_found = false;
      for (std::size_t a = 0U; a < assignment_count_; ++a) {
        const auto& assigned = assignments_[a];
        if (assigned.ring_position == r.ring_position &&
            assigned.assignment_object == r.assignment_object &&
            assigned.assignment_position == r.assignment_position &&
            assigned.pdo_index == r.pdo_index &&
            assigned.vendor_id == r.vendor_id &&
            assigned.product_code == r.product_code) {
          assignment_found = true;
          break;
        }
      }
      if (!assignment_found) return false;
      if (i == 0U) {
        if (r.entry_position != 0U) return false;
      } else {
        const auto& p = mappings_[i - 1U];
        if (r.ring_position < p.ring_position) return false;
        if (r.ring_position == p.ring_position) {
          if (r.vendor_id != p.vendor_id || r.product_code != p.product_code)
            return false;
          const bool same_pdo = r.assignment_object == p.assignment_object &&
              r.assignment_position == p.assignment_position &&
              r.pdo_index == p.pdo_index;
          if (same_pdo) {
            if (r.sync_manager != p.sync_manager ||
                r.direction != p.direction ||
                r.entry_position !=
                    static_cast<std::uint16_t>(p.entry_position + 1U))
              return false;
          } else if (r.entry_position != 0U) {
            return false;
          }
        } else if (r.entry_position != 0U) {
          return false;
        }
      }
      for (std::size_t j = 0U; j < i; ++j) {
        const auto& p = mappings_[j];
        if (p.ring_position == r.ring_position &&
            p.assignment_object == r.assignment_object &&
            p.assignment_position == r.assignment_position &&
            p.pdo_index == r.pdo_index &&
            p.raw_mapping_entry == r.raw_mapping_entry &&
            (r.raw_mapping_entry >> 16U) != 0U) return false;
      }
    }
    return true;
  }

  template <std::size_t OA, std::size_t OE>
  [[nodiscard]] EthercatPdoSnapshotResult compare(
      const EthercatPdoSnapshot<OA, OE>& other) const noexcept {
    if (assignment_count_ == 0U || other.assignment_count() == 0U)
      return EthercatPdoSnapshotResult::Empty;
    if (!assignments_valid() || !other.assignments_valid())
      return EthercatPdoSnapshotResult::InvalidAssignment;
    if (!mappings_valid() || !other.mappings_valid())
      return EthercatPdoSnapshotResult::InvalidMapping;
    if (assignment_count_ != other.assignment_count() ||
        mapping_count_ != other.mapping_count())
      return EthercatPdoSnapshotResult::CardinalityMismatch;
    for (std::size_t i = 0U; i < assignment_count_; ++i) {
      const auto& a = assignments_[i];
      const auto& b = other.assignment(i);
      if (a.ring_position != b.ring_position || a.vendor_id != b.vendor_id ||
          a.product_code != b.product_code ||
          a.assignment_object != b.assignment_object ||
          a.assignment_position != b.assignment_position ||
          a.pdo_index != b.pdo_index)
        return EthercatPdoSnapshotResult::AssignmentMismatch;
    }
    for (std::size_t i = 0U; i < mapping_count_; ++i) {
      const auto& a = mappings_[i];
      const auto& b = other.mapping(i);
      if (a.ring_position != b.ring_position || a.vendor_id != b.vendor_id ||
          a.product_code != b.product_code ||
          a.assignment_object != b.assignment_object ||
          a.assignment_position != b.assignment_position ||
          a.pdo_index != b.pdo_index || a.sync_manager != b.sync_manager ||
          a.direction != b.direction || a.entry_position != b.entry_position ||
          a.raw_mapping_entry != b.raw_mapping_entry)
        return EthercatPdoSnapshotResult::MappingMismatch;
    }
    return EthercatPdoSnapshotResult::Match;
  }

 private:
  std::array<EthercatPdoAssignmentRecord, MaxAssignments> assignments_{};
  std::array<EthercatPdoMappingRecord, MaxEntries> mappings_{};
  std::size_t assignment_count_{0U};
  std::size_t mapping_count_{0U};
};

}  // namespace mca::hardware
