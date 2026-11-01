#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mca/core/types.hpp"

namespace mca::hardware {

enum class HardwareFaultCategory : std::uint8_t {
  Communication,
  SensorEncoder,
  Drive,
  Overcurrent,
  Overtemperature,
  FollowingError,
  Watchdog,
  EmergencyStop,
  Configuration,
  Unknown
};

// Vendor-neutral fault record. vendor_raw_code is intentionally opaque to the
// core layer so a concrete adapter can preserve manufacturer diagnostics
// without coupling the common HAL to a vendor SDK or error taxonomy.
struct HardwareFault {
  HardwareFaultCategory category{HardwareFaultCategory::Unknown};
  std::uint64_t vendor_raw_code{0};
  bool active{false};
  bool latched{false};
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};

  [[nodiscard]] bool valid() const noexcept {
    return sequence != 0U;
  }
};

template <std::size_t Capacity>
struct HardwareFaultSet {
  static_assert(Capacity > 0, "HardwareFaultSet requires positive capacity");

  std::array<HardwareFault, Capacity> faults{};
  std::size_t count{0};

  [[nodiscard]] bool valid() const noexcept {
    if (count > Capacity) return false;
    for (std::size_t i = 0; i < count; ++i) {
      if (!faults[i].valid()) return false;
    }
    return true;
  }
};

// Diagnostics are deliberately separate from lifecycle/state/command ports.
// read_faults() returns the current backend-owned diagnostic snapshot without
// allocating. clear_faults() may clear only faults whose underlying condition
// is inactive; a backend must leave active safety faults observable.
class IHardwareDiagnostics {
 public:
  virtual ~IHardwareDiagnostics() = default;

  virtual core::Status read_faults(HardwareFaultSet<16>& output) noexcept = 0;
  virtual core::Status clear_faults() noexcept = 0;
};

}  // namespace mca::hardware
