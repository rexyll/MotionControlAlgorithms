#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace mca::kinematics {

// Proportionally scales a wheel-command vector to satisfy per-wheel limits.
// The common scale factor preserves the requested motion direction.
template <std::size_t WheelCount>
class WheelCommandAllocator {
  static_assert(WheelCount > 0, "WheelCount must be positive");

 public:
  using WheelVector = std::array<double, WheelCount>;

  bool allocate(const WheelVector& requested, const WheelVector& absolute_limits,
                WheelVector& allocated, double& scale_factor) const noexcept {
    double scale = 1.0;
    for (std::size_t i = 0; i < WheelCount; ++i) {
      if (!std::isfinite(requested[i]) ||
          !std::isfinite(absolute_limits[i]) || absolute_limits[i] <= 0.0) {
        return false;
      }
      const double magnitude = std::abs(requested[i]);
      if (magnitude > absolute_limits[i]) {
        scale = std::min(scale, absolute_limits[i] / magnitude);
      }
    }

    WheelVector result{};
    for (std::size_t i = 0; i < WheelCount; ++i) {
      result[i] = requested[i] * scale;
      if (!std::isfinite(result[i])) return false;
    }
    allocated = result;
    scale_factor = scale;
    return true;
  }
};

}  // namespace mca::kinematics
