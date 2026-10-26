#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace mca::simulation {

// Fixed-size simulator state value. The execution layer owns the simulation
// clock; this type carries only physical state values across the adapter
// boundary. Fixed-size storage keeps the value type allocation-free.
template <typename Scalar, std::size_t Dimensions>
struct SimulationState {
  static_assert(Dimensions > 0, "SimulationState requires at least one dimension");

  std::array<Scalar, Dimensions> position{};
  std::array<Scalar, Dimensions> velocity{};
  std::array<Scalar, Dimensions> acceleration{};

  [[nodiscard]] bool finite() const noexcept {
    for (const auto value : position) {
      if (!std::isfinite(value)) return false;
    }
    for (const auto value : velocity) {
      if (!std::isfinite(value)) return false;
    }
    for (const auto value : acceleration) {
      if (!std::isfinite(value)) return false;
    }
    return true;
  }
};

// Fixed-size actuator command value exchanged between the controller side and
// a simulator/plant adapter. It deliberately contains no timing information;
// SimulationStep remains the sole owner of execution time.
template <typename Scalar, std::size_t Dimensions>
struct ActuatorCommand {
  static_assert(Dimensions > 0, "ActuatorCommand requires at least one dimension");

  std::array<Scalar, Dimensions> effort{};

  [[nodiscard]] bool finite() const noexcept {
    for (const auto value : effort) {
      if (!std::isfinite(value)) return false;
    }
    return true;
  }
};

using ScalarSimulationState = SimulationState<double, 1>;
using ScalarActuatorCommand = ActuatorCommand<double, 1>;

}  // namespace mca::simulation
