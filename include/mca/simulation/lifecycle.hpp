#pragma once

#include "mca/simulation/step.hpp"

namespace mca::simulation {

// Simulator-independent execution participant contract.
// Configuration remains component-specific and must be transactional.
// Once configured, reset() establishes the deterministic baseline and
// step() consumes exactly one externally supplied SimulationStep.
class ISimulationParticipant {
 public:
  virtual ~ISimulationParticipant() = default;
  virtual bool reset() noexcept = 0;
  virtual bool step(const SimulationStep& simulation_step) noexcept = 0;
};

inline bool valid_step(const SimulationStep& simulation_step) noexcept {
  return simulation_step.valid();
}

}  // namespace mca::simulation
