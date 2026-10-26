#pragma once

#include "mca/simulation/state_command.hpp"
#include "mca/simulation/step.hpp"

namespace mca::simulation {

// Dependency-free contract for a future Gazebo integration package.
// Gazebo types and simulator APIs must be confined to the concrete adapter.
template <typename Scalar, std::size_t Dimensions>
class IGazeboSimulationAdapter {
 public:
  using State = SimulationState<Scalar, Dimensions>;
  using Command = ActuatorCommand<Scalar, Dimensions>;

  virtual ~IGazeboSimulationAdapter() = default;

  // Read a coherent simulator snapshot and its authoritative step metadata.
  // On false, both output arguments must remain unchanged.
  virtual bool read_state(SimulationStep& step, State& state) noexcept = 0;

  // Apply one valid command for the supplied step. Concrete adapters define
  // how the step maps to simulator update phases and report failures.
  virtual bool write_command(const SimulationStep& step,
                             const Command& command) noexcept = 0;
};

using IScalarGazeboSimulationAdapter = IGazeboSimulationAdapter<double, 1>;

}  // namespace mca::simulation
