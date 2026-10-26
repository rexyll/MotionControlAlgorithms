#pragma once

#include "mca/simulation/state_command.hpp"
#include "mca/simulation/step.hpp"

namespace mca::simulation {

// Dependency-free boundary for an optional ROS 2 integration package.
// This header deliberately includes no ROS 2 headers or message types. A
// concrete adapter translates between these value types and ROS 2 messages.
template <typename Scalar, std::size_t Dimensions>
class IRos2SimulationAdapter {
 public:
  using State = SimulationState<Scalar, Dimensions>;
  using Command = ActuatorCommand<Scalar, Dimensions>;

  virtual ~IRos2SimulationAdapter() = default;

  // Publish a validated state snapshot associated with the supplied step.
  // Implementations return false on transport/translation failure.
  virtual bool publish_state(const SimulationStep& step,
                             const State& state) noexcept = 0;

  // Read the latest command into output. On false, output must remain
  // unchanged; stale-command policy belongs to the concrete adapter/runtime.
  virtual bool read_command(Command& output) noexcept = 0;
};

using IScalarRos2SimulationAdapter = IRos2SimulationAdapter<double, 1>;

}  // namespace mca::simulation
