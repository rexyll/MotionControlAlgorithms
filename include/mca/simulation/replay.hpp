#pragma once

#include "mca/simulation/state_command.hpp"
#include "mca/simulation/step.hpp"

namespace mca::simulation {

// One deterministic sample at the adapter boundary. The step is the sole
// source of timing; state and command values intentionally carry no timestamp.
template <typename Scalar, std::size_t Dimensions>
struct SimulationRecord {
  SimulationStep step{};
  SimulationState<Scalar, Dimensions> state{};
  ActuatorCommand<Scalar, Dimensions> command{};

  [[nodiscard]] bool valid() const noexcept {
    return step.valid() && state.finite() && command.finite();
  }
};

// Core-neutral logging port. Implementations may target memory, files, or
// external systems, but those dependencies must remain outside mca_core.
template <typename Scalar, std::size_t Dimensions>
class ISimulationLogSink {
 public:
  using Record = SimulationRecord<Scalar, Dimensions>;
  virtual ~ISimulationLogSink() = default;
  // Append exactly one valid record; on failure, the sink must not partially
  // commit that record.
  virtual bool append(const Record& record) noexcept = 0;
};

// Sequential replay port. reset() returns to the first record. next() returns
// false at end-of-stream or on failure and must leave output unchanged then.
template <typename Scalar, std::size_t Dimensions>
class ISimulationReplaySource {
 public:
  using Record = SimulationRecord<Scalar, Dimensions>;
  virtual ~ISimulationReplaySource() = default;
  virtual bool reset() noexcept = 0;
  virtual bool next(Record& output) noexcept = 0;
};

using ScalarSimulationRecord = SimulationRecord<double, 1>;
using IScalarSimulationLogSink = ISimulationLogSink<double, 1>;
using IScalarSimulationReplaySource = ISimulationReplaySource<double, 1>;

}  // namespace mca::simulation
