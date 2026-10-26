# P10 — Simulation and runtime adapter contracts

P10 builds the simulator-independent execution contract before adding ROS 2 or Gazebo adapters. The core remains independent of ROS 2, Gazebo, vendor SDKs, filesystem, and networking.

## Staged tasks

- [x] P10-01 deterministic simulation step/clock contract: add `SimulationStep` and `SimulationClock`; validate finite non-negative time and positive fixed `dt`; make reset/configuration/advance transactional; add deterministic and overflow regression coverage.
- [x] P10-02 simulation lifecycle contract: define a simulator-independent participant interface with transactional reset and one externally supplied `SimulationStep` per execution call; configuration remains component-specific and transactional; add contract-level regression coverage.
- [x] P10-03 unified timestep contract: make the single-axis plant/controller/experiment boundary consume one explicit `SimulationStep`; the experiment owns no competing clock and forwards the same `dt` to PID and plant.
- [ ] P10-04 simulation state/command boundary: introduce explicit value types for simulator state and actuator command exchange without coupling to ROS 2/Gazebo.
- [ ] P10-05 replay/log boundary: define deterministic record/replay value types and a core-neutral logging port.
- [ ] P10-06 ROS 2 adapter minimum contract: add optional adapter interfaces without making ROS 2 a core dependency.
- [ ] P10-07 Gazebo adapter minimum contract: add optional simulator adapter interfaces without making Gazebo a core dependency.
- [ ] P10-08 single-axis end-to-end simulation acceptance: connect controller → plant → experiment through the new contract; use Gazebo only if the deterministic adapter acceptance cannot cover the required integration boundary.

## P10-01 acceptance

`SimulationStep` rejects negative/non-finite time and non-positive/non-finite `dt`. `SimulationClock` rejects invalid configuration, does not mutate state on failed configuration or overflowing advance, resets to a deterministic zero-time/zero-index baseline, and advances by exactly the configured fixed step. Tests run through the existing core test target on Linux and Windows.

No ROS 2 or Gazebo environment is required for P10-01.

## P10-02 acceptance

`ISimulationParticipant` provides the common simulator-independent execution boundary: configuration remains component-specific, `reset()` establishes a deterministic configured baseline, and `step()` consumes exactly one validated `SimulationStep` supplied by the execution owner. Participants must not maintain a competing simulation clock, and failed reset/step operations must not expose partial runtime state. The contract and a fake participant regression are compiled through the existing core test target on Linux and Windows.

No ROS 2 or Gazebo environment is required for P10-02.

## P10-03 acceptance

`SingleAxisExperiment::step()` consumes one externally supplied `SimulationStep`, requires the configured fixed `dt` and monotonic `step_index`, samples the trajectory from the supplied simulation time, and forwards the same `dt` to both the PID controller and second-order plant. Failed or out-of-order steps are rejected without exposing partial experiment state. `run()` is implemented through the same per-step boundary, so batch execution and externally driven execution share one timestep contract. Regression tests cover fixed-`dt` enforcement, step ordering, and non-mutation on rejected input.

No ROS 2 or Gazebo environment is required for P10-03.
