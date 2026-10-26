# P10 — Simulation and runtime adapter contracts

P10 builds the simulator-independent execution contract before adding ROS 2 or Gazebo adapters. The core remains independent of ROS 2, Gazebo, vendor SDKs, filesystem, and networking.

## Staged tasks

- [x] P10-01 deterministic simulation step/clock contract: add `SimulationStep` and `SimulationClock`; validate finite non-negative time and positive fixed `dt`; make reset/configuration/advance transactional; add deterministic and overflow regression coverage.
- [x] P10-02 simulation lifecycle contract: define a simulator-independent participant interface with transactional reset and one externally supplied `SimulationStep` per execution call; configuration remains component-specific and transactional; add contract-level regression coverage.
- [x] P10-03 unified timestep contract: make the single-axis plant/controller/experiment boundary consume one explicit `SimulationStep`; the experiment owns no competing clock and forwards the same `dt` to PID and plant.
- [x] P10-04 simulation state/command boundary: introduce explicit value types for simulator state and actuator command exchange without coupling to ROS 2/Gazebo.
- [x] P10-05 replay/log boundary: define deterministic record/replay value types and a core-neutral logging port.
- [x] P10-06 ROS 2 adapter minimum contract: dependency-free interface and fake-adapter tests pass corrected Linux/Windows CI (Run #1046); no concrete ROS 2 runtime integration is claimed.
- [x] P10-07 Gazebo adapter minimum contract: dependency-free interface and fake-adapter tests pass corrected Linux/Windows CI (Run #1046); no concrete Gazebo plugin/runtime integration is claimed.
- [x] P10-08 single-axis end-to-end simulation acceptance: deterministic in-process closed-loop test covers experiment timestep, state/command adapter boundaries, replay/log behavior, and snapshot failure atomicity; Linux/Windows CI passed in Run #1046.

## P10-01 acceptance

`SimulationStep` rejects negative/non-finite time and non-positive/non-finite `dt`. `SimulationClock` rejects invalid configuration, does not mutate state on failed configuration or overflowing advance, resets to a deterministic zero-time/zero-index baseline, and advances by exactly the configured fixed step. Tests run through the existing core test target on Linux and Windows.

No ROS 2 or Gazebo environment is required for P10-01.

## P10-02 acceptance

`ISimulationParticipant` provides the common simulator-independent execution boundary: configuration remains component-specific, `reset()` establishes a deterministic configured baseline, and `step()` consumes exactly one validated `SimulationStep` supplied by the execution owner. Participants must not maintain a competing simulation clock, and failed reset/step operations must not expose partial runtime state. The contract and a fake participant regression are compiled through the existing core test target on Linux and Windows.

No ROS 2 or Gazebo environment is required for P10-02.

## P10-03 acceptance

`SingleAxisExperiment::step()` consumes one externally supplied `SimulationStep`, requires the configured fixed `dt` and monotonic `step_index`, samples the trajectory from the supplied simulation time, and forwards the same `dt` to both the PID controller and second-order plant. Failed or out-of-order steps are rejected without exposing partial experiment state. `run()` is implemented through the same per-step boundary, so batch execution and externally driven execution share one timestep contract. Regression tests cover fixed-`dt` enforcement, step ordering, and non-mutation on rejected input.

No ROS 2 or Gazebo environment is required for P10-03.


## P10-04 acceptance

`SimulationState<Scalar, Dimensions>` and `ActuatorCommand<Scalar, Dimensions>` are fixed-size value types with finite-value validation. They carry no competing timing fields; `SimulationStep` remains the authoritative time boundary. Scalar aliases are exported from the umbrella header. Linux and Windows CI pass the regression tests for finite/non-finite values and the single-axis aliases.

No ROS 2 or Gazebo environment is required for P10-04.

## P10-05 acceptance

`SimulationRecord` combines one authoritative `SimulationStep`, physical state, and actuator command. The simulator-independent log sink accepts one valid record transactionally; the replay source defines reset, sequential read, end-of-stream, and output-preservation semantics. Core interfaces do not depend on filesystems, networking, or a concrete log format. Tests cover invalid records, sequential-index rejection in a fixed-buffer adapter, exact replay, end-of-stream preservation, and reset-to-first-record behavior.

No ROS 2 or Gazebo environment is required for P10-05.


## P10-06 acceptance

`IRos2SimulationAdapter` is a dependency-free contract for an optional ROS 2 integration package. It exchanges the core's validated `SimulationStep`, `SimulationState`, and `ActuatorCommand` values while importing no `rclcpp`, ROS message, or middleware headers. State publication rejects invalid input; command reads preserve the caller's output when no command is available. Fake-adapter regressions run on Linux and Windows. This validates the adapter boundary only; a concrete ROS 2 package and runtime integration remain outside this task.

No local ROS 2/Gazebo run is required for P10-06.


## P10-07 acceptance

`IGazeboSimulationAdapter` defines a dependency-free state-read/command-write boundary. A state snapshot and its `SimulationStep` are read together; failed reads preserve both output arguments. Command writes reject invalid steps and non-finite commands. The core header imports no Gazebo or ROS 2 types; a fake adapter validates the contract on Linux and Windows. This is an adapter contract, not a claim that a concrete Gazebo plugin has been implemented or run.

No local Gazebo run is required for P10-07.


## P10-06 CI evidence

Runs #1027–#1030 passed on the original test target, but a later audit found the adapter test source was not registered then. Corrected acceptance evidence is Run #1046: both Linux and Windows passed Configure, Build, and Test with `tests/simulation/step_test.cpp` registered. Scope remains the dependency-free contract and fake adapter, not a concrete ROS 2 node/package or live middleware integration.

## P10-07 CI evidence

Runs #1031–#1035 passed on the original test target, but a later audit found the adapter test source was not registered then. Corrected acceptance evidence is Run #1046: both Linux and Windows passed Configure, Build, and Test with `tests/simulation/step_test.cpp` registered. Scope remains the dependency-free adapter contract and fake adapter, not a concrete Gazebo plugin or live runtime test.



## Acceptance correction — test target registration

A follow-up audit on 2026-10-26 found that `tests/simulation/step_test.cpp` had not been registered in `mca_core_tests`. The source was registered in CMake (commit `918de3c7`); the initial corrected runs exposed assertion-type mismatches and duplicate test names, which were fixed. Run #1046 (commit `ce549fef`) passed Configure, Build, and Test on both Linux and Windows with the corrected target. P10-06/P10-07 are accepted as dependency-free adapter contracts only, and P10-08 is accepted for deterministic in-process closed-loop, adapter-boundary, replay, and failure-atomicity coverage. This does not claim concrete ROS 2 or Gazebo runtime integration. No local Ubuntu/Gazebo run is required for these acceptance criteria.


## P10-08 acceptance

The registered `SingleAxisSimulationAcceptance` regressions execute a short closed loop through `SingleAxisExperiment`, publish state through the fake ROS 2 port, write actuator commands through the fake Gazebo port, record/replay sequential `SimulationRecord` values, and check snapshot failure output preservation. The test target now actually compiles this source. Run #1046 passed Configure, Build, and Test on Linux and Windows. This accepts deterministic core-contract behavior only; live middleware/simulator integration remains future work if desired.
