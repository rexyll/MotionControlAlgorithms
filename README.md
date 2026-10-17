# MotionControlAlgorithms

A cross-platform, layered C++ motion-control laboratory and reusable control framework for robotics.

## Goals

The platform unifies motion-control algorithms across:

- Actuator / motor servo systems
- Wheel robots / AGV / AMR
- Industrial manipulators
- UAVs
- Legged robots / quadrupeds / humanoids
- Floating-base systems
- Multi-robot systems
- Optimization-based control, MPC and WBC

Every algorithm is intended to follow the same engineering loop:

**mathematical model → C++/Eigen implementation → unit test → simulation adapter → ROS 2 adapter → benchmark → acceptance test**

## Design principles

1. **Algorithms are domain-independent.** Controllers operate on typed state/command/model interfaces instead of ROS 2 or Gazebo types.
2. **Models are explicit.** Kinematics, dynamics, actuator and vehicle models are first-class modules.
3. **Simulation and hardware are adapters.** Gazebo, ROS 2 and real hardware never leak into the algorithm core.
4. **Composition over inheritance.** A robot controller is assembled from estimators, trajectory generators, feedback controllers, feedforward terms, constraints and allocators.
5. **Real-time boundaries are explicit.** No ROS 2 callbacks, blocking I/O, uncontrolled allocation or logging are required by the control core.
6. **Deterministic testing comes first.** Every controller has analytical/reference tests and scenario benchmarks.
7. **One abstraction, many robots.** Applications should select an algorithm through a stable interface and configuration rather than duplicating implementations.

## Target stack

- C++17
- CMake
- Eigen3
- GoogleTest
- ROS 2 Jazzy (optional integration layer)
- Gazebo Harmonic (optional simulation layer)
- Python is reserved for analysis, plotting and experiment tooling

## High-level architecture

```text
Application / Robot Scenario
        |
        v
+-----------------------------+
| Scenario / Robot Adapters   |
| arm | wheel | uav | legged  |
+-----------------------------+
        |
        v
+-----------------------------+
| Control Composition Layer    |
| trajectory | feedback | FF   |
| constraints | allocation     |
+-----------------------------+
        |
        v
+-----------------------------+
| Algorithm Layer              |
| PID | LQR | DOB | MPC | ... |
| IK | Jacobian | dynamics ... |
+-----------------------------+
        |
        v
+-----------------------------+
| Model Layer                  |
| rigid body | actuator |      |
| vehicle | floating base      |
+-----------------------------+
        |
        v
+-----------------------------+
| Core Math / Runtime          |
| Eigen | geometry | filters   |
| realtime data | clock        |
+-----------------------------+
        ^
        |
+-----------------------------+
| Ports / Adapters             |
| ROS2 | Gazebo | hardware     |
| log | replay | benchmark     |
+-----------------------------+
```

## Repository status

The repository is intentionally being built in stages. The authoritative implementation plan is [`docs/TASKPLAN.md`](docs/TASKPLAN.md), and architecture decisions are recorded in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).
