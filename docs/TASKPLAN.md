# MotionControlAlgorithms Task Plan

## P0 — Platform foundation

- [ ] P0-01 Repository/build system
- [ ] P0-02 Core numeric types and units
- [ ] P0-03 Geometry and linear algebra utilities
- [ ] P0-04 Filters and numerical utilities
- [ ] P0-05 Real-time-safe primitives
- [ ] P0-06 Controller/algorithm interfaces
- [ ] P0-07 Model interfaces
- [ ] P0-08 Configuration and validation
- [ ] P0-09 Test/benchmark framework
- [ ] P0-10 CI on Linux and Windows

Acceptance: clean CMake configure/build/test on Linux and Windows; core has no ROS2/Gazebo dependency.

## P1 — Single-axis and motor servo

- [ ] P1-01 First/second-order plant models
- [ ] P1-02 PID/PI/PD
- [x] P1-03 Cascaded position/velocity/torque loops
- [x] P1-04 DC motor model (Run #70: Linux and Windows CI passed; unit tests, deterministic servo experiment, and docs accepted)
- [x] P1-05 BLDC/PMSM model (Run #79: Linux and Windows CI passed; phase-domain model, reference-oriented unit tests, and docs accepted)
- [x] P1-06 Clarke/Park transforms (Run #86: Linux and Windows CI passed; reference-tested conventions, unit tests, docs, and benchmark accepted)
- [x] P1-07 PMSM dq model (Run #94: Linux and Windows CI passed; dq equations, cross-coupling/back-EMF, torque, mechanical dynamics, pole-pair scaling, unit tests and docs accepted)
- [x] P1-08 FOC current controller (Run #109: Linux and Windows CI passed; dq PI current control, current/voltage/integral limits, conditional anti-windup, closed-loop current tracking, unit tests, benchmark and docs accepted)
- [x] P1-09 decoupling/feedforward (Run #118: Linux and Windows CI passed; PMSM dq decoupling/back-EMF feedforward, analytical/reference tests, independent voltage limits, benchmark and docs accepted)
- [x] P1-10 SVPWM (Run #128: Linux and Windows CI passed; six-sector dwell computation, symmetric zero-vector allocation, linear modulation clamp, overmodulation reporting, unit tests, benchmark and docs accepted)
- [ ] P1-11 friction compensation
- [ ] P1-12 disturbance observer
- [ ] P1-13 state observers
- [ ] P1-14 Step/ramp/sine scalar trajectory generators
- [ ] P1-15 Step-response metrics: overshoot and settling time
- [ ] P1-16 PID/PD + feedforward comparison runner

Acceptance: analytical/reference tests pass; stable current, velocity and position tracking; controller benchmarks generated automatically. Step-response overshoot is normalized by step amplitude; settling uses a configurable tolerance band and hold duration.

## P2 — Wheel robots

- [ ] P2-01 Differential-drive kinematics
- [ ] P2-02 Omni/mecanum kinematics
- [ ] P2-03 Odometry
- [ ] P2-04 Pure Pursuit
- [ ] P2-05 Stanley
- [ ] P2-06 LQR path tracking
- [ ] P2-07 MPC path tracking
- [ ] P2-08 Wheel command allocation
- [ ] P2-09 disturbance/slip simulation

Acceptance: reproducible path tracking benchmark and controller comparison.

## P3 — Manipulators

- [ ] P3-01 SE(3)/SO(3) utilities
- [ ] P3-02 DH/URDF model adapter
- [ ] P3-03 Forward kinematics
- [ ] P3-04 Numerical IK
- [ ] P3-05 DLS IK
- [ ] P3-06 Jacobian
- [ ] P3-07 Singularity metrics
- [ ] P3-08 Cubic/quintic trajectory
- [ ] P3-09 S-curve trajectory
- [ ] P3-10 rigid-body dynamics
- [ ] P3-11 gravity compensation
- [ ] P3-12 computed torque
- [ ] P3-13 Cartesian control
- [ ] P3-14 impedance control
- [ ] P3-15 admittance control
- [ ] P3-16 redundancy/null-space control

Acceptance: FK/IK/Jacobian numerical validation; joint and Cartesian tracking benchmarks; contact-control scenario.

## P4 — UAV

- [ ] P4-01 6DOF rigid-body model
- [ ] P4-02 attitude representations
- [ ] P4-03 cascaded position/attitude PID
- [ ] P4-04 geometric attitude control
- [ ] P4-05 LQR
- [ ] P4-06 MPC
- [ ] P4-07 thrust/mixer allocation

Acceptance: waypoint, trajectory and disturbance benchmarks.

## P5 — Floating-base / legged robots

- [ ] P5-01 floating-base state model
- [ ] P5-02 contact model
- [ ] P5-03 foot trajectory generation
- [ ] P5-04 gait scheduler
- [ ] P5-05 centroidal dynamics
- [ ] P5-06 whole-body control
- [ ] P5-07 QP task hierarchy
- [ ] P5-08 joint impedance/torque control
- [ ] P5-09 locomotion MPC

Acceptance: standing, walking and disturbance-recovery simulations.

## P6 — Humanoid / whole-body

- [ ] P6-01 multi-contact model
- [ ] P6-02 task-space hierarchy
- [ ] P6-03 contact wrench optimization
- [ ] P6-04 balance / ZMP / centroidal control
- [ ] P6-05 whole-body QP
- [ ] P6-06 manipulation while walking

Acceptance: multi-contact benchmark with constraint and balance checks.

## P7 — Multi-robot control

- [ ] P7-01 consensus
- [ ] P7-02 formation control
- [ ] P7-03 leader-follower
- [ ] P7-04 distributed optimization
- [ ] P7-05 distributed MPC
- [ ] P7-06 communication delay/loss model

Acceptance: formation and degraded-communication benchmarks.

## P8 — Advanced optimization/control

- [ ] P8-01 Linearization framework
- [ ] P8-02 LQR/LQG
- [ ] P8-03 nonlinear MPC
- [ ] P8-04 constrained QP infrastructure
- [ ] P8-05 generic MPC interface
- [ ] P8-06 robust/adaptive control
- [ ] P8-07 sliding-mode control
- [ ] P8-08 ADRC
- [ ] P8-09 optimization-based allocation

Acceptance: common benchmark comparing PID/FF, LQR, DOB, MPC and optimization-based controllers where applicable.

## P9 — ROS 2 and Gazebo integration

- [ ] P9-01 ROS 2 Jazzy package layout
- [ ] P9-02 ros2_control hardware interface
- [ ] P9-03 controller adapters
- [ ] P9-04 Gazebo Harmonic adapters
- [ ] P9-05 launch/config system
- [ ] P9-06 rosbag/replay pipeline
- [ ] P9-07 visualization and diagnostics

Acceptance: each domain can run through ROS2/Gazebo without changing core algorithm code.

## P10 — Engineering hardening

- [ ] P10-01 API/documentation stabilization
- [ ] P10-02 benchmark regression CI
- [ ] P10-03 sanitizer builds
- [ ] P10-04 static analysis
- [ ] P10-05 performance benchmarks
- [ ] P10-06 deterministic replay
- [ ] P10-07 hardware-in-the-loop extension points
- [ ] P10-08 release/versioning policy

## Implementation rule

Do not mark a task complete because code compiles. A task is complete only when its implementation, unit tests, documentation, benchmark and acceptance criteria are present.

## Recommended execution order

P0 → P1 → P2 → P3 → P4 → P5 → P6 → P7 → P8 → P9 → P10.

Cross-domain infrastructure should be implemented once and reused; domain packages should contain only domain-specific models, compositions and scenarios.
