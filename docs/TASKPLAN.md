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

- [x] P1-01 First/second-order plant models (Run #174: Linux and Windows CI passed; analytical/reference tests, plant documentation, deterministic micro-benchmarks and CMake integration accepted)
- [x] P1-02 PID/PI/PD (Run #186: Linux and Windows CI passed; PID/PI/PD unit/reference tests, saturation and conditional anti-windup, derivative-on-measurement, closed-loop tracking, documentation, deterministic benchmarks and CMake integration accepted)
- [x] P1-03 Cascaded position/velocity/torque loops
- [x] P1-04 DC motor model (Run #70: Linux and Windows CI passed; unit tests, deterministic servo experiment, and docs accepted)
- [x] P1-05 BLDC/PMSM model (Run #79: Linux and Windows CI passed; phase-domain model, reference-oriented unit tests, and docs accepted)
- [x] P1-06 Clarke/Park transforms (Run #86: Linux and Windows CI passed; reference-tested conventions, unit tests, docs, and benchmark accepted)
- [x] P1-07 PMSM dq model (Run #94: Linux and Windows CI passed; dq equations, cross-coupling/back-EMF, torque, mechanical dynamics, pole-pair scaling, unit tests and docs accepted)
- [x] P1-08 FOC current controller (Run #109: Linux and Windows CI passed; dq PI current control, current/voltage/integral limits, conditional anti-windup, closed-loop current tracking, unit tests, benchmark and docs accepted)
- [x] P1-09 decoupling/feedforward (Run #118: Linux and Windows CI passed; PMSM dq decoupling/back-EMF feedforward, analytical/reference tests, independent voltage limits, benchmark and docs accepted)
- [x] P1-10 SVPWM (Run #128: Linux and Windows CI passed; six-sector dwell computation, symmetric zero-vector allocation, linear modulation clamp, overmodulation reporting, unit tests, benchmark and docs accepted)
- [x] P1-11 friction compensation (Run #134: Linux and Windows CI passed; static/Coulomb/viscous compensation, velocity deadband with requested-torque breakaway direction, output limits, unit tests, benchmark accepted)
- [x] P1-12 disturbance observer (Run #143: Linux and Windows CI passed; model-residual disturbance estimation, first-order filtering, compensation, limits, reset/validation tests, benchmark and docs accepted)
- [x] P1-13 state observers (Run #148: Linux and Windows CI passed; second-order mechanical Luenberger observer, position/velocity/acceleration estimation, unit tests, benchmark and docs accepted)
- [x] P1-14 Step/ramp/sine scalar trajectory generators (Run #153: Linux and Windows CI passed; step/ramp/sine analytic samples, edge-case/finite-output tests, benchmark and docs accepted)
- [x] P1-15 Step-response metrics: overshoot and settling time (Run #163: Linux and Windows CI passed; normalized direction-aware overshoot, configurable tolerance/hold settling time, validation and nonuniform-sampling tests, benchmark and docs accepted)
- [x] P1-16 PID/PD + feedforward comparison runner (Run #170: Linux and Windows CI passed; deterministic same-plant/reference comparison, RMSE and control-effort comparison, overshoot/settling metrics, unit tests, benchmark and docs accepted)

Acceptance: analytical/reference tests pass; stable current, velocity and position tracking; controller benchmarks generated automatically. Step-response overshoot is normalized by step amplitude; settling uses a configurable tolerance band and hold duration.

## P2 — Wheel robots

- [x] P2-01 Differential-drive kinematics (Run #186: Linux and Windows CI passed; forward/inverse wheel-speed mapping, geometry/input validation, unit tests, benchmark, documentation and CMake integration accepted)
- [x] P2-02 Omni/mecanum kinematics (Run #193: Linux and Windows CI passed; standard X-roller mecanum forward/inverse mapping, geometry/input validation, unit tests, benchmark, documentation and CMake integration accepted)
- [x] P2-03 Odometry (Run #207: Linux and Windows CI passed; exact SE(2) constant-twist integration, validation/transactional failure behavior, tests, benchmark, docs and CMake integration accepted)
- [x] P2-04 Pure Pursuit (Run #207: Linux and Windows CI passed; lookahead interpolation, vehicle-frame geometry, curvature/yaw-rate command, invalid-input handling, closed-loop lateral convergence, tests, benchmark, docs and CMake integration accepted)
- [x] P2-05 Stanley (Run #213: Linux and Windows CI passed; nearest-segment projection, steering saturation, signed cross-track convention, closed-loop convergence, tests, benchmark, docs and CMake integration accepted)
- [x] P2-06 LQR path tracking (Run #220: Linux and Windows CI passed; discrete-time kinematic-bicycle LQR with configuration-time Riccati iteration, steering saturation, nearest-segment geometry, closed-loop convergence tests, benchmark, docs and CMake integration accepted)
- [x] P2-07 MPC path tracking (Run #226: Linux and Windows Configure/Build/Test passed; finite-horizon linearized kinematic-bicycle MPC, projected-gradient steering optimization, closed-loop convergence tests, benchmark, docs and CMake integration accepted)
- [x] P2-08 Wheel command allocation (Run #233: Linux and Windows Configure/Build/Test passed; fixed-size proportional wheel-speed limiting, heterogeneous per-wheel bounds, transactional validation, unit tests, benchmark, docs and CMake integration accepted)
- [x] P2-09 disturbance/slip simulation (Run #239: Linux and Windows Configure/Build/Test passed; deterministic per-axis slip attenuation and sinusoidal disturbance model, unit tests, benchmark, docs and CMake integration accepted)

Acceptance: reproducible path tracking benchmark and controller comparison.

## P3 — Manipulators

- [x] P3-01 SE(3)/SO(3) utilities (Run #245: Linux and Windows Configure/Build/Test passed; SE(3)/SO(3) primitives, reference tests, benchmark and documentation accepted)
- [x] P3-02 DH/URDF model adapter (Run #258: Linux and Windows Configure/Build/Test passed; standard DH serial-chain construction, dependency-free URDF parsing, joint limits/axes/origins, transactional validation, unit tests, benchmark and documentation accepted)
- [x] P3-03 Forward kinematics (Run #274: Linux and Windows Configure/Build/Test passed; standard-DH and URDF composition, mixed fixed/revolute/prismatic chain, analytic reference tests, invalid-input transactional behavior, benchmark and documentation accepted)
- [x] P3-04 Numerical IK (Run #283: Linux and Windows Configure/Build/Test passed; finite-difference pose Jacobian, SO(3) orientation residual, damped least-squares update, joint-step and joint-limit enforcement, explicit convergence/invalid/stalled/max-iteration statuses, unit tests, benchmark and documentation accepted)
- [x] P3-05 DLS IK (Run #291: Linux and Windows Configure/Build/Test passed; adaptive damping with accept/reject trials, near-singular convergence, joint-step and joint-limit enforcement, invalid-option handling, unit tests, benchmark, documentation and CMake integration accepted)
- [x] P3-06 Jacobian (Run #298: Linux and Windows Configure/Build/Test passed; analytical planar two-link reference, mixed fixed/revolute/prismatic chain, rotated URDF axis, transactional invalid-input behavior, benchmark, docs and CMake integration accepted)
- [x] P3-07 Singularity metrics (Run #308: Linux and Windows Configure/Build/Test passed; singular values, numerical rank, condition number, manipulability and rank-deficiency behavior, validation tests, benchmark, documentation and CMake integration accepted)
- [x] P3-08 Cubic/quintic trajectory (Run #315: Linux and Windows Configure/Build/Test passed; cubic position/velocity boundary matching, quintic position/velocity/acceleration matching, endpoint clamping, edge cases, unit tests and documentation accepted)
- [x] P3-09 S-curve trajectory (Run #321: Linux and Windows Configure/Build/Test passed; seven-phase jerk-limited profile, reverse/short/cruise/invalid-input tests, benchmark, documentation and CMake integration accepted)
- [x] P3-10 rigid-body dynamics (Run #332: Linux and Windows Configure/Build/Test passed; mass matrix, Coriolis/centrifugal vector, gravity and bias-force reference implementation, analytical tests, benchmark and documentation accepted)
- [x] P3-11 gravity compensation (Run #340: Linux and Windows Configure/Build/Test passed; gravity feedforward scaling, per-joint limits, transactional validation, reference tests, benchmark, docs and CMake integration accepted)
- [x] P3-12 computed torque (Run #348: Linux and Windows Configure/Build/Test passed; model-based joint-space computed torque, PD feedback, torque saturation, transactional validation, analytical/reference tests, benchmark and documentation accepted)
- [x] P3-13 Cartesian control (Run #356: Linux and Windows Configure/Build/Test passed; task-space PD/feedforward wrench control, Jacobian-transpose mapping, orientation feedback, torque saturation, transactional validation, tests, benchmark and documentation accepted)
- [x] P3-14 impedance control (Run #364: Linux and Windows Configure/Build/Test passed; Cartesian virtual spring-damper impedance, feedforward wrench, wrench limiting, transactional validation, reference tests, benchmark and CMake integration accepted)
- [x] P3-15 admittance control (Run #372: Linux and Windows Configure/Build/Test passed; base-frame orientation residual, overflowing-quaternion rejection, transactional state preservation, reference tests, benchmark, documentation and CMake integration accepted)
- [x] P3-16 redundancy/null-space control (Run #386: Linux and Windows Configure/Build/Test passed; exact null-space projection, primary-task leakage/idempotence, damped near-singular behavior, non-finite input rejection with transactional output preservation, deterministic benchmark, documentation and CMake integration accepted)

Acceptance: FK/IK/Jacobian numerical validation; joint and Cartesian tracking benchmarks; contact-control scenario.

## P4 — UAV

- [x] P4-01 6DOF rigid-body model (Run #405: Linux and Windows Configure/Build/Test passed; Newton-Euler derivative, frame conventions, physical-parameter validation, transactional failure behavior, unit tests, benchmark and documentation accepted)
- [x] P4-02 attitude representations (Run #405: Linux and Windows Configure/Build/Test passed; ZYX Euler/quaternion/rotation-matrix conversions, finite/rotation-matrix validation, transactional output behavior, unit tests and documentation accepted)
- [x] P4-03 cascaded position/attitude PID (Run #412: Linux and Windows Configure/Build/Test passed; six PID axes, gravity-compensated force conversion, yaw wrap handling, transactional validation, unit tests, benchmark and documentation accepted)
- [x] P4-04 geometric attitude control (Run #420: Linux and Windows Configure/Build/Test passed; quaternion shortest-arc error, angular-rate feedback, diagonal-inertia gyroscopic compensation, feedforward torque, saturation, unit tests, benchmark and documentation accepted)
- [x] P4-05 LQR position controller (Run #427: Linux and Windows Configure/Build/Test passed; discrete double-integrator model, configuration-time Riccati solution, world-frame acceleration feedback, per-axis saturation, unit/closed-loop tests, benchmark and documentation accepted)
- [x] P4-06 translational MPC (Run #436: Linux and Windows Configure/Build/Test passed; finite-horizon double-integrator prediction, projected-gradient acceleration optimization, fixed-capacity horizon buffers, per-axis saturation, unit/closed-loop tests, benchmark and documentation accepted)
- [x] P4-07 thrust/mixer allocation (Run #445: Linux and Windows Configure/Build/Test passed; X-quadrotor wrench-to-rotor-thrust mixer, collective-priority moment desaturation, reconstructed achieved wrench, validation tests, benchmark and documentation accepted)

Acceptance: waypoint, trajectory and disturbance benchmarks.

## P5 — Floating-base / legged robots

- [x] P5-01 floating-base state model (Run #453: Linux and Windows Configure/Build/Test passed; state conventions, joint dimension/finite-value/quaternion validation, tests, benchmark and documentation accepted)
- [x] P5-02 contact model (Run #460: Linux and Windows Configure/Build/Test passed; unilateral plane point-contact penalty law, regularized Coulomb friction, validation tests, benchmark and documentation accepted)
- [x] P5-03 foot trajectory generation (Run #469: Linux and Windows Configure/Build/Test passed; quintic smoothstep swing path, world-Z clearance arc, position/velocity/acceleration samples, endpoint and invalid-input tests, benchmark and documentation accepted)
- [x] P5-04 gait scheduler (Run #478: Linux and Windows Configure/Build/Test passed; periodic per-leg stance/swing phase, negative-time/offset wrapping, normalized phase progress, validation and transactional failure tests, benchmark and documentation accepted)
- [x] P5-05 centroidal dynamics (Run #487: Linux and Windows Configure/Build/Test passed; world-frame force aggregation, COM-relative moment arms, finite/positive-mass validation, transactional output, tests, benchmark and documentation accepted)
- [x] P5-06 whole-body control (Run #495: Linux and Windows Configure/Build/Test passed; baseline task-wrench torque aggregation, feedforward/secondary torque composition, per-joint saturation, validation tests, benchmark and documentation accepted)
- [x] P5-07 QP task hierarchy (Run #503: Linux and Windows Configure/Build/Test passed; weighted convex QP baseline with box constraints, projected-gradient solver, validation tests, benchmark and documentation accepted)
- [x] P5-08 joint impedance/torque control (Run #505: Linux and Windows Configure/Build/Test passed; joint-space spring-damper torque law, feedforward composition, per-joint saturation, validation tests, benchmark and documentation accepted)
- [x] P5-09 locomotion MPC (Run #510: Linux and Windows Configure/Build/Test passed; COM-level linear MPC with projected-gradient acceleration optimization, standing/walking/disturbance-recovery and transactional validation tests, benchmark and documentation accepted)

Acceptance: standing, walking and disturbance-recovery simulations.

## P6 — Humanoid / whole-body

- [x] P6-01 multi-contact model (Run #519: Linux and Windows Configure/Build/Test passed; multi-point force aggregation, reference-point moments, active-contact accounting, validation and transactional output, tests, benchmark and documentation accepted)
- [x] P6-02 task-space hierarchy (Run #521: Linux and Windows Configure/Build/Test passed; ordered differential-IK levels, damped null-space updates, transactional validation, tests, benchmark and documentation accepted)
- [x] P6-03 contact wrench optimization (Run #522: Linux and Windows Configure/Build/Test passed; regularized projected-gradient wrench allocation, normal-force and friction-pyramid constraints, transactional validation, tests, benchmark and documentation accepted)
- [x] P6-04 balance / ZMP / centroidal control (Run #524: Linux and Windows Configure/Build/Test passed; COM PD acceleration, flat-ground ZMP/support-region classification, finite-input and transactional-output tests, benchmark and documentation accepted)
- [x] P6-05 whole-body QP (Run #527: Linux and Windows Configure/Build/Test passed; weighted least-squares QP wrapper, box bounds, reference regularization, validation tests, benchmark and scope documentation accepted)
- [x] P6-06 manipulation while walking (Run #529: Linux and Windows Configure/Build/Test passed; weighted locomotion/manipulation task coordination, bounds/reference support, residual metrics, validation tests, benchmark and scope documentation accepted)

Acceptance: multi-contact benchmark with constraint and balance checks.

## P7 — Multi-robot control

- [x] P7-01 consensus (Run #534: Linux and Windows Configure/Build/Test passed; graph-weighted discrete-time update, step-size stability validation, transactional failure handling, convergence/multi-coordinate/invalid-input tests, benchmark and scope documentation accepted)
- [x] P7-02 formation control (Run #537: Linux and Windows Configure/Build/Test passed; relative-offset velocity law, per-agent speed saturation, validation/transactional tests, benchmark and scope documentation accepted)
- [x] P7-03 leader-follower (Run #539: Linux and Windows Configure/Build/Test passed; leader velocity feed-forward, arbitrary leader index, follower offset tracking, per-follower speed saturation, transactional validation tests, benchmark and scope documentation accepted)
- [x] P7-04 distributed optimization (Run #541: Linux and Windows Configure/Build/Test passed; gradient-tracking step primitive, doubly-stochastic mixing validation, transactional tests, benchmark and scope documentation accepted)
- [x] P7-05 distributed MPC (Run #543: Linux and Windows Configure/Build/Test passed; bounded receding-horizon coordination over per-agent control sequences, local objective gradients plus graph-neighbor consensus penalty, first-control extraction, dimension/finite-value/control-bound validation, transactional output, tests, benchmark and scope documentation accepted)
- [x] P7-06 communication delay/loss model (Run #545: Linux and Windows Configure/Build/Test passed; deterministic per-link 0/1-step latency and packet-drop handling, delivered-neighbor weighted aggregation, hold-last aggregate when all links drop, topology/mask/finite-state validation, transactional output, tests, benchmark and scope documentation accepted)

Acceptance: formation and degraded-communication benchmarks.

## P8 — Advanced optimization/control

- [x] P8-01 Linearization framework (Run #547: Linux and Windows Configure/Build/Test passed; central finite-difference A/B Jacobians, affine residual, evaluator/dimension/finite-value validation, transactional output, tests, benchmark and scope documentation accepted)
- [x] P8-02 LQR/LQG (Run #552: Linux and Windows Configure/Build/Test passed; discrete-time LQR Riccati solver, linear Kalman predict/update with Joseph covariance form, certainty-equivalent LQG feedback, analytical/reference and validation tests, benchmarks and documentation accepted)
- [x] P8-03 nonlinear MPC (Run #559: Linux and Windows Configure/Build/Test passed; direct-shooting nonlinear MPC with finite-difference control gradients, projected control bounds, transactional validation, scalar nonlinear tracking tests, benchmark and scope documentation accepted)
- [x] P8-04 constrained QP infrastructure (Run #567: Linux and Windows Configure/Build/Test passed; convex QP with equality and box constraints, input/PSD/rank validation, deterministic ADMM baseline, transactional output, tests, benchmark and scope documentation accepted)
- [x] P8-05 generic MPC interface (Run #572: Linux and Windows Configure/Build/Test passed; solver callback contract, problem/result dimension and finite-value validation, first/full control bound checks, transactional failure behavior, tests and documentation accepted)
- [x] P8-06 robust/adaptive control (Run #579: Linux and Windows Configure/Build/Test passed; adaptive parameter update with leakage, robust boundary-layer compensation, parameter projection and control saturation, transactional validation, tests and documentation accepted; task-plan commit verified by Run #580)
- [x] P8-07 sliding-mode control (Run #582: Linux and Windows Configure/Build/Test passed; boundary-layer switching law, output saturation reporting, invalid-input/configuration tests and usage/limitations documentation accepted)
- [x] P8-08 iterative learning control (Run #584: Linux and Windows Configure/Build/Test passed; finite-horizon feedforward update with retention, control bounds, reset, dimension/finite-value validation, transactional failure behavior, tests and documentation accepted)
- [x] P8-09 control barrier function safety filter (Run #587: Linux and Windows Configure/Build/Test passed; scalar control-affine safety constraint, nominal-command projection under actuator bounds, infeasibility and transactional validation tests, documentation accepted)
- [x] P8-10 reference command governor (Run #590: Linux and Windows Configure/Build/Test passed; position/velocity/acceleration limits, discrete stopping-speed bound, feasible velocity interval, transactional validation, boundary/convergence tests and documentation accepted)
- [x] P8-11 multi-constraint scalar CBF filter (Run #592: Linux and Windows Configure/Build/Test passed; interval intersection across multiple scalar CBF inequalities, actuator bounds, infeasibility handling, transactional validation, tests and documentation accepted)
- [ ] P8-12 multi-input CBF safety filter
