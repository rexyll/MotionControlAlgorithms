# P12 — Physical hardware abstraction and device integration

## Objective

Introduce a real-device boundary without coupling the algorithm core to ROS 2,
Gazebo, a vendor SDK, an operating system transport, or a specific robot. The
first path is deliberately narrow: fixed-size joint-state acquisition and
joint-effort output, with an explicit device lifecycle. Later device-specific
capabilities must be justified by a concrete backend rather than added to a
universal device interface speculatively.

## Architecture constraints

- Keep physical I/O outside `include/mca/**` except for dependency-free value
  types and capability interfaces.
- Keep the root CMake configure/build independent of ROS 2, Gazebo, and vendor SDKs.
- Use fixed-size storage at the control boundary; do not require heap allocation
  to carry a joint sample or effort command.
- Represent field availability explicitly. An unavailable acceleration or effort
  measurement is not a measured zero.
- Separate lifecycle, state-source, and command-sink capabilities.
- Preserve caller outputs on failed state reads.
- Reject non-finite commands before hardware I/O.
- Treat failed multi-axis writes as control faults; document whether a concrete
  driver can partially apply a command.
- A software safe-stop request is not a safety-rated emergency stop. Target
  hardware still requires its own risk assessment and independent safety circuit.

## Staged tasks

- [x] P12-01 audit existing ports and define hardware capability boundary.
- [x] P12-02 dependency-free joint-state / effort-command / lifecycle interfaces.
- [x] P12-03 deterministic fake hardware port regressions and Linux/Windows CI.
- [x] P12-04 inspect and design the optional ROS 2 control integration boundary.
- [ ] P12-04a implement and verify a single-DoF Jazzy `ActuatorInterface` bridge
  using an explicit mock backend; this validates the ROS/core mapping only and
  is not a physical driver.
- [ ] P12-05 concrete device backend — select one real actuator/robot device,
  verify its official SDK/transport and safety requirements, then implement a
  driver adapter and documented deployment configuration.
- [ ] P12-06 hardware-in-the-loop acceptance — verify sensor read coherence,
  command timing/freshness, watchdog, fault propagation, stop behavior,
  startup/shutdown and repeated reconnect on the selected device. Requires
  access to the target hardware; simulation tests are not substitutes.

## P12-01 audit and design decisions

The existing repository contains simulator-independent state/command values and
ROS 2/Gazebo simulation ports, but no physical hardware directory or driver.
The selected first contract therefore consists of:

1. `IHardwareLifecycle`: initialize, activate, deactivate, and expose lifecycle state.
2. `IJointStateSource`: read one coherent fixed-size sample of position, velocity,
   acceleration, and effort, with a validity mask and per-driver sequence number.
3. `IJointEffortCommandSink`: write a fixed-size effort command and request a
   software safe stop.

No device configuration parser, vendor callback, serial/CAN/EtherCAT transport,
ROS executor, or OS I/O API is admitted into these core ports. Device-specific
partial-write semantics and hardware safety behavior stay in the concrete adapter.

## P12-02 / P12-03 acceptance — 2026-11-01

The dependency-free types and interfaces live in
`include/mca/hardware/interfaces.hpp` and are exported by `mca/mca.hpp`.
The fixed-size joint state contains position, velocity, acceleration and effort
arrays, a sequence counter and an explicit valid-field mask. Only fields marked
valid participate in finite-value validation. The fixed-size effort command
rejects NaN/infinity. State-source contract requires failed reads to preserve
caller output; the command sink must reject invalid values before device I/O.
Lifecycle is explicitly Uninitialized / Inactive / Active / Faulted.

The deterministic fake-device tests are registered in the existing
`mca_core_tests` target through `tests/simulation/step_test.cpp`; they cover
lifecycle transitions, partial measurements, output preservation, sequence
propagation, finite/non-finite commands and safe-stop requests.

Acceptance evidence: root CI [Run #1325](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41283481426)
on code SHA `59225a651d6f9e3b0a7dce9bab4cdbef3a81a93f` completed successfully
on both Linux and Windows. Configure, Build, the full core test target, and the
dedicated P8-39 through P8-46 regression configure/build/test steps all passed.
The Linux log explicitly reports 615/615 core tests passed, including the three
new hardware-port tests. This accepts the dependency-free port contract and fake
backend tests only; no real device or ROS 2 control plugin is claimed by this
evidence.

## P12-04 ROS 2 control integration boundary audit

The ROS 2 Jazzy API documentation supports two adapter shapes:

- `hardware_interface::ActuatorInterface` is the one-DoF hardware component
  interface, with state/command exports and `read(time, period)` /
  `write(time, period)` hooks. It is appropriate for a single motor or
  actuator adapter. Official API:
  https://control.ros.org/jazzy/doc/api/classhardware__interface_1_1ActuatorInterface.html
- `hardware_interface::SystemInterface` is intended for complex systems with
  multiple joints and optional sensors. It exposes state and command
  interfaces and the same read/write cycle. Official API:
  https://control.ros.org/jazzy/doc/api/classhardware__interface_1_1SystemInterface.html

Both derive from the ROS 2 hardware-component lifecycle. Jazzy documents
UNCONFIGURED, INACTIVE, ACTIVE and FINALIZED phases; INACTIVE means the device
communication is configured and state can be read, while movement command
interfaces are unavailable. ACTIVE enables movement command interfaces and
may enable power/brake circuits. This informs the adapter mapping:
ROS `on_configure` establishes the core device connection, `on_activate` maps
to `IHardwareLifecycle::activate`, `on_deactivate` maps to
`deactivate`, `read` copies a coherent `JointState`, and `write` validates
and forwards an effort command. `on_init` remains responsible for URDF/plugin
configuration parsing at the ROS boundary.

The ROS 2 plugin must live in a separate optional package and translate between
ROS interface storage and the dependency-free MCA ports. ROS 2 types, pluginlib,
URDF parsing, dynamic vectors, and device SDKs do not enter `include/mca/**` or
the root CMake build. Because `SystemInterface` discovers joint counts at
runtime while MCA's first hardware value types are fixed-size templates, a
concrete plugin must choose a known robot dimension or explicitly batch/bridge
runtime-sized ROS storage at the adapter boundary; it must not silently allocate
or resize in the hard real-time write path.

## P12-04a reference plugin — implementation in progress

The optional package `ros2/mca_ros2_control_adapter` provides a reusable
single-DoF `ActuatorHardware` bridge and a pluginlib-exported
`mca_ros2_control_adapter/MockActuator`. The bridge maps the ROS lifecycle to
MCA lifecycle operations, copies only valid backend state fields into declared
ROS state interfaces, and rejects non-finite effort before forwarding a command.
Deactivation requests a software safe-stop before disabling the backend.

The MockActuator backend returns synthetic zero position/velocity and stores
effort in memory; it does not simulate a motor or communicate with hardware.
The optional ROS 2 workflow is being extended to compile the package and run a
plugin-discovery/load test. Do not mark P12-04a accepted until that workflow passes.

## P12-05 / P12-06 boundary

No specific physical robot, motor, drive, bus or vendor SDK has been selected in
this task. Do not implement a speculative vendor backend. P12-05 requires a
concrete device target and verified official SDK/transport/control-mode/fault
documentation. P12-06 requires physical hardware access and separate HIL
evidence; Gazebo or a mock backend cannot substitute for it.
