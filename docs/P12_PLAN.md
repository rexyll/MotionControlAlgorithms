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
- [x] P12-04a implement and verify a single-DoF Jazzy `ActuatorInterface` bridge
  using an explicit mock backend; this validates the ROS/core mapping only and
  is not a physical driver.
- [x] P12-04b implement and verify a fixed two-joint Jazzy `SystemInterface`
  bridge with a pluginlib mock backend. Accepted on 2026-11-01 after ROS 2
  Jazzy workflow Run #26 and root CI Run #1342 passed on code SHA
  `ea1fd4648af76fdcb64c721f16526c29c681e904`; see the detailed acceptance
  record below. This validates the ROS/core adapter and mock backend only,
  not a physical driver or hardware-in-the-loop behavior.
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

## P12-04a reference plugin — accepted 2026-11-01

The optional package `ros2/mca_ros2_control_adapter` provides a reusable
single-DoF `ActuatorHardware` bridge and a pluginlib-exported
`mca_ros2_control_adapter/MockActuator`. The bridge uses Jazzy's
`HardwareComponentInterfaceParams` initialization API, maps ROS lifecycle
callbacks to MCA lifecycle operations, caches state/command handles before the
control loop, copies only valid backend state fields, rejects non-finite effort,
and rejects non-increasing state sample sequence numbers. The read/write path
uses non-blocking handle access; it does not construct interface-name strings
or perform interface-name lookups each cycle. Deactivation requests software
safe-stop before disabling the backend.

The MockActuator backend returns synthetic zero position/velocity and stores
effort in memory; it does not simulate a motor or communicate with hardware.
Acceptance evidence: ROS 2 Jazzy workflow
[Run #15](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41285081125),
code SHA `8a21ad1dcdcedd191302a025719348b4c643ace4`, completed successfully.
Both optional packages built and installed. The existing `mca_ros2_adapter`
reported 4 tests, 0 errors, 0 failures, 0 skipped. The new
`mca_ros2_control_adapter` reported 2 tests, 0 errors, 0 failures, 0 skipped;
these cover plugin discovery/construction and the mock lifecycle/configuration,
state read, finite effort write, NaN rejection, deactivation safe-stop path,
and rejection of writes after deactivation. The build has no deprecated
`on_init(HardwareInfo)` warning; deprecated declarations are treated as errors
for this package. This accepts the ROS/core adapter contract only. It is not
physical device driver or hardware-in-the-loop evidence.

## P12-05 / P12-06 boundary

No specific physical robot, motor, drive, bus or vendor SDK has been selected in
this task. Do not implement a speculative vendor backend. P12-05 requires a
concrete device target and verified official SDK/transport/control-mode/fault
documentation. P12-06 requires physical hardware access and separate HIL
evidence; Gazebo or a mock backend cannot substitute for it.


## P12-04b multi-joint reference plugin — accepted 2026-11-01

The optional `mca_ros2_control_adapter` package now also contains
`SystemHardware`, a deliberately fixed two-joint implementation of Jazzy's
`hardware_interface::SystemInterface`, and the pluginlib-exported
`mca_ros2_control_adapter/MockSystem`. The fixed dimension is explicit at the
MCA boundary (`JointState<double, 2>` and
`JointEffortCommand<double, 2>`); runtime-sized ROS metadata is handled only
during initialization and interface export.

The adapter validates a two-joint topology, requires position and velocity
state plus effort command interfaces for both joints, optionally exports effort
state, pre-caches all state/command handles before the control loop, checks
sample validity and monotonically increasing sequence numbers, and reads the
complete two-axis command before issuing one fixed-size backend write. A
non-finite value on either axis rejects the whole command before the backend
call. Deactivation requests software safe-stop before backend deactivation.
The mock backend provides deterministic values only and does not simulate plant
dynamics or communicate with hardware.

Focused tests cover plugin discovery, lifecycle, two-joint state export,
two-joint effort write, whole-command rejection for a NaN, post-deactivation
write rejection, safe-stop zero effort, and invalid joint topology. Acceptance evidence:
- ROS 2 Jazzy workflow [Run #26](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41291882781)
  passed the optional adapter package build and test steps.
- Root CI [Run #1342](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41291882763)
  passed on both Linux and Windows, including Configure, Build, Test, and the
  dedicated P8-39 through P8-46 regression configure/build/test steps.
- Both workflows tested code SHA
  `ea1fd4648af76fdcb64c721f16526c29c681e904`.

The focused ROS 2 tests verify plugin discovery, lifecycle, two-joint state
export/read, two-joint effort write, whole-command rejection for a NaN,
post-deactivation write rejection, safe-stop zero effort, and invalid joint
topology. The CI evidence accepts the fixed two-joint ROS/core adapter and its
deterministic mock backend only; no physical driver, plant dynamics, or HIL
behavior is claimed.


## P12-05a — vendor-neutral hardware profile and virtual device

The first reusable device backend is intentionally manufacturer-independent.
`VirtualJointDeviceConfig<N>` provides per-axis inertia, viscous damping,
stiffness, effort limits, position/velocity limits, initial position and a
maximum integration step. `VirtualJointDevice<N>` implements the existing
`IHardwareLifecycle`, `IJointStateSource` and
`IJointEffortCommandSink` contracts, and adds a bounded simulation-only
`step(dt)` operation. The plant uses a simple rotary-joint equation
`J*qdd = effort - damping*qd - stiffness*q`, semi-implicit Euler integration,
and position/velocity bounds.

Default profile (all values are synthetic engineering assumptions, not a
manufacturer data sheet): inertia 0.02 kg·m², viscous damping 0.02 N·m·s/rad,
stiffness 0 N·m/rad, effort limit ±5 N·m, position range ±π rad, velocity
limit ±10 rad/s, initial position 0 rad, maximum simulation step 10 ms.
Each axis can override every parameter. Invalid profiles and out-of-range
commands are rejected; invalid writes do not replace the last accepted command.
State samples include position, velocity, acceleration and applied effort with
a monotonically increasing sample sequence. Failed reads preserve caller output.

The generic ports remain independent of ROS 2, Gazebo and vendor SDKs. A future
vendor adapter should translate vendor telemetry/commands into these normalized
SI-unit values and preserve device-specific fault codes, timing and partial-write
semantics at its boundary. The current virtual backend is single-thread-owned;
physical backends must define bounded synchronization and transport/watchdog
behavior for their actual execution model.

Acceptance scope: deterministic virtual-plant and hardware-port behavior only.
It does not establish real device parameters, communication compatibility,
driver-package build, safety certification or HIL acceptance.
