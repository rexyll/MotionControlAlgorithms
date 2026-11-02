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
- [x] P12-05a.1 vendor-neutral virtual joint plant and catalog-backed motor
  simulation profile; software/simulation scope only.
- [x] P12-05a.2 fault-injectable virtual-device diagnostics and safe-recovery
  semantics; accepted on 2026-11-01 for simulation-only scope after root CI
  Run #1356 and ROS 2 Jazzy Run #32 passed on code SHA
  `b0c5d85c6ebe86bacaf5c625db7cda9f30ac2d1f`.
- [x] P12-05a.3 connect the deterministic two-joint virtual plant to a
  pluginlib-exported ROS 2 Jazzy `SystemInterface`; accepted on 2026-11-01
  for simulation-only scope on code SHA
  `f0513644ce5f6ad0909c7757cbda2bdd3e366a91`. See
  [P12-05a.3 acceptance](P12-05a3_ACCEPTANCE.md).
- [x] P12-05a.4 live Gazebo Harmonic + ROS 2 Jazzy `ros2_control`
  integration, including two-joint motion and hardware/controller lifecycle
  recovery; accepted on 2026-11-02 on code SHA
  `dc43644d0e82384f8c96ad2725a52ec7dfd1e17e`. See
  [P12-05a.4 acceptance](P12-05a4_ACCEPTANCE.md).
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
callbacks to MCA lifecycle operations, caches state/command handles before
the control loop, copies only valid backend state fields, rejects non-finite effort,
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
`mca_ros2_control_adapter/MockSystem`. The fixed dimension is explicit at
the MCA boundary (`JointState<double, 2>` and
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


## P12-05a.1 — catalog-backed motor simulation profile

Added an optional motor-model example for maxon EC-i 40, 48 V, Hall-sensor
part 496653 in
`include/mca/plant/vendor_profiles/maxon_ec_i_40_496653.hpp`. The profile
records the manufacturer's catalog values in SI units: 48 V, 7560 rpm no-load,
6310 rpm nominal, 73.9 mN·m continuous torque, 1.19 A continuous current,
2.59 Ω phase-to-phase terminal resistance, 1.93 mH phase-to-phase terminal
inductance, 59.6 mN·m/A torque constant, 160 rpm/V speed constant, 12.8 g·cm²
rotor inertia, 10,000 rpm maximum speed and 7 pole pairs.

Official references:
- maxon product page:
  https://www.maxongroup.com/maxon/view/product/motor/ecmotor/EC-i/496653
- maxon EC-i 40 datasheet:
  https://www.maxongroup.com/medias/sys_master/root/9344338165790/Datasheet-EC-i-303-EN.pdf

The catalog reports terminal resistance/inductance phase-to-phase, while
`BldcPmsm` uses an uncoupled per-phase approximation. Therefore the profile
does not silently convert these electrical parameters. Callers must explicitly
supply a `ModelMapping` with per-phase resistance/inductance and damping.
The test uses R_phase = R_terminal/2 and L_phase = L_terminal/2 solely as an
illustrative star-connected, zero-mutual-coupling approximation; viscous
damping 0.0001 N·m/(rad/s) is synthetic. Neither assumption is represented as
a manufacturer specification. Torque/back-EMF convention must also be matched
to the selected BLDC/PMSM model before treating this as a validated plant.

The profile test verifies the catalog values, SI conversions and that the
explicitly mapped model configuration is accepted by the existing motor plant.
This is a parameter/configuration test, not physical motor validation.


## P12-05a.2 — fault-injectable virtual-device diagnostics — accepted 2026-11-01

Added `IHardwareDiagnostics` as a dependency-free, fixed-capacity diagnostics
capability. A fault record preserves a vendor-opaque raw code, normalized
category, active/latched state, monotonically increasing sequence and caller-
supplied timestamp. Diagnostics remain separate from lifecycle, state-source
and command-sink interfaces.

`FaultInjectableVirtualJointDevice<N>` decorates the deterministic virtual
joint device. It supports multiple simultaneous categories, rejects a duplicate
active category, requests software safe-stop on injection, blocks subsequent
effort commands and simulation steps while fault records remain, and permits
explicit fault resolution followed by latch clearing and reactivation. State
reads remain available while faulted so a test harness can inspect the stopped
state. An empty `clear_faults()` is a no-op and must not deactivate a healthy
device; a regression test covers this behavior.

Focused tests verify healthy-device no-op clearing, immediate zero-effort
safe-stop and command inhibition, preservation of opaque raw fault codes,
active/latched semantics, rejection of clearing active faults, explicit
resolve/clear/reactivation, and retention of multiple independent categories.

Acceptance evidence on code SHA
`b0c5d85c6ebe86bacaf5c625db7cda9f30ac2d1f`:
- Root CI [Run #1356](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41313455621)
  passed Linux and Windows. Both platforms reported 628/628 core tests passed;
  the four `FaultInjectableVirtualJointDevice` tests passed, including the
  empty-clear regression. The dedicated P8-39 through P8-46 regression suites
  also passed.
- ROS 2 Jazzy workflow
  [Run #32](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41313455557)
  passed both mock hardware plugin tests and the scalar adapter test.

This acceptance covers deterministic software behavior and diagnostics
contracts only. Fault injection is a test facility, not a real drive fault
decoder. It does not establish physical watchdog response, safety-rated stop,
vendor fault mapping, communication timing, real-time scheduling, physical
device behavior or hardware-in-the-loop acceptance. P12-05 and P12-06 remain
open until a concrete target device and hardware access are available.


## P12-05a.3 — ROS 2 control integration with the virtual joint plant — accepted 2026-11-01

Added pluginlib-exported `mca_ros2_control_adapter/VirtualSystem`, a fixed
two-joint Jazzy `SystemInterface` implementation backed by
`VirtualJointDevice<2>`. The existing `MockSystem` remains unchanged and
continues to test the adapter boundary with fixed mock telemetry.

The dual-joint backend now has an optional `update(period_seconds)` hook with
a default no-op implementation. `SystemHardware::read()` validates the ROS
control period and invokes this hook before acquiring the coherent backend
sample. The virtual backend advances its plant by the supplied period only
while active; zero period is a no-op, invalid/negative periods are rejected,
and periods above the plant's configured 10 ms integration bound fail
explicitly rather than being silently sub-stepped. Physical backends can retain
the default no-op and implement their own acquisition/update semantics.

The ROS plugin tests verify plugin discovery/construction, opposing positive
and negative two-joint effort commands producing corresponding position and
velocity feedback, effort feedback, software safe-stop/deactivation, and
rejection of a control period exceeding the integration bound. These tests
exercise the ROS plugin and deterministic virtual plant together; they do not
validate Gazebo integration or a physical device.

Acceptance evidence on code SHA
`f0513644ce5f6ad0909c7757cbda2bdd3e366a91`:
- Root CI [Run #1358](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41317219152)
  passed Linux and Windows. Both platforms completed Configure, Build and
  Test; each reported 628/628 core tests passed, and all dedicated P8-39
  through P8-46 regression configure/build/test jobs passed.
- ROS 2 Jazzy workflow
  [Run #33](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41317219131)
  passed the optional adapter build and all three test targets. The new
  `test_virtual_system_plugin` reported 3/3 tests passed.

This acceptance is limited to deterministic software and virtual-plant
integration. It does not establish Gazebo bridge behavior, simulation-time
synchronization with Gazebo, physical motor/drive behavior, transport timing,
watchdog behavior, safety-rated stop, or HIL acceptance. P12-05 concrete
hardware backend and P12-06 physical HIL remain open pending selection and
access to a real target device.


## P12-05a.4 — live Gazebo + ros2_control integration — accepted 2026-11-02

The pluginlib-exported `mca_gz_ros2_control_adapter` was loaded by Gazebo
Harmonic and connected to ROS 2 Jazzy `controller_manager`. The live
two-joint test verified hardware initialization, controller activation,
non-zero joint motion, explicit controller/hardware deactivation, hardware
reactivation, and controller recovery. The adapter preserves its monotonically
increasing command sequence across reactivation so the backend does not reject
the first resumed command as stale.

Gazebo workflow [Run #28](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41393338877)
and root CI [Run #1398](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41393338912)
passed on code SHA `dc43644d0e82384f8c96ad2725a52ec7dfd1e17e`. The live
run collected 2,938 valid joint-state samples and observed approximately
3.14 rad absolute movement on each joint, retained the `1e-4) movement
threshold, and ended with
`Live Gazebo + controller_manager closed-loop acceptance passed`. Root CI
passed Linux and Windows. See [P12-05a.4 acceptance](P12-05a4_ACCEPTANCE.md)
for the detailed lifecycle and runtime evidence.

This closes the Gazebo/ROS 2 simulation integration milestone only. It does
not establish physical actuator behavior, vendor transport compatibility,
safety-rated stopping, hard real-time behavior or HIL acceptance. P12-05
concrete hardware backend and P12-06 HIL remain open.
