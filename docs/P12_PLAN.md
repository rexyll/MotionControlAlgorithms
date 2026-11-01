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
- [ ] P12-02 dependency-free joint-state / effort-command / lifecycle interfaces.
- [ ] P12-03 deterministic fake hardware port regressions and Linux/Windows CI.
- [ ] P12-04 inspect and design the optional ROS 2 control integration boundary.
- [ ] P12-05 implement one concrete vendor/device adapter after selecting target hardware.
- [ ] P12-06 perform target-device hardware-in-the-loop acceptance.

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

## P12-02 / P12-03 acceptance

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
propagation, finite/non-finite commands and safe-stop requests. Do not mark
P12-02/P12-03 accepted until Linux and Windows Configure/Build/Test pass.
