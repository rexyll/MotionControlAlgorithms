# P12-05a.3 Acceptance — ROS 2 VirtualSystem plant integration

Date: 2026-11-01

## Result

**Accepted for deterministic software/virtual-plant integration only.**

The new ROS 2 Jazzy `VirtualSystem` plugin connects the two-joint ros2_control
`SystemInterface` to MCA's deterministic `VirtualJointDevice<2>`. This closes
the software integration path from ROS effort command interfaces to simulated
joint dynamics and state feedback. It is not Gazebo integration or physical
hardware/HIL validation.

## Implementation

- Added pluginlib class `mca_ros2_control_adapter/VirtualSystem`.
- The plugin composes the existing vendor-neutral `VirtualJointDevice<2>`;
  `MockSystem` remains a separate fixed-output adapter fixture.
- Added an optional `IDualJointBackend::update(period_seconds)` hook. Its
  default implementation is a no-op, preserving compatibility with mock and
  future physical backends.
- `SystemHardware::read()` validates the supplied ROS period and calls the
  backend update hook before reading the coherent state sample.
- The virtual backend advances only while active. A zero period is a no-op;
  negative/non-finite periods and integration periods greater than the plant's
  configured 10 ms bound are rejected.
- The plugin does not silently sub-step an oversized period.

## Focused tests

`test_virtual_system_plugin` verifies:

1. Plugin discovery and construction through pluginlib.
2. Positive and negative effort commands on two joints produce matching
   positive and negative position/velocity feedback.
3. Effort state feedback reflects the applied effort.
4. Deactivation performs the virtual device's software safe-stop, rejects
   subsequent writes, and leaves state readable.
5. A period exceeding the integration bound is rejected without preventing a
   subsequent valid-period read.

ROS 2 Jazzy workflow [Run #33](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41317219131)
completed successfully on code SHA
`f0513644ce5f6ad0909c7757cbda2bdd3e366a91`. The optional adapter package
built successfully; the existing actuator and mock-system tests passed, and
`test_virtual_system_plugin` reported 3/3 tests passed. Overall, all three
test targets passed.

Root CI [Run #1358](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41317219152)
completed successfully on the same code SHA. Linux and Windows both passed
Configure, Build, the 628/628 core test suite, and all dedicated P8-39 through
P8-46 regression configure/build/test jobs.

## Limitations

- The virtual plant is a deterministic software model with synthetic default
  parameters; it is not a calibrated physical actuator model.
- This acceptance does not verify Gazebo bridge operation or synchronization
  against Gazebo simulation time.
- It does not verify physical drive communication, command freshness, watchdog
  latency, hardware fault decoding, safety-rated emergency stop, or HIL behavior.
- The default virtual device is single-thread-owned. No claim is made about
  physical transport thread safety or hard real-time scheduling.

P12-05 concrete hardware backend and P12-06 physical HIL remain open pending
selection and access to a real target device.
