# mca_ros2_control_adapter

Optional ROS 2 Jazzy `ros2_control` bridge for the dependency-free physical
hardware ports in `include/mca/hardware/interfaces.hpp`.

## Adapter design

- `ActuatorHardware` derives from Jazzy's `hardware_interface::ActuatorInterface`
  for a single-DoF motor/actuator.
- Concrete plugin classes provide an `IScalarJointBackend` implementation that
  combines MCA lifecycle, coherent joint-state read and effort-command sink
  capabilities.
- ROS 2 lifecycle callbacks map to `IHardwareLifecycle`: configure initializes
  the backend, activate enables it, deactivate requests a software safe-stop
  before deactivation, and cleanup releases the backend.
- `read()` copies one validated, monotonically sequenced MCA state snapshot to
  pre-exported ROS state-interface handles. A repeated or stale sample fails;
  unavailable measurements are not substituted with zero.
- `write()` reads an effort value through a pre-exported command handle,
  validates finiteness and sequence, then forwards the command to the backend.
  The real-time path does not construct interface-name strings, perform name
  lookup, or wait for interface storage.
- Runtime ROS interface storage, URDF metadata, pluginlib and vendor transports
  remain in this optional package. The root CMake project has no ROS dependency.

## Reference mock plugin

The package exports `mca_ros2_control_adapter/MockActuator`. It is only a
deterministic adapter-boundary fixture: its synthetic position and velocity
remain zero and it stores effort commands in memory. It does not model a motor,
provide physical feedback or control a real actuator. The ROS 2 Jazzy test
loads it through pluginlib, performs lifecycle configuration/activation, reads
the mock state, sends a finite effort command, verifies NaN rejection, and
checks that writes are rejected after deactivation.

Example URDF fragment:

```xml
<ros2_control name="MCAActuator" type="actuator">
  <hardware>
    <plugin>mca_ros2_control_adapter/MockActuator</plugin>
  </hardware>
  <joint name="motor_joint">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

The first adapter supports exactly one joint, exactly one `effort` command
interface, and mandatory `position` plus `velocity` state interfaces. Optional
acceleration and effort state interfaces are accepted only when the backend
marks those measurements valid.

## Build and test on ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-hardware-interface ros-jazzy-pluginlib
colcon build --base-paths ros2 --packages-select mca_ros2_control_adapter
colcon test --base-paths ros2 --packages-select mca_ros2_control_adapter
colcon test-result --test-result-base build/mca_ros2_control_adapter/test_results --verbose
```

This package is driver-neutral. P12-05 still requires a selected physical device,
its official SDK/transport, control mode and safety/fault handling. A plugin-load
and mock lifecycle test are not hardware-in-the-loop acceptance.
