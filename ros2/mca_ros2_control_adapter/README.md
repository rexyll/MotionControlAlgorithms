# mca_ros2_control_adapter

Optional ROS 2 Jazzy `ros2_control` bridge for the dependency-free physical
hardware ports in `include/mca/hardware/interfaces.hpp` and the deterministic
virtual joint plant in `include/mca/hardware/virtual_joint_device.hpp`.

## Adapter design

- `ActuatorHardware` derives from Jazzy's `hardware_interface::ActuatorInterface`
  for a single-DoF motor/actuator.
- `SystemHardware` derives from Jazzy's `hardware_interface::SystemInterface`
  for a fixed two-joint system.
- Concrete plugin classes provide an `IScalarJointBackend` or
  `IDualJointBackend` implementation that combines MCA lifecycle, coherent
  joint-state read and effort-command sink capabilities.
- ROS 2 lifecycle callbacks map to `IHardwareLifecycle`: configure initializes
  the backend, activate enables it, deactivate requests a software safe-stop
  before deactivation, and cleanup releases the backend.
- `read()` copies one validated, monotonically sequenced MCA state snapshot to
  pre-exported ROS state-interface handles. A repeated or stale sample fails;
  unavailable measurements are not substituted with zero.
- `write()` reads effort values through pre-exported command handles, validates
  finiteness and sequence, then forwards the complete command to the backend.
  The real-time path does not construct interface-name strings, perform name
  lookup, or wait for interface storage.
- Runtime ROS interface storage, URDF metadata, pluginlib and vendor transports
  remain in this optional package. The root CMake project has no ROS dependency.

## Reference mock plugins

The package exports `mca_ros2_control_adapter/MockActuator` and
`mca_ros2_control_adapter/MockSystem`. They are deterministic adapter-boundary
fixtures: the scalar mock returns synthetic zero position/velocity and stores
effort in memory; the two-joint mock returns fixed state values and echoes
applied effort. Neither simulates plant dynamics, provides physical feedback,
nor communicates with hardware.

## Virtual two-joint plant plugin

The package also exports `mca_ros2_control_adapter/VirtualSystem`. It composes
MCA's `VirtualJointDevice<2>` and advances the deterministic rotary-joint plant
from the ROS `read(time, period)` callback using the supplied control period.
The command written in one control cycle is applied to the plant on the next
read/update cycle. The default plant integration bound is 10 ms; a larger
period is rejected explicitly rather than silently sub-stepped. A zero period
does not advance the plant.

The virtual device's default parameters are synthetic engineering assumptions,
not manufacturer data. Configure the joint limits and plant parameters in the
core device config when building a dedicated simulation harness. The virtual
plugin is a software integration fixture; it is not a Gazebo plugin, calibrated
motor model, physical driver or HIL substitute.

Example URDF fragment for the two-joint virtual system:

```xml
<ros2_control name="MCAVirtualSystem" type="system">
  <hardware>
    <plugin>mca_ros2_control_adapter/VirtualSystem</plugin>
  </hardware>
  <joint name="joint_a">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <joint name="joint_b">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

The plugin requires exactly two uniquely named joints, one effort command
interface per joint, and position/velocity state interfaces. Effort state is
optional but must be declared consistently for both joints. The plugin
validates topology during initialization.

## Single-DoF mock actuator example

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

## Build and test on ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-hardware-interface ros-jazzy-pluginlib
colcon build --base-paths ros2 --packages-select mca_ros2_control_adapter
colcon test --base-paths ros2 --packages-select mca_ros2_control_adapter
colcon test-result --test-result-base build/mca_ros2_control_adapter/test_results --verbose
```

P12-05 still requires a selected physical device, its official SDK/transport,
control mode and safety/fault handling. Plugin loading and virtual-plant tests
are not hardware-in-the-loop acceptance.
