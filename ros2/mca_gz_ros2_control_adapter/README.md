# mca_gz_ros2_control_adapter

Optional ROS 2 Jazzy / Gazebo Harmonic integration. The exported
`mca_gz_ros2_control_adapter/GazeboSystem` plugin derives from
`gz_ros2_control::GazeboSimSystemInterface` and maps two Gazebo joint entities
to MCA's fixed-size dual-joint state and effort ports.

## Data path

- `initSim()` resolves the two configured joint names to Gazebo ECS entities.
- The backend ensures `JointPosition`, `JointVelocity`, and `JointForceCmd`
  components exist for each joint.
- `read()` obtains both joints' position and velocity from the same ECS
  update context and exports a monotonically sequenced MCA state snapshot.
- `write()` validates the complete two-joint effort command and writes it to
  each joint's `JointForceCmd` component.
- Deactivation and shutdown request zero effort before disabling the backend.

This path uses the Gazebo physics engine as the plant. It does not instantiate
or advance `VirtualJointDevice`, and does not use the separate
`ScalarJointSystem` ROS/Gazebo Transport bridge. The physics engine applies
the effort command during its simulation update; the next read observes the
resulting joint state.

## Scope and limitations

The initial plugin supports exactly two joints, one effort command interface
per joint, and position/velocity state interfaces. It is a simulator backend,
not a physical device driver. A zero-effort software request is not a
safety-rated stop. A failure while writing multiple joint commands can leave a
partial Gazebo component update and must be treated as a simulation/control
fault. Physical hardware and HIL acceptance remain separate.

## Build and test

On Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic:

```bash
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-ament-cmake-gtest
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2 --packages-select mca_gz_ros2_control_adapter
colcon test --base-paths ros2 --packages-select mca_gz_ros2_control_adapter
colcon test-result --verbose
```

The package's tests verify pluginlib discovery, construction, and hardware
interface topology. They do not by themselves constitute a live Gazebo
closed-loop acceptance test.
