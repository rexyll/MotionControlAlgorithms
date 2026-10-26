# mca_ros2_adapter

Optional ROS 2 Jazzy integration for the simulator-independent core ports.

## Initial scalar topics

- State topic (std_msgs/msg/Float64MultiArray): simulation time in seconds, fixed dt in seconds, step index encoded as a double, position, velocity, acceleration.
- Command topic (std_msgs/msg/Float64): scalar actuator effort.

The adapter rejects invalid state snapshots, ignores non-finite command messages, and refuses to return missing or stale commands. A failed read leaves the caller output unchanged. Freshness uses std::chrono::steady_clock, so simulation-time pauses do not make old commands fresh.

The array layout is a prototype wire contract. Before external deployment, replace it with a versioned custom message containing an integer step index, explicit units, and a schema compatibility policy.

## Build on ROS 2 Jazzy

Source /opt/ros/jazzy/setup.bash, then run from the repository root:

    colcon build --base-paths ros2 --packages-select mca_ros2_adapter

The root CMake project does not discover or link ROS 2; this package is opt-in and built separately.
