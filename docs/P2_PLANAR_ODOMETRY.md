# P2-03 Planar odometry

## Scope

P2-03 provides a ROS2/Gazebo-independent planar odometry primitive for wheel-robot domains. The core integrates a constant body-frame twist over each sample interval and maintains an SE(2) pose `(x, y, yaw)`.

## Model

For body twist `(v_x, v_y, omega)` and sample interval `dt`, yaw advances by

`Delta_yaw = omega * dt`.

For nonzero angular velocity, the body-frame displacement uses the exact constant-twist SE(2) exponential:

`dx = (v_x sin(Delta_yaw) + v_y (1-cos(Delta_yaw))) / omega`

`dy = (-v_x (1-cos(Delta_yaw)) + v_y sin(Delta_yaw)) / omega`.

The body displacement is then rotated into the world frame using the pose's current yaw. When angular velocity is near zero, the implementation uses the straight-line limit `dx=v_x dt`, `dy=v_y dt`.

## API

Header: `include/mca/odometry/planar_odometry.hpp`

- `Pose2D` stores world-frame position and yaw.
- `BodyTwist2D` stores body-frame linear and angular velocity.
- `PlanarOdometry::reset()` sets an initial pose.
- `PlanarOdometry::update()` integrates one positive finite sample interval.
- Invalid input is rejected without modifying the current pose.

## Validation

Unit tests cover:

- straight-line motion;
- arbitrary initial heading;
- exact circular-arc integration;
- lateral body motion for omnidirectional platforms;
- invalid/non-finite input and state preservation.

Benchmark: `mca_planar_odometry_benchmark` measures repeated fixed-step integration without ROS2/Gazebo dependencies.

## Design boundary

This component intentionally does not own wheel encoders, ROS messages, timestamps, covariance estimation, slip models, or frame-tree publication. Those concerns belong to higher-level adapters and later P2/P9 tasks.
