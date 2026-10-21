# P2-05 Stanley Path Tracking

## Scope

A ROS2/Gazebo-independent Stanley controller for a planar polyline and a bicycle-model steering interface.

## Conventions and control law

- World coordinates: x/y; positive yaw is counter-clockwise.
- The path is directed from each vertex to the next.
- Cross-track error is positive when the vehicle is to the right of the nearest directed path segment; a negative value therefore commands a right turn when the vehicle is left of the path.
- Heading error is the wrapped difference between path tangent heading and vehicle heading.
- Steering command: `delta = heading_gain * heading_error + atan2(cross_track_gain * cross_track_error, softening_speed + abs(speed))`.
- Steering is clamped to the configured maximum angle.
- Yaw rate uses the kinematic bicycle relation `omega = v * tan(delta) / wheelbase`.

The nearest point is found by projecting onto every non-degenerate path segment. Duplicate consecutive vertices are ignored; paths with fewer than two points or no usable segment are rejected. Invalid input does not modify the caller's command.

## API

Use `mca::path_tracking::StanleyConfig` to configure cross-track and heading gains, softening speed, signed linear speed, wheelbase, and steering limit. `StanleyCommand` reports linear speed, steering angle, yaw rate, cross-track and heading errors, and the selected segment index.

## Design boundary

This component does not own localization, path smoothing, speed planning, wheel allocation, vehicle dynamics, ROS2/Gazebo messages, or actuator safety limits. The steering limit is a geometric controller limit, not a substitute for platform-level safety constraints.

## Verification

`tests/path_tracking/stanley_test.cpp` covers configuration/input validation, straight-path equilibrium, correction direction, heading correction and steering saturation, nearest-segment selection with duplicate vertices, and closed-loop convergence from a lateral offset.

`benchmarks/stanley_benchmark.cpp` measures deterministic command-generation cost on a fixed sinusoidal path with 501 vertices.
