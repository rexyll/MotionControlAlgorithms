# P2-01 Differential-drive kinematics

## Scope

This module provides the geometry-only kinematic mapping for a differential-drive robot. It has no ROS 2, Gazebo, hardware, or controller dependency.

## Model

Let `r` be the wheel radius and `b` the distance between the left and right wheel contact lines. For body twist

- `v`: forward linear velocity in m/s
- `omega`: yaw angular velocity in rad/s

and wheel angular velocities `omega_l`, `omega_r` in rad/s:

```text
omega_l = (v - b/2 * omega) / r
omega_r = (v + b/2 * omega) / r
```

The inverse mapping is

```text
v     = r/2 * (omega_l + omega_r)
omega = r/b * (omega_r - omega_l)
```

The sign convention is positive forward `v` and positive counter-clockwise yaw `omega`.

## API

`mca::kinematics::DifferentialDriveKinematics` stores `wheel_radius` and `track_width` and exposes:

- `inverse(Twist2D, WheelSpeeds&)`
- `forward(WheelSpeeds, Twist2D&)`
- geometry validation and accessors

The methods reject non-finite inputs and invalid geometry. The implementation performs no dynamic allocation.

## Acceptance

The unit tests verify:

1. invalid and non-finite inputs are rejected;
2. straight motion produces equal wheel speeds;
3. pure rotation produces equal-and-opposite wheel speeds;
4. forward kinematics matches the analytical equations;
5. inverse/forward mappings round-trip within floating-point tolerance.

A deterministic micro-benchmark is built when Google Benchmark is available.
