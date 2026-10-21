# P2-02 Mecanum Kinematics

## Scope

This module provides a dependency-light, ROS2/Gazebo-independent forward and inverse kinematic model for a four-wheel standard X-roller mecanum base.

The convention is:

- body `x`: forward, m/s;
- body `y`: left, m/s;
- positive `z` yaw: counter-clockwise, rad/s;
- wheel order: front-left, front-right, rear-left, rear-right;
- wheel angular velocity: rad/s;
- wheel radius `r` in meters;
- `half_length` is the center-to-front/rear-wheel distance;
- `half_width` is the center-to-left/right-wheel distance.

The implementation is intentionally a pure kinematics primitive. It does not contain wheel dynamics, friction, slip, motor limits, chassis integration, ROS2 messages, or Gazebo plugins.

## Model

Let

`G = half_length + half_width`.

For body velocity `(vx, vy, wz)`, inverse kinematics are:

```text
w_fl = (vx - vy - G*wz) / r
w_fr = (vx + vy + G*wz) / r
w_rl = (vx + vy - G*wz) / r
w_rr = (vx - vy + G*wz) / r
```

Forward kinematics are:

```text
vx = r/4 * (w_fl + w_fr + w_rl + w_rr)
vy = r/4 * (-w_fl + w_fr + w_rl - w_rr)
wz = r/(4G) * (-w_fl + w_fr - w_rl + w_rr)
```

These equations correspond to the standard 45-degree roller X configuration. A mechanically different roller orientation or wheel numbering convention requires a different sign matrix and should not be silently mixed with this API.

## API

`mca::kinematics::MecanumKinematics` supports:

- geometry configuration and validation;
- inverse body-twist to four-wheel speed mapping;
- forward four-wheel speed to body-twist mapping;
- finite-input/output validation;
- deterministic, allocation-free operation.

Invalid or non-finite geometry is rejected. Calls made before successful configuration, or with non-finite inputs, return `false`.

## Tests and acceptance

The unit tests cover:

1. invalid geometry rejection;
2. pure forward motion;
3. pure lateral motion and wheel signs;
4. pure yaw rotation;
5. analytical forward-kinematics values;
6. inverse/forward round-trip consistency;
7. non-finite input rejection.

`benchmarks/mecanum_benchmark.cpp` measures inverse and forward mapping independently.

Acceptance requires the analytical/reference tests, benchmark target and CMake integration to pass on both Linux and Windows CI.
