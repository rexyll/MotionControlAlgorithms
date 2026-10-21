# P3-01 SE(3)/SO(3) utilities

This module provides dependency-light rigid-body geometry primitives for the robotics stack.

## Scope

- `Vec3` for 3D translation and vector algebra.
- Unit `Quaternion` rotation representation with normalization and multiplication.
- `Mat3` rotation matrices and quaternion/matrix conversion.
- Axis-angle, SO(3) exponential and logarithm maps.
- `Pose3D` composition, inverse, relative pose and point transformation.

The implementation is header-only and does not depend on ROS 2, Gazebo or a middleware runtime.

## Conventions

A pose maps a local point `p` to the parent frame as `t + R p`. Composition follows `T_ab T_bc = T_ac`. Quaternions use scalar-first `(w,x,y,z)` storage and right-handed active rotations.

The SO(3) exponential input is a rotation vector in radians. The logarithm returns the principal rotation vector with angle in `[0, pi]`.

## Acceptance coverage

Unit tests cover axis-angle rotation, quaternion/matrix round-trip, exponential/logarithm round-trip, SE(3) composition consistency and the zero-rotation identity. A deterministic Google Benchmark target exercises point transformation and rotation operations.
