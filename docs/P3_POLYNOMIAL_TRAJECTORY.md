# P3-08 Cubic and Quintic Polynomial Trajectories

This header-only module provides scalar cubic Hermite and quintic polynomial trajectories. It depends only on the existing `ScalarSample` interface and C++ standard library, so it can be used without ROS 2 or Gazebo.

## Boundary conditions

- `CubicTrajectory(q0, v0, qf, vf, T)`: matches initial/final position and velocity.
- `QuinticTrajectory(q0, v0, a0, qf, vf, af, T)`: matches initial/final position, velocity and acceleration.
- `sample(t)` returns position, velocity and acceleration in the caller's units; time is seconds.
- Samples at or before zero return the initial boundary state; samples at or after the duration return the final boundary state. A non-positive duration returns the final position with zero derivatives.

Coefficients are computed once at construction. Sampling uses fixed scalar arithmetic and performs no dynamic allocation.

## Verification

GoogleTest covers cubic boundary values and an analytical midpoint, quintic six-boundary matching and rest-to-rest motion, endpoint clamping, non-positive duration, and finite interior samples. Dual-platform Configure/Build/Test passed in [Run #315](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/39580038085), on implementation commit `36e2060bbe3548f071122006506ba9ec36ec331c`.
