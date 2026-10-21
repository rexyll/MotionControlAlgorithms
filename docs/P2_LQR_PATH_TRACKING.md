# P2-06 LQR Path Tracking

## Scope

A middleware-independent path tracker using a discrete-time linear-quadratic regulator (LQR) around a kinematic bicycle model.

## Error convention and model

The nearest point is projected onto each non-degenerate directed path segment. Lateral error is positive when the vehicle lies to the left of the segment direction. Heading error is the wrapped difference `vehicle_yaw - path_tangent_yaw`.

For a forward speed (v>0), wheelbase (L), and sample time (T), the small-error model is

`e_y(k+1) = e_y(k) + v T e_psi(k)`
`e_psi(k+1) = e_psi(k) + (v T / L) delta(k)`

The controller solves the 2-state discrete algebraic Riccati equation iteratively during configuration, deriving the feedback gains from lateral-error, heading-error, and steering-effort weights. The runtime law is `delta = -K_y e_y - K_psi e_psi`; steering is saturated to the configured limit. Yaw rate is `omega = v tan(delta) / L`.

## Configuration and failure behavior

Weights, sample time, forward speed, wheelbase and steering limit must be finite and positive; the steering limit must be below pi/2. The Riccati iteration must converge for configuration to succeed. Input paths must contain at least two finite points and at least one non-degenerate segment. Failed compute calls do not modify the output command. The controller recomputes gains when configuration changes; it does not allocate or solve the Riccati equation in the control loop.

## Design boundary

This component does not perform localization, path smoothing, speed planning, wheel allocation, vehicle dynamics, ROS2/Gazebo messaging, or platform safety enforcement. It uses a local straight-line error model, so sharp curvature, large tracking errors, and low-level actuator dynamics may require gain scheduling or a richer model.

## Verification

`tests/path_tracking/lqr_path_tracker_test.cpp` covers configuration validation, zero-error equilibrium, correction signs, heading feedback, nearest-segment selection, duplicate vertices, transactional failure behavior, steering saturation, and closed-loop lateral convergence.

`benchmarks/lqr_path_tracker_benchmark.cpp` measures command-generation cost against a fixed 501-point sinusoidal path. Riccati gain synthesis occurs only in configuration, not in the benchmarked control loop.
