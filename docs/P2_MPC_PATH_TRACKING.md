# P2-07 MPC Path Tracking

## Scope

A middleware-independent receding-horizon path tracker based on a linearized kinematic-bicycle error model. Each control call projects the vehicle onto the closest non-degenerate path segment, optimizes a finite sequence of steering inputs, and applies only the first input.

## Model and objective

For positive forward speed (v), wheelbase (L), sample period (T), lateral error (e_y), heading error (e_\psi), and steering input \(\delta\):

- \(e_y(k+1)=e_y(k)+vT e_\psi(k)\)
- \(e_\psi(k+1)=e_\psi(k)+(vT/L)\delta(k)\)

The finite-horizon quadratic objective penalizes lateral error, heading error, steering effort, and terminal lateral/heading error. Projected gradient descent optimizes steering inputs within the configured steering bounds. The first optimized steering value is converted to yaw rate using \(\omega=v\tan(\delta)/L\).

## Configuration and failure behavior

Weights, sample period, forward speed, wheelbase, steering bound and gradient step must be finite and positive. Horizon and optimization iteration count are bounded to prevent accidental unbounded work. Input poses and path points must be finite, the path must contain at least two points and at least one non-degenerate segment, and failure leaves the output command unchanged.

## Design limitations

This is a compact linear MPC baseline, not a nonlinear or constraint-complete automotive MPC. It does not model actuator lag, steering-rate limits, obstacle avoidance, speed planning, localization uncertainty or road-boundary constraints. The local straight-line error model is most appropriate for modest tracking errors and curvature. The optimizer runs synchronously inside `compute`; benchmark the selected horizon and iteration count against the target control period before deployment.

## Verification

`tests/path_tracking/mpc_path_tracker_test.cpp` checks configuration validation, zero-error equilibrium, steering correction direction, saturation, nearest-segment selection, duplicate vertices, transactional failure behavior and closed-loop lateral convergence.

`benchmarks/mpc_path_tracker_benchmark.cpp` measures end-to-end command generation for a fixed 501-point sinusoidal path, including finite-horizon optimization.
