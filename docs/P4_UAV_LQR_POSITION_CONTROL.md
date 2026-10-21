# P4-05 UAV LQR position control

## Overview

`mca::uav::LqrPositionController` implements three independent discrete-time LQR loops for world-frame translational position and velocity. Each axis uses acceleration as the control input and a sampled double-integrator model. The discrete algebraic Riccati equation is solved during configuration, so the update path does not perform matrix iterations or allocate memory.

## Model and control law

For each axis, the state is the position and velocity error, with sampled dynamics

`x[k+1] = A x[k] + B a[k]`, where `A = [[1, dt], [0, 1]]` and `B = [dt^2/2, dt]^T`.

The cost uses positive position and velocity weights and a positive acceleration penalty. After solving the discrete Riccati equation, the controller applies `a = a_ff - Kp (p - p_des) - Kv (v - v_des)`. The acceleration output is clamped independently per axis to the configured limits. Position, velocity, and feedforward acceleration are expressed in world coordinates.

## Validation and limitations

Configuration rejects non-finite or non-positive sample time, weights, and acceleration limits, and fails if the Riccati iteration does not converge. Updates reject non-finite state/reference values and preserve the caller's output on failure. The controller assumes decoupled translational axes and does not itself convert acceleration into attitude, thrust, or motor commands; a flight stack must account for gravity, tilt limits, thrust limits, drag, and actuator dynamics.

## Tests and benchmark

Unit tests cover configuration validation, zero-error feedforward, gain feedback, per-axis saturation, transactional invalid-input behavior, and a closed-loop double-integrator convergence scenario. The deterministic benchmark measures only the update path.
