# P8-10 — Reference command governor

## Purpose

A reference governor smooths a requested scalar position before it reaches a downstream controller. This implementation enforces configured position bounds and limits the magnitude of reference velocity and acceleration.

## Update law

For current governed position x, velocity v, target r, sample interval Δt, maximum velocity v_max and acceleration a_max:

- Position error: e = r − x.
- Stopping-distance velocity target: v* = sign(e) min(v_max, √(2 a_max |e|)).
- Acceleration-limited velocity: v_next = clamp(v*, v − a_max Δt, v + a_max Δt).
- Position integration: x_next = clamp(x + v_next Δt, x_min, x_max).
- Reported velocity and acceleration are computed from the actual bounded position increment and velocity change.

The stopping-speed term reduces the requested speed as the remaining distance shrinks. The discrete-time position clamp preserves hard position bounds; as with any sampled limiter, callers must choose a suitable update interval and account for downstream actuator dynamics.

## API and failure behavior

Configure finite ordered position bounds and positive finite velocity/acceleration limits, then call reset with an initial in-bounds position and admissible velocity. Update rejects non-finite values, non-positive dt, and target positions outside configured bounds. Invalid calls leave both the caller's output and internal state unchanged. Reconfiguration requires a fresh reset.

## Scope and limitations

This is a scalar, discrete-time command-smoothing primitive, not a trajectory planner or a proof of closed-loop stability. It does not model jerk, actuator lag, multi-axis synchronization, collision geometry, or downstream tracking error. Integrate it upstream of the relevant controller and validate sample timing and limits for the actual robot.

## Verification

Unit tests cover velocity/acceleration bounds, position bounds, reset lifecycle, invalid-input transactional behavior, and target convergence. Linux and Windows CI are required before acceptance.
