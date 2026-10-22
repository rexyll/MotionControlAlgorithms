# P8-10 — Reference command governor

## Purpose

A reference governor smooths a requested scalar position before it reaches a downstream controller. This implementation enforces configured position bounds and limits the magnitude of reference velocity and acceleration.

## Update law

For current governed position x, velocity v, target r, sample interval Δt, maximum velocity v_max and acceleration a_max:

- Position error: e = r − x.
- Continuous stopping-speed bound: v_stop = √(2 a_max |e|).
- Discrete stopping-speed bound: v_disc = √((a_max Δt)² + 2 a_max |e|) − a_max Δt, reserving one sample of travel for the semi-implicit position update.
- Velocity target: v* = sign(e) min(v_max, v_stop, v_disc).
- Acceleration-limited velocity: v_next = clamp(v*, v − a_max Δt, v + a_max Δt).
- Position integration: x_next = x + v_next Δt, with velocity additionally constrained to the interval that keeps x_next inside [x_min, x_max]. If the position, velocity, and acceleration constraints have no common feasible next velocity, update returns `ConstraintViolation` without mutating state or output.
- Reported velocity and acceleration are computed from the actual bounded position increment and velocity change.

The stopping-speed term reduces the requested speed as the remaining distance shrinks. The feasible-velocity intersection preserves position bounds without using a hard position clamp that could create an acceleration spike. Callers must choose a suitable update interval and account for downstream actuator dynamics.

## API and failure behavior

Configure finite ordered position bounds and positive finite velocity/acceleration limits, then call reset with an initial in-bounds position and admissible velocity. Update rejects non-finite values, non-positive dt, and target positions outside configured bounds. Invalid calls leave both the caller's output and internal state unchanged. Reconfiguration requires a fresh reset.

## Scope and limitations

This is a scalar, discrete-time command-smoothing primitive, not a trajectory planner or a proof of closed-loop stability. It does not model jerk, actuator lag, multi-axis synchronization, collision geometry, or downstream tracking error. Integrate it upstream of the relevant controller and validate sample timing and limits for the actual robot.

## Verification

Unit tests cover velocity/acceleration bounds, position bounds, reset lifecycle, invalid-input transactional behavior, and target convergence. Linux and Windows CI are required before acceptance.
