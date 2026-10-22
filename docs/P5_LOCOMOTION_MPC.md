# P5-09 Locomotion MPC

## Scope

P5-09 provides a lightweight linear model-predictive controller for planar legged locomotion at the COM level. It is intentionally independent of ROS 2 and Gazebo and is designed as a deterministic, testable baseline for the later whole-body/contact stack.

## State and dynamics

The state is

`x = [px, py, vx, vy]`

and the control is planar acceleration `u = [ax, ay]`. With sample time `dt`:

- `p(k+1) = p(k) + dt * v(k) + 0.5 * dt^2 * u(k)`
- `v(k+1) = v(k) + dt * u(k)`

## Objective

The finite-horizon objective contains:

- state tracking penalties for position and velocity;
- a separate terminal state penalty;
- quadratic control effort penalty.

The implementation uses projected gradient descent over the complete acceleration sequence and returns the first control for receding-horizon use.

## Constraints and validation

Each acceleration axis has a symmetric box constraint. Inputs are checked for finite values, positive sample time, valid horizon, nonnegative state/terminal weights, positive control weights and limits, positive iteration count, and valid step size. Failed solves do not modify an existing output object.

The default step size is a conservative lightweight baseline; it is not intended to replace a production QP/MPC solver with an analytically tuned Lipschitz bound.

## Acceptance coverage

`tests/legged/locomotion_mpc_test.cpp` covers:

1. standing: near-zero acceleration;
2. forward walking reference: positive forward acceleration and bounded control;
3. disturbance recovery: negative acceleration after a positive velocity disturbance;
4. invalid-input transactional behavior;
5. non-finite reference rejection.

The benchmark evaluates horizons 10, 20 and 30 with a deterministic forward reference.

## Deliberate scope boundary

This is a COM-level linear MPC baseline. It does not yet solve footstep timing, contact switching, friction cones, terrain constraints, centroidal angular momentum, or full-body inverse-dynamics constraints. Those belong to the subsequent contact and whole-body phases.
