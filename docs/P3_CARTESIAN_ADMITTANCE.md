# P3-15 Cartesian admittance control

## Scope

`CartesianAdmittance` is a stateful fixed-base Cartesian admittance controller. It converts an external wrench into a virtual Cartesian acceleration, integrates that acceleration into a commanded twist and pose, and supports a desired equilibrium pose/twist, feedforward wrench and software limits.

## Control law

Using base-frame ordering `[vx, vy, vz, wx, wy, wz]`:

`M a = W_ext + W_ff + K (X_d - X) - D (V - V_d)`

`V_next = V + dt a`

`p_next = p + dt v_linear,next`

`R_next = Exp(dt omega_next) R`

`M` is the positive diagonal virtual mass/inertia, while `K` and `D` are diagonal non-negative stiffness and damping coefficients. The orientation residual is `Log(R_d R^T)`, whose rotation vector is already expressed in the base frame; it must not be rotated a second time. The rotation update uses the existing SO(3) exponential map.

The commanded pose/twist are controller state. `reset()` initializes them explicitly, which makes startup and recovery deterministic.

## Validation

Reference tests cover:

- external wrench to acceleration/velocity/pose integration;
- damping opposing commanded velocity;
- stiffness returning the commanded state toward the desired pose;
- orientation integration through the exponential map;
- orientation error direction with a non-identity current orientation;
- symmetric wrench, acceleration and velocity limits;
- reset behavior and invalid-update state preservation;
- invalid configuration rejection.

The benchmark exercises the complete stateful configure/reset/update path on a deterministic six-dimensional Cartesian controller.

## Safety and limitations

This implementation is a virtual Cartesian admittance filter/controller, not a complete contact-safety system. Software wrench, acceleration and velocity limits do not replace drive current/torque limits, collision detection, force sensing, watchdogs, emergency stops or hardware safety mechanisms. The implementation assumes diagonal virtual mass, damping and stiffness matrices and does not model robot dynamics, operational-space inertia, contact dynamics or environment dynamics.

## CI acceptance

Cross-platform CI acceptance is pending.
