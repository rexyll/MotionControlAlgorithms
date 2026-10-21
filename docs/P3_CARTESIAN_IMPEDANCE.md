# P3-14 Cartesian impedance control

## Scope

`CartesianImpedance` implements a fixed-base task-space virtual spring-damper controller. The equilibrium pose defines the virtual spring reference; task-space stiffness and damping define the desired compliance. A feedforward wrench can be added for gravity/contact bias compensation, and an optional symmetric wrench limit bounds the commanded virtual interaction force/torque before Jacobian-transpose mapping.

## Control law

Using base-frame ordering `[vx, vy, vz, wx, wy, wz]`:

`e = [p_eq - p, Log(R_eq R^T)]`

`V = J(q) dq`

`W = W_ff + K e + D (V_d - V)`

`tau = J(q)^T W`

`K` and `D` are diagonal non-negative stiffness/damping vectors. The implementation is intentionally stateless and does not require a robot dynamics model or an operational-space inertia matrix.

## Validation

Reference tests cover:

- virtual spring force mapped through the Jacobian transpose;
- damping opposing measured Cartesian velocity;
- symmetric wrench limiting before torque mapping;
- invalid stiffness rejection with transactional output preservation.

The benchmark exercises the complete FK/Jacobian/controller path on a deterministic six-revolute-link model.

## Safety and limitations

This is a Cartesian virtual spring-damper impedance law, not a complete contact-safety system. Wrench limits are software command clamps and do not replace drive current/torque limits, collision detection, force sensing, watchdogs, or hardware safety mechanisms. The implementation does not model operational-space inertia, Coriolis terms, contact dynamics, or environment stiffness.

## CI acceptance

Run #364 passed on both Linux and Windows with Configure, Build and Test successful for commit `51e64773c913263630984d4d9b6bfdc7348028ad`.
