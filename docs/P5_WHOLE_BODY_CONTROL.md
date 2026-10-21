# Whole-body torque composition baseline

P5-06 provides a deterministic baseline for combining joint feedforward/model compensation, multiple task-space wrench objectives, and a secondary joint-torque term.

For each task, the Jacobian maps generalized velocity to task velocity and the wrench uses its matching task-force convention:

```text
tau = tau_feedforward + sum(J_i^T * wrench_i) + tau_secondary
tau = clamp_per_joint(tau, -limit, +limit)
```

The API validates dimensions and finite values, supports optional per-joint symmetric torque limits, and only replaces the output on success. An empty torque-limit vector disables clipping.

This is a composition primitive, not a constrained whole-body QP: it does not solve contact inequalities, friction cones, floating-base dynamics, or strict task priorities. Use P5-07 for optimization-based task hierarchy and constraint handling.
