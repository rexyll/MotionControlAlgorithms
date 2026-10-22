# P6-02 Task-space hierarchy

This module provides a deterministic differential-IK baseline for ordered task levels. Each level supplies a task Jacobian and desired task-space velocity. The solver computes a damped least-squares update through the remaining null-space projector, then carries the projected subspace to lower-priority tasks.

Inputs are checked for dimensions, finite values, non-negative damping and a positive singular-value rank tolerance. The result (joint velocity, per-level residual norms and accumulated effective rank) is committed only after the full solve succeeds.

This is a practical sequential null-space baseline. Damping and finite-precision projector updates can permit small lower-priority leakage; it is not an exact lexicographic optimization method. Inequality constraints, joint-limit avoidance, collision constraints and contact wrench optimization are outside this task and are addressed by later QP/contact tasks.
