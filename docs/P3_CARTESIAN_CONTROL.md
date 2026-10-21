# P3-13 Cartesian control

## Scope and control law

`CartesianControl` implements a stateless task-space PD controller for a fixed-base serial manipulator. The desired pose is represented by `Pose3D`; desired twist and feedforward wrench use the base-frame ordering `[vx, vy, vz, wx, wy, wz]`.

The controller computes

`e = [p_d - p, Log(R_d R^T)]`

`V = J(q) dq`

`W = W_ff + Kp e + Kd (V_d - V)`

`tau = J(q)^T W`

with optional symmetric per-joint torque limits.

## Validation and limitations

Tests cover translational Jacobian-transpose mapping, orientation feedback with torque saturation, and transactional invalid-gain handling. The implementation intentionally uses the existing FK/Jacobian and SO(3) primitives and does not introduce a dynamics model. It is therefore a task-space PD/feedforward controller rather than full operational-space inverse-dynamics control. Torque limits are a software clamp, not a substitute for drive-level safety protections.

Cross-platform CI acceptance is pending.
