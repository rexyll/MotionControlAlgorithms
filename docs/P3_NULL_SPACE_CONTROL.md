# P3-16 Null-space control

## Scope

`mca::control::NullSpaceControl` computes a joint-space projector for a task Jacobian and projects a secondary joint-velocity objective into the task null space. It is intended for redundant manipulators where a secondary objective can be used for posture preference or joint-limit avoidance without intentionally changing the primary task velocity.

## Equations

For a task Jacobian \(J\), the Moore-Penrose pseudoinverse \(J^+\), and secondary velocity \(\dot q_s\):

\[
N = I - J^+J, \qquad \dot q_{null}=N\dot q_s.
\]

The zero-damping mode truncates singular values at the configured positive tolerance. A positive damping value uses \(\sigma/(\sigma^2+\lambda^2)\) for each singular value. Damping improves numerical behavior near singularities, but the resulting matrix is a damped projector and does not guarantee exact task decoupling.

## API and validation

The static `compute` API accepts a finite non-empty Jacobian, a secondary velocity vector matching the Jacobian column count, optional non-negative damping, and a positive finite singular-value tolerance. Outputs are only assigned after successful validation and computation. The implementation uses Eigen's Jacobi SVD and has no ROS 2 dependency.

## Tests and benchmark

Reference tests cover an analytical one-row Jacobian, a redundant two-task/three-joint case, projector idempotence in the exact pseudoinverse mode, damped near-singular inputs, invalid-input state preservation, and damping validation. The benchmark repeatedly evaluates a deterministic 6-by-7 task Jacobian with damped pseudoinverse computation.

## Safety and limitations

This utility produces a kinematic velocity projection, not a complete whole-body controller. It does not enforce joint position/velocity/acceleration limits, collision constraints, torque limits, dynamic consistency, or real-time deadlines. The caller must combine the result with primary-task control and appropriate constraint/safety handling. For damped inversion, secondary motion can leak into the primary task near singularities.
