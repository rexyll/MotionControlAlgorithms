# P3-06 Geometric Jacobian

`mca::kinematics::Jacobian` computes the manipulator geometric Jacobian from the robot model and joint coordinates without numerical differencing.

## API and conventions

- `Jacobian::solve(model, q, result, error)` validates inputs through forward kinematics and writes the output transactionally.
- `JacobianResult::rows` stores six rows in base coordinates: linear velocity `vx, vy, vz`, followed by angular velocity `wx, wy, wz`.
- Columns follow the model joint order with fixed joints omitted, matching the coordinate vector consumed by forward kinematics.
- Revolute/continuous columns use `[axis × (tip − joint_origin); axis]`; prismatic columns use `[axis; 0]`.
- URDF axes are transformed through the joint-origin pose. Standard-DH joints use the parent-frame z axis.
- The core API has no ROS 2 or Gazebo dependency.

The result maps joint rates to the base-frame end-effector twist locally. It is a geometric Jacobian, not an analytical Euler-angle-rate Jacobian.

## Verification

Analytical planar two-link values, a mixed fixed/revolute/prismatic chain with a rotated prismatic axis, and transactional invalid-input handling are covered by unit tests. Dual-platform CI validation is pending.
