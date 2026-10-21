# P3-04 Numerical Inverse Kinematics

`mca::kinematics::NumericalIk` iteratively finds movable-joint coordinates for a target end-effector pose using the existing forward-kinematics model.

## Method

- Computes Cartesian translation error and SO(3) logarithmic orientation error.
- Estimates the 6-by-N geometric Jacobian by finite differences.
- Uses a damped least-squares update `dq = J^T (J J^T + lambda^2 I)^-1 e`.
- Clamps per-iteration joint increments and, by default, clamps coordinates to each joint's declared limits.
- Returns explicit status, iteration count, final position/orientation residuals, joint coordinates and a diagnostic message.

## Limitations

This is a general iterative numerical solver, not a global planner. Convergence depends on the initial configuration; unreachable targets may stall or exhaust the iteration budget. Damping improves behavior near rank deficiency but does not guarantee a solution. Fixed joints are excluded from the coordinate vector. Revolute/continuous coordinates are radians and prismatic coordinates are metres.

## Verification

Unit tests cover a reachable analytic single-joint target, invalid coordinate/options, and iteration-limit handling. The algorithm is header-only and has no ROS 2/Gazebo dependency.
