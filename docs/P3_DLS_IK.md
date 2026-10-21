# P3-05 Damped Least-Squares Inverse Kinematics

`mca::kinematics::DampedLeastSquaresIk` is a separate pose IK solver with adaptive damping and step acceptance.

## Method

- Uses forward kinematics and a finite-difference 6-by-N pose Jacobian.
- Solves `(J J^T + lambda^2 I)y=e`, then computes `dq=J^T y`.
- Evaluates each candidate against the actual FK pose residual; only cost-reducing steps are accepted.
- Increases damping after rejected steps and decreases it after accepted steps, bounded by configured minimum/maximum damping.
- Enforces per-joint increment and optional joint-coordinate limits.
- Reuses the `NumericalIkResult` and `IkStatus` contract so callers can handle convergence and failure uniformly.

## Options and limitations

Damping adaptation is a bounded backtracking strategy, not a proof of global convergence. The solver can return `UnreachableOrStalled` if no cost-reducing candidate is found within the trial budget. Initial guess quality and task reachability still matter. Revolute coordinates use radians; prismatic coordinates use metres. The header has no ROS 2/Gazebo dependency.

## Verification

Tests cover a reachable one-joint target, convergence on a two-joint chain, near-singular convergence, joint-limit enforcement for an unreachable target, and invalid damping configuration. Linux and Windows Configure/Build/Test passed in [GitHub Actions Run #291](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/39569416658), commit `089d4c0465828c0095905aed87f64e3eab113da9`.
