# P3-10 Rigid-body manipulator dynamics

## Scope

This header-only reference implementation computes the fixed-base serial-chain manipulator mass matrix, generalized Coriolis/centrifugal vector, gravity vector, and their sum as bias forces. It uses SI units and is independent of ROS 2/Gazebo.

## Inertial data

Supply one `LinkInertial` record per `RobotModel::links` entry, including the base. Each record contains non-negative mass, a center of mass expressed in that link's frame, and a 3x3 inertia tensor about the center of mass expressed in the same link frame. A massless base can be represented with zero mass and zero inertia. DH/URDF geometry alone is insufficient to infer inertial parameters.

## API

- `RigidBodyDynamics::mass_matrix(model, inertias, q, M)` computes (M(q)) from COM translational Jacobians and rotated COM inertia tensors.
- `RigidBodyDynamics::coriolis(model, inertias, q, dq, c)` computes the generalized velocity-dependent vector (c=C(q,dq)dq) using central finite differences of (M(q)) and Christoffel symbols.
- `RigidBodyDynamics::gravity(model, inertias, q, gravity_acceleration, G)` computes the generalized gravity vector for a base-frame gravity acceleration such as `{0,0,-9.81}`.
- `bias_forces(...)` returns (c+G).

The mass matrix is symmetrized numerically. Outputs are committed only after successful validation and finite-result checks. Inputs are validated for model consistency, dimensions, finite values, and non-negative mass/inertia diagonals.

## Numerical and performance notes

This is a transparent reference implementation rather than a production real-time recursive Newton–Euler / articulated-body algorithm. Coriolis computation uses finite differences and scales poorly with joint count; replace it with an analytic or recursive method before high-DOF real-time deployment. The inertia tensor is expected to be physically meaningful and symmetric positive semidefinite; this first version validates finite entries and non-negative diagonal values but does not prove physical realizability.

## Validation

Tests use a one-link revolute arm with a point-mass COM to verify (M=mr^2), gravity generalized force, zero-velocity Coriolis behavior, added rotational inertia, and transactional invalid-input handling. CI acceptance: Run #332 passed on both Linux and Windows (Configure/Build/Test); commit `b5fbba25138d20a854c6ec7d143daf75b72c8f2d`. Workflow: https://github.com/rexyll/MotionControlAlgorithms/actions/runs/39585979345.
