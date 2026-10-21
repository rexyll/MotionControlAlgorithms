# P3-03 Forward Kinematics

`mca::kinematics::ForwardKinematics` computes the base-to-tip pose of a validated serial `RobotModel`.

## Scope

- Fixed, revolute, continuous and prismatic joints.
- Joint coordinates are supplied only for movable joints, in model order.
- Revolute/continuous coordinates use radians.
- Prismatic coordinates use metres.
- Existing joint `origin` and normalized joint `axis` are respected.
- Returns the end-effector pose plus every link pose.
- Header-only and independent of ROS 2/Gazebo.

## Composition rule

URDF joints use the parsed parent-to-joint origin followed by motion about/along the joint-frame axis. Prismatic displacement is rotated by the joint origin orientation before being added to the parent-frame position.

DH-adapter joints carry their DH parameters explicitly and use the standard convention:

`Rz(theta + q) * Tz(d + q_prismatic) * Tx(a) * Rx(alpha)`

The revolute variable is applied before link length/offset transforms; for prismatic joints, the variable is added to `d`. This is intentionally distinct from URDF's origin-then-motion convention.

## Validation

The solver rejects invalid serial models, a mismatched movable-joint coordinate count, and non-finite joint coordinates. Results are committed transactionally only after all link poses are finite.

## Verification and benchmark

`tests/kinematics/forward_kinematics_test.cpp` covers a two-link planar DH reference, fixed/revolute/prismatic composition, URDF origin/axis composition, and transactional invalid-input behavior.

`benchmarks/forward_kinematics_benchmark.cpp` measures deterministic six-joint serial-chain FK throughput.
