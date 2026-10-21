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

For each joint, the local transform starts from `RobotJoint::origin`.

- Fixed: no additional motion.
- Revolute/continuous: compose an axis-angle rotation by the supplied joint coordinate.
- Prismatic: translate along the joint axis by the supplied coordinate.

The local transform is then composed with the accumulated parent pose.

This representation also supports the P3-02 standard-DH adapter: the adapter stores the zero-variable DH transform as the joint origin, while FK applies the runtime joint variable.

## Validation

The solver rejects invalid serial models, a mismatched movable-joint coordinate count, and non-finite joint coordinates. Results are committed transactionally only after all link poses are finite.

## Verification and benchmark

`tests/kinematics/forward_kinematics_test.cpp` covers a two-link planar DH reference, fixed/revolute/prismatic composition, URDF origin/axis composition, and transactional invalid-input behavior.

`benchmarks/forward_kinematics_benchmark.cpp` measures deterministic six-joint serial-chain FK throughput.
