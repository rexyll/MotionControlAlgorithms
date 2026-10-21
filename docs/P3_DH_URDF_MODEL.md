# P3-02 — DH / URDF model adapter

## Scope

P3-02 provides a ROS2/Gazebo-independent serial-manipulator model representation and two adapters:

- `DhModelAdapter`: builds a serial chain from standard Denavit–Hartenberg parameters.
- `UrdfModelAdapter`: parses a dependency-free URDF subset into the same model representation.

The adapter is deliberately separate from forward kinematics, Jacobians and IK; those are implemented in later P3 tasks.

## Model representation

`mca::kinematics::RobotModel` contains ordered links and joints, root/tip names, joint types, joint-frame origin poses, normalized axes and limits. `RobotModel::valid()` verifies the serial-chain invariant and finite geometry.

Supported joint types are fixed, revolute, continuous and prismatic.

## DH convention

P3-02 uses standard DH parameters `(a, alpha, d, theta)`. The adapter stores the zero-configuration transform for each parameter row. Joint axis is the local +Z axis; the variable joint coordinate is intentionally left to P3-03 FK.

## URDF subset

Supported elements are `robot`, `link`, `joint`, `parent`, `child`, `origin`, `axis` and `limit`. Supported joint types are `fixed`, `revolute`, `continuous` and `prismatic`.

The parser accepts single-line or multiline serial-chain URDF text, normalizes non-zero joint axes, applies identity origin defaults, and converts URDF roll-pitch-yaw to the existing SO(3) quaternion type.

The adapter rejects unknown links, duplicate links, unsupported joint types, zero axes for movable joints, multiple parents, multiple roots and branched/non-serial chains. Parsing is transactional: the output model is changed only after a complete valid model has been built.

This is intentionally a small dependency-free parser for the core algorithm library. Full URDF ecosystem features and ROS resource resolution belong in the P9 integration layer.

## Validation

Tests cover DH construction, parameter validation and transactional behavior; multiline URDF ordering, joint types, normalized axes, limits, defaults and malformed/branched models. A deterministic micro-benchmark measures repeated DH construction and URDF parsing.
