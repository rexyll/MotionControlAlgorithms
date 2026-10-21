# P3-11 Gravity compensation

## Scope

`GravityCompensation` wraps the rigid-body gravity vector as a joint-space feedforward torque command for a fixed-base serial manipulator. It supports a non-negative commissioning scale and optional symmetric per-joint absolute torque limits.

## API and behavior

- `GravityCompensation::compute(...)` delegates gravity calculation to `RigidBodyDynamics::gravity` using the supplied joint positions, inertial parameters, and base-frame gravity acceleration.
- Scale `0.0` disables feedforward; `1.0` applies the full model estimate.
- An empty limits vector means no additional software clamp. Otherwise it must contain one finite, non-negative absolute limit per joint.
- Output is transactional: invalid inputs leave the caller's output unchanged.

## Safety and limitations

This is feedforward only, not a stabilizing feedback controller. Model errors, payload changes, friction, actuator limits, and gravity-vector conventions can cause residual torque. Keep independent drive-level current/torque limits and emergency-stop behavior enabled. Commission from zero scale and increase only after verifying sign and magnitude on a safely supported robot.

## Validation

Reference tests cover analytical gravity output, scale behavior, saturation, empty limits, invalid options, and transactional output. Cross-platform CI acceptance is pending.
