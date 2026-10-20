# P1-11 Friction Compensation

## Scope

`mca::control::FrictionCompensator` provides a platform-independent torque feedforward term for a single rotational axis. It is intended to be added to a feedback controller output:

`tau_command = tau_feedback + tau_compensation`

The implementation has no ROS 2 or Gazebo dependency and performs no dynamic allocation in the runtime path.

## Model

The compensation uses three terms:

- static/breakaway friction: `tau_static * sign(tau_requested)` while the measured velocity is inside the configured deadband;
- Coulomb friction: `tau_coulomb * sign(omega)` outside the velocity deadband;
- viscous friction: `b * omega` at all velocities.

Therefore, outside the deadband:

`tau_comp = b * omega + tau_coulomb * sign(omega)`

Inside the deadband:

`tau_comp = b * omega + tau_static * sign(tau_requested)`

If both velocity and requested torque are exactly zero, the compensation is zero. This avoids an arbitrary torque direction at rest.

The compensation is deliberately a feedforward command rather than an estimate of the physical friction force. The caller is responsible for adding it to the feedback torque and for applying any final actuator torque limits.

## Configuration

`FrictionCompensationConfig` contains:

- `static_friction_nm`
- `coulomb_friction_nm`
- `viscous_friction_nm_per_rad_s`
- `velocity_deadband_rad_s`
- independent minimum/maximum compensation limits

All physical coefficients must be finite and non-negative. The deadband may be zero.

## Numerical and control considerations

The velocity deadband prevents sign chatter around zero velocity. The requested torque determines the breakaway direction when stopped. This is useful for position servoing because a position error can request the direction in which the axis needs to start moving.

The model is intentionally simple and deterministic. It does not attempt to identify friction online, model Stribeck curves, temperature dependence, backlash, or hysteresis. Those behaviors can be added as separate higher-level models later without changing this API.

## Tests

The unit tests cover:

- invalid configuration and non-finite input rejection;
- Coulomb plus viscous compensation for positive and negative velocity;
- breakaway direction from requested torque at rest;
- zero command at exact rest;
- independent output limits;
- viscous contribution inside the velocity deadband.

## Benchmark

`mca_friction_compensation_benchmark` measures the steady-state `compute()` runtime over a deterministic set of velocities and requested torques.
