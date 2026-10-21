# P5-08 Joint impedance / torque control

## Scope

P5-08 adds a deterministic joint-space impedance torque primitive for floating-base/legged control compositions. The law is

`tau = tau_ff + K (q_des - q) + D (dq_des - dq)`

followed by optional symmetric per-joint torque saturation.

## Design

- `position` and `velocity` are measured joint states.
- `desired_position` and `desired_velocity` define the virtual spring-damper target.
- `stiffness` and `damping` are per-joint non-negative gains.
- `feedforward_torque` carries gravity, dynamics, contact or other model-based compensation supplied by a higher-level controller.
- `torque_limits` is optional; when present, each joint is clamped to `[-limit, +limit]`.
- The primitive is stateless and has no ROS 2/Gazebo dependency.
- Invalid input is rejected transactionally: the output is not modified on failure.

This intentionally separates the impedance law from model-based compensation and safety/state-estimation layers. It can therefore be composed with the existing centroidal/whole-body and dynamics primitives.

## Validation

The unit tests cover the analytical spring-damper result, per-joint saturation, negative-gain rejection, non-finite input rejection with output preservation, and convergence of a one-joint unit-inertia reference plant.

The benchmark measures the stateless controller for 12- and 24-joint vectors.

## Limitations / next steps

This is a joint-space torque primitive rather than a full actuator model. It does not include torque-rate limiting, motor current limits, friction compensation, passivity observers, or floating-base contact constraints. Those belong to subsequent composition/safety layers.
