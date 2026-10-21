# P2-09 Wheel-robot disturbance and slip simulation

## Scope

A deterministic command-to-motion model for repeatable planar mobile-robot experiments. It has no ROS 2, Gazebo, sensor, or hardware dependency and can be composed with `PlanarOdometry` to compare ideal and disturbed trajectories.

## Model

For each body-twist channel, the commanded component is attenuated by a slip fraction and combined with an additive sinusoidal disturbance:

`actual = commanded * (1 - slip) + amplitude * sin(2*pi*frequency*time + phase)`

The three independent channels are longitudinal velocity (m/s), lateral velocity (m/s), and yaw rate (rad/s). Slip fractions are constrained to [0, 1], disturbance frequencies must be non-negative, and all values must be finite. The time argument is seconds and must be non-negative.

## API

- `WheelRobotDisturbanceConfig` configures per-axis slip, amplitude, frequency and phase.
- `WheelRobotDisturbanceModel::configure` validates the complete configuration.
- `compute(command, time_s, actual)` calculates the effective body twist without internal state or random-number generation.
- Invalid inputs or non-finite results return false and leave the output unchanged.

Because the model is deterministic for a given configuration, command and timestamp, tests and benchmarks are reproducible. This is a compact disturbance model, not a tire-contact or terrain physics model; richer stochastic or contact-dependent models can be added later.

## Validation

Unit tests cover transparent pass-through, independent slip attenuation, sinusoidal disturbances and repeatability, invalid configuration/input, and output transactional safety. `mca_wheel_robot_disturbance_benchmark` measures fixed-input command transformation cost.
