# P1 Single-Axis Servo Validation

## Cascaded position/velocity/torque controller

The reference scalar servo is intentionally split into two layers:

- `mca::control::CascadedServo`: controller-only composition.
- `mca::experiment::CascadedServoExperiment`: deterministic benchmark composition using the DC motor plant.

The controller executes three nested PID loops at the caller-supplied period:

```text
q_ref
  -> position PID
  -> velocity_ref
  -> velocity PID
  -> torque_ref
  -> torque PID
  -> actuator voltage
```

The DC motor then applies:

```text
L di/dt = V - R i - Ke omega
J domega/dt = Kt i - B omega - tau_load
```

with semi-implicit Euler integration.

## Benchmark outputs

`CascadedServoExperiment` reports:

- position RMSE and maximum absolute error
- position overshoot normalized by step amplitude
- position settling time using a configurable tolerance band and hold time
- velocity tracking RMSE and maximum error
- torque tracking RMSE and maximum error
- actuator constraint violation count and maximum violation
- final position, velocity, current, torque and voltage

The experiment is deterministic: it uses a fixed simulation step and does not depend on wall-clock timing.

## Acceptance baseline

The initial scalar acceptance test uses:

- `dt = 100 us`
- `10000` simulation steps
- `1 rad` position step
- `12 V` actuator limit
- `10 A` current monitoring limit
- DC motor parameters defined directly in the test

The test requires the final position to be within `0.01 rad`, final velocity below `0.05 rad/s`, final torque below `0.01 N*m`, and position RMSE below `0.15 rad`.

A separate test deliberately lowers the voltage limit and verifies that the benchmark records a constraint violation while the applied voltage remains clamped.

## Real-time boundary

The controller classes do not allocate dynamically during `update()` and do not depend on ROS 2, Gazebo, files or network APIs. The experiment is a test/benchmark composition and is not intended to define the hard-real-time execution boundary.

Future FOC/PMSM work will keep the electrical current loop separate from this higher-level cascade so that the architecture remains:

```text
position -> velocity -> torque -> current/FOC -> SVPWM -> motor
```
