# P1-16 PID/PD + Feedforward Comparison Runner

## Purpose

`mca::experiment::PidFeedforwardComparison` provides a deterministic, ROS2/Gazebo-independent experiment for comparing the same PID/PD controller with and without model-based scalar feedforward.

The runner executes both cases against identical second-order plants, initial state, reference trajectory and timestep. This makes the comparison suitable for regression tests and controller-tuning experiments rather than a one-off demo.

## Control law

Baseline:

```text
u = PID(reference, position)
```

Feedforward case:

```text
u = PID(reference, position) + kv * v_ref + ka * a_ref + bias
```

For a second-order plant

```text
J*qdd + B*qd + K*q = u
```

the velocity/acceleration model terms can be selected as:

```text
kv = B
ka = J
```

when `K = 0`. A nonzero stiffness term is intentionally not hidden inside the feedforward implementation; position-dependent feedforward can be added explicitly by the caller when required.

## Reference experiment

The acceptance scenario uses a one-second scalar ramp from 0 to 1 rad followed by a hold. Both cases use the same:

- `dt = 1 ms`
- 2000 samples
- `J = 1 kg*m^2`
- `B = 0.5 N*m*s/rad`
- PD gains `Kp = 10`, `Kd = 4`
- feedforward `kv = 0.5`, `ka = 1.0`

The runner reports RMSE, maximum absolute error, RMS control effort, normalized overshoot, settling time, final position and final velocity for both cases, plus relative RMSE and control-effort changes.

## Design constraints

- no dynamic allocation in the runtime path;
- deterministic fixed-step simulation;
- no ROS2/Gazebo dependency;
- identical plant/reference conditions for both cases;
- invalid numerical/configuration inputs are rejected;
- benchmark target is registered when Google Benchmark is available.

## Acceptance

P1-16 is complete only after Linux and Windows CI pass Configure/Build/Test, the comparison test demonstrates a measurable RMSE improvement from model-based feedforward, documentation is present, and the benchmark target builds.
