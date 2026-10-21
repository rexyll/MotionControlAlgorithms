# P5-04 Gait scheduler

The gait scheduler maps elapsed time and per-leg cycle offsets to each leg's cycle phase, stance/swing state, and normalized progress within the active phase.

## Phase convention

For period (T), time (t), and per-leg offset (o_i) measured in cycles:

`phase_i = wrap(t/T + o_i, [0,1))`

A leg is in stance while `phase_i < duty_factor`; otherwise it is in swing. The normalized phase is mapped independently to `[0,1)` over stance or swing. The duty factor must be strictly between zero and one.

## API and behavior

- `GaitSchedulerParameters`: positive cycle period, duty factor and one offset per leg.
- `GaitSchedulerOutput`: a `LegGaitPhase` per configured leg, preserving input ordering.
- `evaluate_gait`: supports negative time and unnormalized offsets, rejects non-finite/invalid input and preserves output on failure.
- Default four-leg offsets `{0, 0.5, 0.5, 0}` schedule diagonal phase pairs; actual leg ordering and duty factor remain caller-configurable.

This component is a phase clock, not a footstep planner or contact estimator. It does not adapt timing to terrain, measured forces, or missed contacts. The output is deterministic for a given parameter set and time.
