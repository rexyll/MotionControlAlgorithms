# P1-15 Step-response metrics

`mca::analysis::compute_step_response_metrics` provides deterministic, allocation-free metrics for sampled scalar step responses.

## Metrics

Given `initial` and `target`, define the signed step as

`step = target - initial`

and its amplitude as `abs(step)`.

### Overshoot

Overshoot is normalized by the step amplitude:

`overshoot_fraction = max(0, directional_peak - target) / abs(target - initial)`

For a positive step, only values above the target contribute. For a negative step, only values below the target contribute. `overshoot_percent` is the same value multiplied by 100.

This normalization makes a 10% overshoot equal to `0.10` for both positive and negative steps.

### Settling time

The settling band is

`abs(actual - target) <= tolerance_ratio * abs(target - initial)`.

A candidate settling time starts at the first sample entering the band. It becomes valid only after the response has remained continuously inside the band for at least `hold_time_s` **and** all subsequent supplied samples remain inside the band. A later excursion therefore invalidates an earlier candidate.

This definition supports nonuniform sample spacing because elapsed time is calculated from the supplied timestamps rather than assuming a fixed sample period.

If the response does not settle in the supplied data, `settled` is false and `settling_time_s` is NaN.

## API properties

- C++17 compatible.
- No dynamic allocation.
- ROS 2 / Gazebo independent.
- Requires finite values and strictly increasing nonnegative timestamps.
- Rejects zero-amplitude steps.
- Recommended tolerance ratios are 0.02 (2%) or 0.05 (5%).
- `hold_time_s` may be zero when a pure tolerance-band definition is desired.

## Acceptance coverage

The unit tests cover positive and negative steps, normalized overshoot, no overshoot, exact tolerance-boundary behavior, nonuniform timestamps, non-settling responses, later excursions, and invalid input.

The benchmark evaluates a 1000-sample underdamped-like response without allocations in the metric implementation.
