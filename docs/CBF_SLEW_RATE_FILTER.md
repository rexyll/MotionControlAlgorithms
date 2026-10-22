# Discrete-Time CBF Filter with Actuator Slew-Rate Limits

`CbfSlewRateFilter` combines a discrete-time control barrier function (CBF) constraint with actuator magnitude and per-update slew-rate bounds.

For each control axis i, the feasible interval is the intersection of the configured actuator interval and the interval around the last applied control:

`u_min_eff[i] = max(u_min[i], u_prev[i] - delta_max[i])`

`u_max_eff[i] = min(u_max[i], u_prev[i] + delta_max[i])`

The existing discrete-time CBF projection then selects the feasible control closest to the nominal command while enforcing the one-step safety constraints. `previous_control` should represent the last applied command, and `max_control_delta` is the maximum absolute change allowed on this update (per-axis, in the same units as control).

## Failure behavior

- Invalid vector dimensions, non-finite previous controls, or negative/non-finite slew limits return `InvalidArgument`.
- An empty intersection between magnitude and slew bounds returns `ConstraintViolation`.
- If the CBF constraints are infeasible inside the intersection, the underlying CBF status is propagated.
- The caller's output is only replaced after a successful safety projection.

A zero slew limit is valid and freezes that control axis for the current update. This component constrains control changes; it does not itself model actuator dynamics or guarantee safety if the one-step barrier prediction is inaccurate.

## Validation

The unit tests cover slew-bound enforcement, equivalence to the nominal discrete-time CBF filter when rate limits are inactive, infeasible intersections, invalid dimensions/limits, and transactional output preservation.
