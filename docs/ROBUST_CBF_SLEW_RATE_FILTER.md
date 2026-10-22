# Robust Discrete-Time CBF Filter with Actuator Slew-Rate Limits

`RobustCbfSlewRateFilter` composes a worst-case discrete-time control barrier constraint with actuator magnitude and per-update slew-rate bounds.

For each control axis, the feasible actuator interval is intersected with the interval around the last applied control:

`u_min_eff = max(u_min, u_prev - delta_max)`

`u_max_eff = min(u_max, u_prev + delta_max)`

For each barrier, the robust prediction subtracts the configured absolute additive error bound from the nominal drift before the CBF projection. The resulting command is the closest feasible control to the nominal request that satisfies the worst-case one-step barrier constraint within the intersected actuator limits.

## Failure behavior

- Invalid dimensions, non-finite previous controls, or negative/non-finite slew limits return `InvalidArgument`.
- Invalid uncertainty-bound dimensions, negative values, or non-finite values return `InvalidArgument`.
- An empty magnitude/slew intersection or infeasible robust safety projection returns `ConstraintViolation`.
- Output is committed only after successful robust projection.

`previous_control` should be the last applied actuator command. Slew limits are per update and use the same units as control. A zero uncertainty bound reduces to the non-robust slew-rate filter. The safety guarantee is only as sound as the supplied one-step prediction and error bounds.

## Validation

Tests cover worst-case safety and slew enforcement, zero-uncertainty equivalence, uncertainty-induced infeasibility, invalid dimensions/bounds, and transactional output preservation.
