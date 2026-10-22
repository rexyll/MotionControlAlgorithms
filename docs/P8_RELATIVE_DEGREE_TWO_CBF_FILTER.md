# P8-13 — Relative-Degree-Two Multi-Input CBF Safety Filter

## Purpose

P8-13 extends the multi-input first-order CBF safety filter to safety functions whose control influence appears at relative degree two. It minimally modifies a vector nominal command while enforcing multiple exponential-CBF inequalities and actuator bounds.

For each barrier `h_i(x)`, the supplied local derivatives are assumed to satisfy

\[
\ddot h_i = L_f^2 h_i + L_gL_f h_i\,u.
\]

With positive cascade gains `alpha1` and `alpha2`, the filter enforces

\[
L_f^2 h_i + L_gL_f h_i\,u + \alpha_1 \dot h_i + \alpha_1\alpha_2 h_i \ge s,
\]

where `s >= 0` is the requested safety margin.

## Algorithm

The implementation converts each relative-degree-two condition into the affine inequality

\[
a_i u + b_i \ge s
\]

and delegates projection to the existing deterministic `MultiInputCbfFilter`. The nominal vector is therefore projected onto the intersection of all barrier half-spaces and per-axis actuator bounds.

The underlying active-set enumeration is intentionally limited to small, low-dimensional control problems. It is not a replacement for a general-purpose QP solver.

## API

- `RelativeDegreeTwoCbfConfig`: positive cascade gains, actuator bounds, coefficient tolerance, safety margin and constraint-count cap.
- `RelativeDegreeTwoCbfInput`: barrier values, first derivatives, second-order drift derivatives, control derivatives `LgLf h`, and nominal control.
- `RelativeDegreeTwoCbfOutput`: filtered control, minimum effective CBF margin, active constraint count and modification flag.
- `filter()` returns `ConstraintViolation` when the actuator-bounded intersection is infeasible; output remains unchanged on failure.

## Validation

Tests cover the cascaded relative-degree-two inequality, unchanged safe nominal commands, infeasibility with transactional output preservation, dimension validation and non-finite input rejection. Linux and Windows CI remain the acceptance gate.

## Scope and limitations

The filter assumes the caller supplies valid local derivatives and a model for which the relative-degree-two exponential CBF construction is appropriate. It does not estimate derivatives, compensate model uncertainty, or by itself guarantee safety under sampling delay, state-estimation error, unmodelled dynamics or actuator faults.
