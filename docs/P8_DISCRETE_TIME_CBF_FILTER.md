# P8-14 — Discrete-Time Multi-Input CBF Safety Filter

## Purpose

Enforce a one-step discrete-time control barrier condition for systems whose next-step barrier prediction is affine in the control. This complements continuous-time CBF filters when the controller operates at a fixed sampling period.

## Condition

For each barrier value h_i[k], the filter assumes a supplied prediction

\[
h_i[k+1;u] = d_i + G_i u
\]

and enforces

\[
h_i[k+1;u] \ge (1-\gamma)h_i[k] + s,
\qquad 0 < \gamma \le 1,\quad s\ge 0.
\]

Equivalently, it projects the nominal control onto the half-spaces

\[
G_i u + d_i - (1-\gamma)h_i[k] - s \ge 0
\]

and per-axis actuator bounds. The projection minimizes squared Euclidean distance from the nominal command.

## API and validation

- `DiscreteTimeCbfConfig`: decay parameter `gamma`, actuator bounds, tolerance, safety margin and constraint-count cap.
- `DiscreteTimeCbfInput`: current barrier values, control-independent one-step predicted barrier, control-effect matrix and nominal control.
- `DiscreteTimeCbfOutput`: filtered control, minimum predicted barrier residual, active-constraint count and modification flag.
- Invalid dimensions, non-finite values and invalid configuration are rejected; infeasible constraints return `ConstraintViolation` without modifying the caller's output.

The implementation reuses the deterministic active-set projection in `MultiInputCbfFilter`, so it is intended for small, low-dimensional control inputs and bounded constraint counts.

## Limitations

The caller must provide a valid one-step barrier prediction under the actual sampling model. This component does not construct system dynamics, estimate state, handle delays or uncertainty, or independently guarantee safety when predictions are inaccurate. Sampling time, state-estimation quality, actuator dynamics and model error must be considered by the system integrator.
