# P8-12 Multi-Input Control Barrier Function Filter

## Purpose

The filter minimally modifies a vector nominal command while enforcing multiple first-order control-affine CBF conditions and per-axis actuator bounds. For each barrier, the supplied local model must satisfy

\[
L_f h_i(x) + L_g h_i(x)u + \alpha h_i(x) \ge 0.
\]

It projects the nominal command by minimizing one-half of the squared Euclidean distance over the feasible intersection.

## Algorithm and scope

Actuator bounds are converted to linear half-spaces. The implementation enumerates independent active sets of up to the control dimension, checks every candidate against all inequalities, and selects the feasible candidate with the smallest deviation from the nominal command. The total number of barrier and actuator half-spaces is capped at 20 (default 16). This deterministic method is intended for small, low-dimensional control problems; runtime grows combinatorially. Larger problems should use a dedicated QP solver and benchmark worst-case latency.

## Failure semantics

- Invalid dimensions, non-finite values, invalid bounds or excessive constraint count return an error.
- Infeasible intersections return ConstraintViolation.
- Output is transactional and changes only after a finite feasible solution is found.
- The minimum safety margin covers CBF residuals, not actuator-bound residuals.

## Validation

Tests cover projection, unchanged safe nominal commands, actuator bounds, infeasibility, invalid inputs, excessive constraint counts and transactional output preservation.
