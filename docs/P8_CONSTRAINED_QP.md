# P8-04 Constrained QP Infrastructure

## Scope

`mca/control/constrained_qp.hpp` provides a dependency-free baseline convex quadratic-programming primitive:

\[
\min_x \frac12 x^T Hx + g^Tx
\]

subject to

\[
Ax=b,\qquad l\le x\le u.
\]

The Hessian is required to be symmetric positive semidefinite. Equality rows must be linearly independent.

## Algorithm

The solver uses ADMM with a split variable for the box constraint. The `x` update solves the equality-constrained quadratic subproblem through its KKT system; the `z` update is an element-wise projection onto `[l,u]`; the scaled dual variable is then updated in the standard ADMM form.

This gives a deterministic baseline that is useful for MPC, whole-body control and allocation layers without introducing an external optimization dependency.

## Validation

The implementation validates dimensions, finite inputs, bound ordering, Hessian symmetry/positive-semidefiniteness, equality rank and solver options. Output is transactional: failures do not overwrite an existing result.

Tests cover:

- box-constrained convex optimum;
- equality plus box constraints;
- invalid bounds with transactional output preservation;
- rejection of a non-convex Hessian.

A micro-benchmark is included alongside the existing optimization benchmark target.

## Limitations

This is intentionally a baseline solver, not a replacement for mature sparse QP packages. It currently supports equality constraints plus element-wise bounds, but not arbitrary linear inequalities, warm-start APIs, sparse matrices, infeasibility certificates, or advanced active-set/interior-point strategies. Real-time production deployments should benchmark convergence and worst-case iteration counts for their robot-specific problem sizes.
