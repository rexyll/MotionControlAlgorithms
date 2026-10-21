# P5-07 QP task hierarchy baseline

This module provides a deterministic convex weighted quadratic-programming baseline for combining task levels. Each task contributes

`0.5 * w * ||A x - b||^2`

to the objective, with a small positive regularization term making the Hessian positive definite. Optional lower/upper bounds are enforced by projected gradient descent.

The task levels are supplied in priority order and use decreasing weights to encode practical priority. This is intentionally a **weighted hierarchy**, not an exact lexicographic hierarchical-QP solver. Exact priority constraints, contact inequalities, friction cones and active-set optimization remain future extensions.

The solver validates dimensions, finite values, positive weights/regularization, bound consistency and numerical outputs. The output is transactional: an invalid solve leaves the previous output unchanged.

Default step size is selected from the largest Hessian eigenvalue, giving a conservative fixed-step gradient method for the positive-definite quadratic objective. A caller may provide an explicit positive step size when a tighter problem-specific choice is known.
