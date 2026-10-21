# P3-07 Singularity Metrics

`mca::kinematics::SingularityMetrics` evaluates singular values and numerical rank from a geometric Jacobian. It is dependency-free and uses a bounded Jacobi eigensolver on the smaller Gram matrix (`JᵀJ` or `JJᵀ`), whose dimension is at most six.

## Output conventions

- Singular values are sorted descending.
- Numerical rank uses `max(absolute_rank_tolerance, relative_rank_tolerance * maximum_singular_value)`.
- `singular` is true when the numerical rank is below `min(6, joint_count)`.
- Condition number is maximum/minimum singular value for full numerical rank, otherwise infinity.
- Manipulability is the product of the singular values returned for the smaller Gram matrix; it is zero for a rank-deficient Jacobian.
- The smallest singular value is a continuous indicator of proximity to rank loss; its scale depends on the mixed translational/angular units of the geometric Jacobian.

## Verification

Tests cover known diagonal singular values, exact rank deficiency, a six-row/seven-column Jacobian, option/input validation and transactional output behavior. Linux and Windows CI validation pending.
