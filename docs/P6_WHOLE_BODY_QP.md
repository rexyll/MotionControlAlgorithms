# P6-05 Whole-body QP baseline

The API minimizes a weighted least-squares objective over generalized variables:
```text
0.5 * sum(weight_i * ||J_i x - target_i||^2)
+ 0.5 * regularization * ||x - reference||^2
subject to lower_bounds <= x <= upper_bounds
```
Tasks supply a Jacobian, target and positive weight. Optional variable bounds are enforced by the existing deterministic projected-gradient QP solver; the optional reference regularizes underdetermined directions. Results include task residual norms, objective and iteration count. Invalid dimensions, non-finite values, invalid weights, and inconsistent bounds are rejected without changing output. Benchmark cases use 12, 24 and 36 variables.

This is a box-constrained weighted-QP baseline, not a complete humanoid whole-body optimizer. Floating-base dynamics equalities, general contact inequalities, friction cones and complementarity are not yet represented. Do not treat it as hardware-ready.
