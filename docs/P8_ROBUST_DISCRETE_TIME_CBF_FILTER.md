# P8-15 — Robust Discrete-Time Multi-Input CBF Filter

## Robust one-step condition

This wrapper accounts for a known bound on additive error in each one-step barrier prediction. If the nominal prediction is

\[
h_i[k+1;u] = d_i + G_i u + e_i, \qquad |e_i| \leq \delta_i,
\]

the worst-case lower bound is \(d_i + G_i u - \delta_i\). The filter enforces

\[
d_i + G_i u - \delta_i \geq (1-\gamma)h_i[k] + s.
\]

It delegates the resulting affine constraints, actuator bounds and nominal-command projection to `DiscreteTimeCbfFilter`. Error bounds must be finite, nonnegative, and supplied for every barrier.

## API and failure behavior

- `RobustDiscreteTimeCbfInput` combines a standard discrete-time prediction with per-barrier absolute error bounds.
- `RobustDiscreteTimeCbfFilter` uses the same `DiscreteTimeCbfConfig` and `DiscreteTimeCbfOutput` as the nominal filter.
- Invalid error bounds and dimensions are rejected; infeasible robust constraints return `ConstraintViolation`.
- The caller's output is updated only after a feasible robust control is found.

## Limitations

The bounds must conservatively cover actual additive prediction error over the control interval. This implementation does not estimate uncertainty or prove that a chosen bound is valid. It assumes affine control dependence and uses the small-problem active-set enumeration of the underlying filter; worst-case latency grows combinatorially with the constraint count.
