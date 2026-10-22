# P7-04 Distributed optimization

## Model

This module provides one gradient-tracking iteration for agents optimizing local objectives over a shared vector. With row-wise agent estimates \(X\), local gradients \(G\), tracker state \(Y\), doubly-stochastic mixing matrix \(W\), and positive step size \(\alpha\):

\[
X^+ = WX - \alpha Y,
\qquad
Y^+ = WY + G(X^+) - G(X).
\]

The caller computes each agent's local gradient at the newly produced estimates and supplies it through `gradients_at_next_estimates` for the update. The module validates matrix dimensions, finite values, nonnegative mixing weights, and row/column stochasticity. Output updates are transactional.

## Tests and benchmark

Unit tests cover the update equations, multi-coordinate estimates, rejection of a non-doubly-stochastic mixing matrix, invalid input handling and output preservation. The benchmark covers 8, 32 and 128 agents.

## Scope boundary

This is a numerical algorithm step, not a distributed runtime. It assumes all agents use compatible objective-gradient coordinates and a valid doubly-stochastic mixing matrix. It does not implement transport, message freshness, asynchronous updates, fault tolerance, or automatic step-size selection. Convergence guarantees depend on graph/mixing assumptions, objective regularity and step size; validate those conditions for the application.
