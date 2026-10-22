# P7-01 Graph consensus

## Model

Each agent stores one or more state coordinates in a row of a matrix. A nonnegative adjacency matrix specifies how strongly each agent weights its neighbors. One explicit-Euler update is

\[
x_i^+ = x_i + \alpha \sum_j a_{ij}(x_j-x_i), \qquad \alpha = k\Delta t.
\]

The implementation accepts directed weights, although convergence to a common average is only expected under additional graph and weight assumptions. For a convex-combination step it enforces \(\alpha \max_i \sum_j a_{ij}\le 1\). Invalid dimensions, negative/non-finite weights, non-finite states, or an unstable step are rejected without changing the caller's output.

## Validation and benchmark

Tests cover the neighbor-average update, multi-coordinate state, average convergence on a connected undirected chain, invalid inputs, stability-bound rejection and transactional output preservation. The benchmark covers 8, 32 and 128 agents.

## Scope boundary

This is a synchronous, single-step consensus primitive. It does not model communication delay/loss, packet scheduling, asynchronous updates, actuator dynamics, formation geometry or collision avoidance. Those are separate planned modules and must be addressed before deploying a distributed controller.
