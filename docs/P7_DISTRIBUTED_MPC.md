# P7-05 Distributed MPC

## Model and update

Each row of `control_sequences` contains one robot's flattened finite-horizon control plan. The caller supplies the gradient of that robot's local prediction/tracking/effort objective. The graph adjacency is nonnegative, symmetric and has a zero diagonal.

For agent (i), one coordination step applies

[
U_i^+ = operatorname{clip}left(U_i-etaleft(
abla J_i(U_i)+hosum_j a_{ij}(U_i-U_j)ight),-u_{max},u_{max}ight).
]

The consensus term is the graph-Laplacian gradient of a quadratic disagreement penalty. The output contains the updated horizon and its first control vector, which is the command to apply before replanning on the next sample.

## Validation and tests

The function validates agent count, horizon/control dimensions, finite values, symmetric nonnegative adjacency, zero diagonal, step/gain/bound parameters, and finite output. It updates the output transactionally. Tests cover consensus motion, local-gradient updates, hard control bounds, disconnected graphs, invalid shapes, non-finite data and output preservation. Benchmarks vary agent count and horizon.

## Scope boundary

This is a numerical coordination primitive, not a complete distributed MPC runtime or local dynamics solver. Each caller remains responsible for generating model-based objective gradients and for enforcing state, collision, actuator-rate, and safety constraints not represented by the elementwise control bound. A single gradient step does not imply convergence or closed-loop stability; choose step size and consensus gain using the local model and validate the coupled closed loop. No transport, delay handling, asynchronous optimization or fault recovery is implemented here.
