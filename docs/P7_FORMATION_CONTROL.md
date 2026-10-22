# P7-02 Relative formation control

## Model

Each robot has a position row \(p_i\) and a desired shape offset row \(r_i\). The offset pattern is translation invariant. For nonnegative adjacency weights, the controller computes

\[
v_i = k \sum_j a_{ij}\big[(p_j-p_i)-(r_j-r_i)\big].
\]

For a connected undirected graph with mutually consistent offsets, the relative position errors are driven toward zero under the ideal continuous-time model. Each agent command is independently saturated to the configured maximum speed.

## Validation

Tests cover relative-error correction direction, per-agent speed saturation, invariance to common translation of all desired offsets, and invalid-input rejection without mutating the output. The benchmark covers 8, 32 and 128 agents.

## Scope boundary

This is a kinematic, velocity-level formation primitive. It does not regulate the formation's global position or orientation, guarantee collision avoidance, model actuator dynamics, or address delay/loss/asynchronous communication. Directed or disconnected graphs do not in general guarantee convergence to the requested shape. Validate graph connectivity and offset consistency for the target deployment.
