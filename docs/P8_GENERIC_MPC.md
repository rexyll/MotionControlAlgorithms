# P8-05 Generic MPC Interface

## Purpose

`mca/control/generic_mpc.hpp` defines the common contract shared by MPC implementations without forcing a particular optimization algorithm. Linear MPC, nonlinear MPC, QP-based MPC and future constrained solvers can be connected through the same problem/result boundary.

## Problem contract

`GenericMpcProblem` contains:

- `initial_state`: finite, non-empty state vector.
- `control_lower` / `control_upper`: finite per-input bounds with matching dimensions.
- `horizon`: finite positive horizon, currently limited to 200 samples.

`GenericMpcOptions::horizon` must match the problem horizon. The interface deliberately keeps dynamics and cost functions out of the generic contract; each concrete solver owns its model representation.

## Solver contract

`MpcSolverFunction` receives a validated problem and options and fills `GenericMpcResult`.

The result must contain:

- first control vector with dimension `nu`;
- full control sequence with shape `nu x N`;
- predicted states with shape `nx x (N+1)`;
- finite scalar objective value.

Both the first control and every control in the sequence are checked against the configured bounds before the result is committed to the caller.

## Transactional behavior

Validation and solver failures leave the caller's existing result unchanged. A successful call commits the candidate result only after all dimensional, finite-value and bound checks pass.

This makes the interface suitable for a receding-horizon loop: a concrete solver can be replaced without changing the caller's validation and result-handling code.

## Scope

This is an integration contract, not an optimization algorithm. It intentionally does not prescribe Riccati recursion, direct shooting, SQP, ADMM, active-set QP, warm starting, or real-time memory behavior. Those belong to concrete solver implementations.
