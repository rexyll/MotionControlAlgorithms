# P8-01 Linearization framework

## Contract

The linearize_discrete_dynamics function builds a local affine approximation of a discrete-time nonlinear system x[k+1] = f(x[k], u[k]) at an operating point (x0, u0).

- A = df/dx and B = df/du are estimated by central finite differences.
- affine_offset = f(x0,u0) - A*x0 - B*u0 makes the local affine model match the nonlinear evaluator at the operating point.
- The result retains the operating state, input and next state for verification and downstream use.

The evaluator is supplied by the caller, keeping the framework independent of a specific robot model, integrator, solver or control law.

## Numerical and failure behavior

The finite-difference step must be finite and positive. Per-coordinate perturbations scale by max(1, abs(coordinate)). Evaluator failure, wrong next-state dimension, non-finite input/output or invalid options return false; output is assigned only after all evaluations and matrix checks pass. The evaluator must be deterministic and side-effect-free during one linearization.

## Validation

Tests recover known A/B/affine terms for a linear system, compare central differences against analytic derivatives of a nonlinear system, and cover invalid evaluator, dimension mismatch, non-finite dynamics output, invalid step and transactional output preservation. A benchmark sweeps state/input dimensions.

## Scope boundary

This is a generic local finite-difference linearizer for discrete-time dynamics. It does not guarantee global accuracy, differentiability or stability; it does not provide automatic differentiation, continuous-time discretization, Jacobian sparsity exploitation or operating-point selection. Choose perturbation sizes appropriate to the model units and numerical scale.
