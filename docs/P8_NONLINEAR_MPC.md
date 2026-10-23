# P8-03 Nonlinear MPC baseline

`mca/control/nonlinear_mpc.hpp` provides a dependency-free generic nonlinear MPC baseline based on direct shooting and projected finite-difference gradient descent.

## Formulation

For a discrete nonlinear model

`x[k+1] = f(x[k], u[k])`,

the solver minimizes

`J = sum(k=0..N-1) l(x[k],u[k]) + V(x[N])`

subject to element-wise control bounds `u_min <= u[k] <= u_max`.

The optimizer initializes the complete control sequence at the midpoint of the bounds by default, or accepts `NonlinearMpcOptions::initial_control_sequence` as an optional warm start. A warm-start matrix must match control-dimension × horizon and contain only finite values; values are projected into the configured control bounds before rollout. `shift_nonlinear_mpc_warm_start` advances a previous solution by one control step and repeats the final column when extending the horizon, without modifying its output on invalid input. The solver estimates the objective gradient with finite differences, and uses bounded backtracking to accept only steps that strictly reduce the objective. Every candidate is projected into the control bounds; fixed control dimensions (equal lower/upper bounds) are supported with zero gradient for that dimension. If no improving step is found within the configured line-search budget, optimization stops and returns the best accepted sequence. The first control vector is returned for receding-horizon use.

## Scope and guarantees

- generic state/control dimensions through Eigen dynamic vectors;
- caller-supplied nonlinear dynamics and stage/terminal costs;
- deterministic bounded optimization and line-search iteration counts;
- optional validated warm starts with bound projection and a transactional receding-horizon shift helper;
- non-increasing objective across accepted optimization iterations;
- support for fixed control dimensions where lower and upper bounds are equal;
- explicit element-wise control bounds;
- finite-value and dimension validation;
- transactional output: an invalid solve leaves the caller's result unchanged;
- no external nonlinear-programming dependency.

This is a reference/baseline solver rather than a real-time SQP or interior-point implementation. Direct shooting with finite-difference gradients scales poorly with horizon and control dimension, and state/path constraints are intentionally outside this baseline. Production MPC should consider analytic/automatic derivatives, warm starts, trust regions, state/path constraint handling, and solver-specific real-time guarantees.

Tests cover nonlinear rollout/convergence, objective non-increase under an aggressive initial step, fixed control bounds, invalid line-search configuration, callback validation, and transactional failure behavior. The benchmark compares cold-start and receding-horizon warm-start solve cost for several horizons.
