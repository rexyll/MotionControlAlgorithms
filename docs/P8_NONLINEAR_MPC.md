# P8-03 Nonlinear MPC baseline

`mca/control/nonlinear_mpc.hpp` provides a dependency-free generic nonlinear MPC baseline based on direct shooting and projected finite-difference gradient descent.

## Formulation

For a discrete nonlinear model

`x[k+1] = f(x[k], u[k])`,

the solver minimizes

`J = sum(k=0..N-1) l(x[k],u[k]) + V(x[N])`

subject to element-wise control bounds `u_min <= u[k] <= u_max`.

The optimizer initializes the complete control sequence at the midpoint of the bounds by default, computing each midpoint as half the lower bound plus half the upper bound to avoid overflow for large-but-finite same-sign bounds, or accepts `NonlinearMpcOptions::initial_control_sequence` as an optional warm start. A warm-start matrix must match control-dimension × horizon and contain only finite values; values are projected into the configured control bounds before rollout. `shift_nonlinear_mpc_warm_start` advances a previous solution by one control step and repeats the final column when extending the horizon, without modifying its output on invalid input. The solver estimates the objective gradient with finite differences. It divides each finite-difference objective value by the finite control interval before subtracting the scaled values, avoiding an intermediate overflow when the raw cost difference exceeds the floating-point range but the resulting gradient is representable. If the quotient itself is not representable, the solver still rejects the non-finite gradient. The solver uses bounded backtracking to accept only steps that strictly reduce the objective. Every candidate is projected into the control bounds; fixed control dimensions (equal lower/upper bounds) are supported with zero gradient for that dimension. If no improving step is found within the configured line-search budget, optimization stops and returns the best accepted sequence. The first control vector is returned for receding-horizon use. `NonlinearMpcResult::iterations_completed` counts outer gradient iterations entered, while `line_search_evaluations` counts candidate trials. `termination` distinguishes exhausting the outer iteration budget, stopping because no candidate strictly improved the objective, and satisfying an optional projected-gradient tolerance. Set `NonlinearMpcOptions::projected_gradient_tolerance` to a finite positive value to enable this stopping rule; zero disables it and preserves the prior iteration behavior. `NonlinearMpcResult::projected_gradient_norm` reports the norm of the bound-projected gradient mapping, scaled by `NonlinearMpcOptions::gradient_step`; this makes the diagnostic invariant to the configured optimization step size for the same local gradient and active-set geometry. This is a practical first-order stopping diagnostic, not a formal KKT certificate for general nonconvex problems.

## Scope and guarantees

- generic state/control dimensions through Eigen dynamic vectors;
- caller-supplied nonlinear dynamics and stage/terminal costs;
- deterministic bounded optimization and line-search iteration counts;
- optional validated warm starts with bound projection and a transactional receding-horizon shift helper;
- non-increasing objective across accepted optimization iterations;
- termination diagnostics (`IterationLimitReached` or `NoImprovingStep`), completed outer iterations, and line-search trial count. `NoImprovingStep` means the configured search budget found no strictly improving candidate; it is not a mathematical convergence certificate;
- support for fixed control dimensions where lower and upper bounds are equal;
- explicit element-wise control bounds;
- finite-value and dimension validation;
- transactional output: an invalid solve leaves the caller's result unchanged;
- no external nonlinear-programming dependency.

This is a reference/baseline solver rather than a real-time SQP or interior-point implementation. Direct shooting with finite-difference gradients scales poorly with horizon and control dimension, and state/path constraints are intentionally outside this baseline. Production MPC should consider analytic/automatic derivatives, warm starts, trust regions, state/path constraint handling, and solver-specific real-time guarantees.

Tests cover nonlinear rollout/convergence, objective non-increase under an aggressive initial step, fixed control bounds, invalid line-search configuration, callback validation, and transactional failure behavior. A deterministic receding-horizon regression repeatedly solves, applies only the first control, shifts the previous sequence, and warm-starts the next cycle while checking bounded controls and closed-loop state reduction. Projected-gradient tests cover first-order stationarity at active upper and lower bounds, a fixed control dimension, verify that a nonstationary interior point does not trigger the stopping rule, and verify that the reported mapping norm is invariant to the configured gradient step. Initialization tests cover representable midpoints of large same-sign finite bounds and preserve the caller's result when the initial rollout fails. A gradient regression uses large finite objective values whose direct subtraction overflows while their finite-difference quotient remains representable. The projected-gradient mapping respects box constraints: at an active bound, an objective gradient pointing out of the feasible interval is stationary for the constrained first-order test; a fixed dimension has no feasible descent direction. The benchmark compares cold-start and receding-horizon warm-start solve cost for several horizons.
