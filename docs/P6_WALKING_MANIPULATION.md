# P6-06 Manipulation while walking

## Purpose

Coordinates a locomotion/stability task with an end-effector manipulation task in a single bounded generalized-command solve. The caller supplies both task Jacobians and target velocities (or accelerations, provided the chosen generalized variables and Jacobians use that convention).

The objective is a weighted least-squares compromise:
```text
min_x 0.5*w_l*||J_l*x - v_l||^2
     + 0.5*w_m*||J_m*x - v_m||^2
     + 0.5*lambda*||x - x_ref||^2
subject to lower_bounds <= x <= upper_bounds
```

The locomotion weight defaults higher than the manipulation weight, allowing base balance/tracking to dominate when the two tasks conflict. Weights are soft priorities, not strict lexicographic guarantees. Per-variable bounds can represent joint velocity/acceleration limits. The result reports the generalized command and each task's residual norm so a higher-level supervisor can detect tracking compromise.

## Validation and benchmark

Tests cover a conflicting locomotion/manipulation target, task-weight trade-off, generalized-command bounds, invalid dimensions, and non-finite input with transactional output preservation. The benchmark exercises 12-, 24- and 36-variable cases.

## Scope boundary

This module is a task-level coordination baseline. It does not generate contact schedules, infer foot contact, enforce friction cones, or solve full floating-base inverse dynamics. It must be combined with a validated balance/contact controller and robot-specific limits before hardware use.
