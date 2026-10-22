# P8-08 — Iterative learning control

## Purpose

This module provides a sample-indexed feedforward update for tasks that repeat with consistent timing and trajectory alignment. It is useful as a building block for repeated robot motions, but it does not replace feedback control.

## Update law

For trial k and sample i,

\[
u_{k+1}[i] = \operatorname{clip}(\rho u_k[i] + L e_k[i], u_{\min}, u_{\max})
\]

where \(e_k[i]\) is the measured tracking error, \(L\ge0\) is the learning gain, and \(0\le\rho\le1\) retains or forgets the previous feedforward sequence. The implementation uses a fixed configured sample count and componentwise output bounds.

## API and operational assumptions

- Configure the sample count, learning gain, retention factor, and output limits.
- Optionally seed a previous feedforward sequence with `reset`; values are projected into the configured limits.
- At the end of each trial, pass the aligned error vector to `update`. The returned sequence becomes the stored sequence for the next trial.
- Invalid dimensions and non-finite inputs are rejected without changing stored state or the caller's output.
- The caller must preserve sample timing and indexing between trials, define error sign consistently, and combine the learned feedforward with a stabilizing feedback controller.

## Limitations and safety

This is a basic componentwise learning law, not a convergence proof. Excessive learning gain, changing trajectories, delays, noise, non-repeatable disturbances, or actuator saturation can worsen tracking. Validate convergence on a simulated plant and use conservative limits before hardware deployment.
