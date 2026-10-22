# P8-11 — Multi-Constraint Scalar CBF Filter

## Purpose

This filter extends the scalar CBF safety layer to multiple simultaneous first-order barrier inequalities while retaining a single scalar actuator. It projects the nominal command onto the intersection of all safe command intervals and configured actuator bounds.

For each barrier h_i, it enforces

\[
L_f h_i + L_g h_i u + \alpha h_i \ge 0,\quad \alpha > 0.
\]

Each inequality gives a lower bound, an upper bound, or a control-independent feasibility check. The feasible set is their intersection with [u_min, u_max]. Since the input is scalar, the projection is an interval intersection rather than a general QP.

## API and behavior

- MultiBarrierFilterConfig: positive alpha, actuator limits, and coefficient tolerance.
- BarrierConstraint: barrier value and the drift/control derivatives for one barrier.
- MultiBarrierFilterInput: non-empty vector of constraints and nominal command.
- MultiBarrierFilterOutput: filtered command, minimum achieved barrier margin, number of near-active constraints, and modification flag.
- Returns ConstraintViolation if any barrier is infeasible or the intervals have an empty intersection. Invalid input and failure leave the output unchanged.

## Scope and limitations

The implementation supports multiple barriers for one scalar control input; it is not a multi-input QP solver. Barrier derivatives must be supplied by the caller. It does not by itself guarantee safety under model error, sampling delay, state-estimation error, or actuator faults. The complete closed loop must be validated for the target robot and operating envelope.
