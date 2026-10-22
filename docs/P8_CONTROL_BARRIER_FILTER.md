# P8-09 — Control Barrier Function Safety Filter

## Purpose

The scalar control barrier function (CBF) filter is a safety layer placed between a nominal controller and a single actuator. It minimally modifies the nominal command to satisfy a first-order barrier inequality while respecting actuator limits.

For a control-affine system and barrier function, assume

\[
\dot h(x,u) = L_f h(x) + L_g h(x)u.
\]

The filter enforces the zeroing-CBF condition

\[
L_f h(x) + L_g h(x)u + \alpha h(x) \ge 0, \quad \alpha > 0,
\]

together with \(u_{min} \le u \le u_{max}\). In the scalar-input case, these constraints form an interval; the solution is the projection of the nominal command onto that feasible interval.

## API

- `ControlBarrierFilterConfig`: positive `alpha`, actuator bounds, and numerical coefficient tolerance.
- `ControlBarrierFilterInput`: barrier value, drift derivative \(L_f h\), control derivative \(L_g h\), and nominal command.
- `ControlBarrierFilterOutput`: filtered command, achieved barrier margin, and whether the command changed.
- `filter()` returns `ConstraintViolation` when the safety inequality cannot be met within the actuator bounds. On failure, the output is unchanged.

## Limitations and integration

This implementation handles one scalar control input and one first-order barrier inequality. It does not construct barrier functions, estimate model derivatives, resolve multiple simultaneous barriers, or guarantee safety under unmodelled dynamics, sampling delay, state-estimation error, or actuator faults. Integrate it as a safety filter after a nominal controller and validate the complete closed loop for the target robot and operating envelope.
