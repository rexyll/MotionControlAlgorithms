# P1-08 FOC current controller

## Scope

`mca::control::FocCurrentController` implements the inner current-control layer used by field-oriented control of a PMSM/BLDC drive. It is intentionally independent of ROS 2, Gazebo and any inverter hardware API.

Signal path:

`i_d_ref, i_q_ref -> PI_d/PI_q -> v_d, v_q -> PMSM dq plant`

The controller uses the Clarke/Park convention documented in `docs/P1_CLARKE_PARK.md` and is intended to drive the `mca::plant::PmsmDq` model from P1-07.

## Discrete PI equations

For each axis:

`e[k] = i_ref[k] - i[k]`

`I_candidate[k] = clamp(I[k-1] + Ki * e[k] * dt, I_min, I_max)`

`v_unsat[k] = Kp * e[k] + I_candidate[k]`

`v[k] = clamp(v_unsat[k], v_min, v_max)`

When the output is saturated and the error would push it farther into saturation, the candidate integral is rejected. This conditional-integration rule prevents integral windup while allowing integration immediately after the controller leaves saturation.

The same logic is applied independently to d and q axes.

## Current and voltage limits

Both current references are clamped to the configured `[current_min, current_max]` range before the PI calculation. d and q voltage outputs have independent configurable limits, which permits asymmetric limits when a future inverter abstraction requires them.

## Numerical and real-time properties

- C++17 header-only controller.
- No dynamic allocation in the runtime update path.
- Explicit finite-value and positive-`dt` validation.
- Deterministic discrete-time behavior.
- No ROS 2/Gazebo dependency.

## Validation

Unit tests cover invalid configuration/dt, discrete PI reference values, current-reference limiting, voltage saturation, anti-windup and recovery after leaving saturation.

P1-08 acceptance additionally requires a closed-loop PMSM dq current-tracking experiment/benchmark and Linux + Windows CI validation before the task is marked complete.

## Boundaries

This controller does **not** implement electrical decoupling/feedforward. Cross-coupling compensation and back-EMF feedforward are intentionally deferred to P1-09. SVPWM is deferred to P1-10.
