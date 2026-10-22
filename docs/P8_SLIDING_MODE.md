# P8-07 — Sliding-mode control

## Scope

This module provides a deterministic scalar boundary-layer sliding-mode control law. It is intended as a reusable control primitive; it is not a complete plant-specific controller or a closed-loop stability guarantee.

## Control law

Given a caller-defined sliding variable \(s\) and equivalent/nominal control \(u_{eq}\), the controller computes

\[
u = \operatorname{clip}\left(u_{eq} - k\,\operatorname{sat}(s/\phi), u_{min}, u_{max}\right)
\]

where \(k \ge 0\) is the switching gain and \(\phi > 0\) is the boundary-layer thickness. Inside the boundary layer, the switching term varies linearly with \(s\); outside it, the term is bounded by \(k\). This smooth approximation reduces ideal sign-function chattering at the cost of a residual boundary layer.

## API

- Include `mca/control/sliding_mode.hpp` or the umbrella `mca/mca.hpp`.
- Configure `SlidingModeConfig` with a nonnegative switching gain, positive boundary layer, and ordered finite output limits.
- Call `update(sliding_variable, nominal_control, output)` each control cycle.
- Check `core::Status`; invalid inputs do not overwrite the supplied output. The controller itself is stateless between updates and performs no dynamic allocation.

## Engineering limitations

The caller must derive the sliding surface and nominal/equivalent control for the plant, select gains with model uncertainty and actuator limits in mind, and validate stability and performance for the actual sampled system. The output clamp is an actuator guard, not proof of reaching or invariance. Sensor noise, sampling, unmodeled dynamics, and actuator delay can materially affect behavior; test in simulation before hardware use.
