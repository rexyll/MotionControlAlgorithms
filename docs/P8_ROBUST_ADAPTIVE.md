# P8-06 Robust/adaptive control

## Scope

This phase adds a dependency-light scalar-control compensation primitive. It is a
baseline building block, not a complete robot-specific adaptive control proof or
a replacement for torque/current safety limits in the actuator layer.

## Control law

For regressor vector \(\phi\), parameter estimate \(\hat\theta\), sliding/error
signal \(s\), and nominal command \(u_0\):

- Adaptive compensation: \(u_a = \hat\theta^T\phi\).
- Continuous robust compensation: \(u_r=-k_r\operatorname{clip}(s/\delta,-1,1)\).
- Command: \(u=\operatorname{clip}(u_0+u_a+u_r,-u_{max},u_{max})\).
- Projected sigma-modified adaptation:
  \(\dot{\hat\theta}=\Gamma s\phi-\sigma\hat\theta\), followed by componentwise
  projection to \([-\theta_{max},\theta_{max}]\).

The boundary layer \(\delta>0\) softens switching around zero. Projection bounds
parameter drift; sigma leakage damps estimates when excitation is weak.

## API and validation

Header: `mca/control/robust_adaptive.hpp`; class: `RobustAdaptiveController`.
Configure requires finite positive adaptation gain, boundary layer, control limit,
and parameter limit; leakage and robust gains must be finite and nonnegative.
Update rejects dimension mismatch, non-finite input, and non-positive time steps.
The state and output are committed only after a candidate update is validated.

## Limitations and deployment notes

The caller must choose a regressor and error signal consistent with its plant and
derive gains for that system. The primitive does not guarantee closed-loop stability
for arbitrary regressors, delays, unmodeled dynamics, or sampling rates. Validate
against a plant model and hardware limits before real-robot deployment.
