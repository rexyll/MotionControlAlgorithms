# P4-01 Six-degree-of-freedom UAV rigid-body model

## Scope and frames

`mca::uav::RigidBodyModel6D` evaluates the Newton-Euler equations for a rigid vehicle. It returns a state derivative and deliberately does not integrate time; the caller chooses a numerical integrator and simulation step.

- Position and linear velocity are expressed in the world frame.
- The quaternion represents body-to-world attitude, stored as `(w,x,y,z)`.
- Angular velocity, applied force and applied torque are expressed in the body frame.
- Mass is in kg, inertia in kg·m², force in N, torque in N·m, and time in seconds.

## Equations

For mass (m), body inertia (I), body angular velocity (omega), body force (F_b), body torque (	au_b), body-to-world rotation (R), and world gravity (g_w):

[
dot p_w = v_w,qquad dot v_w = R F_b/m + g_w
]

[
dot q_{wb} = 	frac12 q_{wb}otimes(0,omega),qquad
dotomega = I^{-1}left(	au_b-omega	imes(Iomega)ight).
]

The model normalizes the input attitude before rotating forces and computing quaternion rate. It requires positive finite mass and a finite, symmetric positive-definite inertia tensor. It does not model rotor dynamics, aerodynamic drag, wind, ground contact, actuator saturation, or motor allocation.

## API and error behavior

Use `RigidBodyParameters6D`, `RigidBodyState6D`, and `RigidBodyInput6D` with `RigidBodyModel6D::derivative(...)`. Validation or non-finite results return `false` and leave the output derivative unchanged. This is a compact reference model, not a time integrator or a full flight controller.

## Validation and benchmark

Unit tests check hover thrust balancing gravity, body-to-world force rotation, the diagonal-inertia Euler equation, quaternion-rate convention, normalization of non-unit attitudes, transactional rejection of invalid input, and rejection of non-physical inertia. The deterministic micro-benchmark repeatedly evaluates one fixed vehicle state and input.
