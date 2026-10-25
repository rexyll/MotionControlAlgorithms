# P1-07 PMSM dq Model

## Scope

P1-07 adds a rotor-frame sinusoidal PMSM plant model using exactly the Clarke/Park sign convention fixed in P1-06. The model is intended for current-control and servo experiments and remains independent of ROS 2/Gazebo.

## Electrical equations

Let `theta_e = p * theta_m` and `omega_e = p * omega_m`.

```text
vd = R_s i_d + L_d di_d/dt - omega_e L_q i_q
vq = R_s i_q + L_q di_q/dt + omega_e (L_d i_d + psi_f)
```

The implementation solves these equations for the current derivatives:

```text
di_d/dt = (vd - R_s i_d + omega_e L_q i_q) / L_d

di_q/dt = (vq - R_s i_q - omega_e (L_d i_d + psi_f)) / L_q
```

`psi_f` is permanent-magnet flux linkage in Wb. `L_d` and `L_q` are independent so both surface-mounted and salient PMSM behavior can be represented.

## Electromagnetic torque

```text
T_e = 3/2 * p * [psi_f i_q + (L_d - L_q) i_d i_q]
```

The first term is permanent-magnet torque. The second term is reluctance/saliency torque.

For a surface-mounted PMSM with `L_d = L_q`, the torque reduces to

```text
T_e = 3/2 * p * psi_f * i_q
```

## Mechanical equations

```text
J d(omega_m)/dt = T_e - B omega_m - T_load
d(theta_m)/dt = omega_m
```

The electrical angle and speed exposed by the model are

```text
theta_e = p theta_m
omega_e = p omega_m
```

## Numerical integration

The model uses semi-implicit Euler: electrical currents are updated first, then electromagnetic torque is evaluated from the updated current state, and mechanical velocity/position are advanced. This matches the integration style of the existing DC and phase-domain motor models.

## Validation

`tests/plant/pmsm_dq_test.cpp` checks:

- invalid configuration and timestep handling;
- zero-speed d/q electrical equations;
- signs and magnitudes of cross-coupling and permanent-magnet back-EMF terms;
- surface-PMSM torque equation;
- saliency torque contribution;
- mechanical acceleration from electromagnetic torque;
- electrical angle/speed scaling by pole pairs.

These are analytical/reference tests rather than controller-tuning tests. The model is accepted only after the common Linux and Windows CI configure/build/test pipeline passes.

## Boundary with later stages

P1-07 is the plant. It does not contain PI current controllers, voltage decoupling/feedforward, current limiting, PWM generation, inverter dead time, saturation, magnetic nonlinearities, or switching behavior. P1-08 will add FOC current control, while P1-09 will add explicit decoupling/feedforward terms around the same electrical equations.
## Checked power diagnostics

`electrical_power(vd, vq, output)` computes dq-frame electrical input power using the model's amplitude-invariant transform convention: `P_e = 3/2 * (vd * i_d + vq * i_q)`. `mechanical_power(output)` computes shaft power from electromagnetic torque and mechanical speed. Both are status-returning, diagnostic-only APIs: they preserve caller output on failure, never mutate runtime state, and report invalid inputs or non-finite intermediate/final arithmetic explicitly. These equations do not include inverter losses or switching effects.
`copper_loss(output)` reports stator winding loss using the amplitude-invariant dq convention: `P_cu = 3/2 * R_s * (i_d^2 + i_q^2)`. Intermediate squares, sum, resistance product, and final scaling are checked; output and runtime state remain unchanged on failure.


Mechanical-power worked point (default surface-mounted PMSM parameters): with `i_d = 0 A`, `i_q = 2 A`, four pole pairs, permanent-magnet flux `ψ_f = 0.05 Wb`, and mechanical speed `ω = 10 rad/s`, the torque equation gives `T_e = 3/2 * p * ψ_f * i_q = 0.6 N·m`; `mechanical_power(output)` therefore returns `6 W`. Reversing mechanical speed while holding the currents fixed gives `-6 W`. This is shaft power from the model's electromagnetic torque and speed, not a prediction that electrical input power must equal shaft power instantaneously.

Worked point (default `R_s = 0.5 Ω`): for stored currents `i_d = 1 A`, `i_q = -2 A` and supplied voltages `v_d = 2 V`, `v_q = 1 V`, terminal input power is `3/2 * (2*1 + 1*(-2)) = 0 W`; copper loss is `3/2 * 0.5 * (1^2 + (-2)^2) = 3.75 W`. This illustrates that the instantaneous electrical input diagnostic and copper-loss diagnostic are separate calculations; they must not be interpreted as a complete energy balance during a transient, where magnetic stored energy and other losses may matter.
Sign convention: with the amplitude-invariant dq convention used by this model, `3/2 * (vd*id + vq*iq)` is positive for net electrical power entering the stator and negative for net power returned to the source. Copper loss is nonnegative for positive resistance and depends on squared dq currents.
