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
