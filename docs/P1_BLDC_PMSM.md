# P1-05 BLDC/PMSM Plant Model

## Scope and assumptions

`mca::plant::BldcPmsm` is a deterministic averaged three-phase motor plant supporting sinusoidal PMSM back-EMF and trapezoidal BLDC back-EMF. It is independent of ROS 2, Gazebo, and hardware SDKs.

The model uses uncoupled phase inductances and omits neutral-point dynamics, saturation, iron losses, cogging torque, inverter switching, dead time, and magnetic saliency. It is a first plant abstraction for validating phase-domain algorithms, not a high-fidelity electrical machine model.

## Equations

For each phase (k\in\{a,b,c\}\):

\[
L_s\dot i_k=v_k-R_s i_k-e_k(\theta_e,\omega_m)
\]

\[
J\dot\omega_m=\tau_e-B\omega_m-\tau_{load},\qquad
\dot\theta_m=\omega_m,\quad \theta_e=p\theta_m
\]

The back-EMF amplitude is proportional to electrical speed (p\omega_m\), where (p) is the pole-pair count. PMSM uses sinusoidal phase shapes; BLDC uses a trapezoidal shape with 120-degree flat regions and 60-degree commutation ramps. Electromagnetic torque is calculated from the phase-current/back-EMF shape projection with a configurable torque constant.

The state uses SI units and semi-implicit Euler integration. For numerical stability, choose a time step small relative to the phase electrical time constant (L_s/R_s).

## Validation

`tests/plant/bldc_pmsm_test.cpp` checks invalid configuration rejection, three-phase current response and direction, pole-pair electrical-angle conversion, and invalid timestep/non-finite input rejection.

## Limitations and next step

This phase-domain model intentionally precedes the Clarke/Park transforms and the dedicated PMSM dq model. It does not claim current-regulated FOC behavior. P1-06 introduces reference-tested coordinate transforms; P1-07 then adds the rotor-frame dq equations.
## Checked power diagnostics

`electrical_power(va, vb, vc, output)` computes instantaneous terminal input power as `va * ia + vb * ib + vc * ic` from the supplied phase voltages and stored phase currents. `mechanical_power(output)` computes shaft power from checked electromagnetic torque and mechanical speed. The status-returning diagnostics preserve the output argument on failure, do not mutate plant state, reject non-finite voltage inputs, and report `NumericalError` for non-finite intermediate products or sums. They do not include inverter losses or switching effects.
`copper_loss(output)` reports the total phase-winding loss `R_s * (i_a^2 + i_b^2 + i_c^2)` with checked intermediate arithmetic. It preserves caller output and runtime state on failure.


Mechanical-power worked point (default sinusoidal PMSM mode and `K_t = 0.05 N·m/A`): choose mechanical angle `π/12 rad`, mechanical speed `10 rad/s`, and phase currents `(1, 0, 0) A`. With four pole pairs, electrical angle is `π/3 rad`, so the phase-A sinusoidal shape is `sin(π/3) = √3/2`; electromagnetic torque is `0.05*(√3/2) ≈ 0.0433013 N·m`, and `mechanical_power(output)` returns approximately `0.433013 W`. Reversing mechanical speed reverses the shaft-power sign without changing the instantaneous torque for this operating point.

Worked point (default phase resistance `R_s = 0.2 Ω`): for stored currents `(i_a, i_b, i_c) = (1, -1, 0) A`, supplied phase voltages `(v_a, v_b, v_c) = (2, -1, 0) V` produce terminal input power `2*1 + (-1)*(-1) + 0*0 = 3 W`; copper loss is `0.2*(1^2 + (-1)^2 + 0^2) = 0.4 W`. Reversing current polarity alone changes the sign of the matching terminal-power contributions but does not change copper loss. These values do not account for inverter losses, switching, or magnetic-energy changes during transients.
Sign convention: `va*ia + vb*ib + vc*ic` is positive when net electrical power enters the three-phase plant and negative when it is returned to the source. Copper loss remains nonnegative for positive phase resistance because it depends on squared phase currents, not current polarity.
