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
