# P4-04 Geometric attitude control

## Overview

mca::uav::GeometricAttitudeController implements quaternion-based attitude PD feedback without converting orientation to Euler angles. It computes a shortest-arc relative quaternion error in the current body frame, applies proportional attitude feedback and angular-rate damping, adds feedforward body torque and diagonal-inertia gyroscopic compensation, then clamps each torque axis independently.

## Frames and control law

The quaternions map body coordinates to world coordinates and use (w,x,y,z) ordering. The relative error is q_e = conjugate(q_current) * q_desired. If q_e.w is negative, all four components are negated so q and -q produce the same physical command along the shorter quaternion arc.

The attitude error vector is 2 times the vector part of this sign-corrected quaternion. For small errors it approximates the rotation vector. The commanded torque before saturation is

tau = K_R e_R + K_omega (omega_des - omega) + omega × (J omega) + tau_ff,

where J is the configured diagonal body inertia, omega is measured body angular velocity, and tau_ff is the optional body-frame feedforward torque. Gains and inertia are axis-wise values; the controller does not model off-diagonal inertia terms or desired angular-acceleration feedforward.

## Validation and limitations

Configuration requires finite positive diagonal inertia, finite non-negative attitude/rate gains, and finite non-negative per-axis torque limits. Updates reject non-finite state, non-finite feedforward torque, and zero-norm quaternions. Failed updates leave the output unchanged. The class performs no dynamic allocation in the update path.

This is a reusable attitude-control primitive, not a complete flight controller. State estimation, desired trajectory generation, actuator mixing, thrust constraints, motor dynamics and closed-loop stability tuning remain system-level responsibilities. Quaternion feedback avoids Euler-angle coordinate singularities, but physical behavior still depends on gains, inertia accuracy, sampling and actuator limits.

## Tests and benchmark

Tests cover identity tracking, attitude error direction and scale, antipodal quaternion equivalence, rate feedback, feedforward torque, gyroscopic compensation, per-axis saturation, invalid input transactionality and configuration validation. The benchmark measures the controller update path with a deterministic state/setpoint.
