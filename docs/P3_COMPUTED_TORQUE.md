# P3-12 Computed torque

## Scope and control law

`ComputedTorque` provides a stateless joint-space model-based control law for a fixed-base serial manipulator:

`tau = M(q) [ddq_des + Kp (q_des - q) + Kd (dq_des - dq)] + c(q,dq) + G(q)`

It reuses P3-10 rigid-body dynamics and applies non-negative diagonal PD gains plus optional symmetric per-joint torque limits. Inputs are desired position, velocity and acceleration; trajectory generation is outside this component.

## Validation and limitations

Tests check the analytical one-link acceleration-plus-gravity result, PD feedback with torque saturation, and transactional invalid-gain handling. The reference dynamics implementation computes Coriolis terms by finite differences and is not intended for high-DOF hard real-time use. Torque limits here are a software clamp, not a substitute for drive-level protections. Verify model parameters, units, gravity direction and feedback signs on a safely supported robot before enabling actuation.

## Cross-platform CI acceptance

Run #348 passed on both Linux and Windows with Configure, Build and Test successful for commit `f0b0835203b24d09d4ff4fed1545f982022a93bf`.

CI: https://github.com/rexyll/MotionControlAlgorithms/actions/runs/39588631153
