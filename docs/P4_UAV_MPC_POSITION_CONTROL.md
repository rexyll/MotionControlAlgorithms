# P4-06 UAV translational MPC position control

## Model

`mca::uav::MpcPositionController` applies a receding-horizon optimizer independently to each world-frame translational axis. The prediction model is a sampled double integrator:

- `p[k+1] = p[k] + dt*v[k] + 0.5*dt^2*u[k]`
- `v[k+1] = v[k] + dt*u[k]`

The optimization variable `u` is an acceleration correction relative to the desired position/velocity trajectory. The objective penalizes predicted position error, velocity error, acceleration correction, and terminal state error. Projected gradient descent enforces a bounded correction; the first optimized correction is combined with feedforward acceleration and clamped to the configured per-axis acceleration limits.

## Runtime and validation

The horizon is fixed-capacity and bounded to 100 steps; the update path uses stack-backed arrays and does not allocate. Horizon length, iteration count, positive finite weights/limits, sample time, and gradient step are validated at configuration. Non-finite input is rejected transactionally. `predicted_cost` reports the optimized predicted state/control cost before feedforward is applied.

## Scope and limitations

This is a decoupled translational baseline, not a full nonlinear flight MPC. It does not model attitude coupling, gravity, aerodynamic drag, obstacle constraints, thrust/tilt limits, actuator lag, or control-rate constraints. Feedforward and acceleration saturation must be consistent with the downstream attitude/thrust controller. The projected-gradient solver has a bounded iteration budget but is not a general proof of global optimality; tune weights and gradient step for the intended sample time and horizon.

## Verification

Tests cover invalid configuration and input, feedforward at zero error, correction direction, independent acceleration limits, and a receding-horizon double-integrator convergence scenario. The benchmark measures the fixed-capacity update path.
