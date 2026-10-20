# P1-13 State Observers

## Scope

P1-13 provides a reusable, ROS2/Gazebo-independent state observer for a single second-order mechanical axis. It estimates velocity from measured position while using a known torque input and a nominal mechanical model.

## Model

The nominal plant is

```text
J*qdd + B*qd + K*q = tau
```

with state `x = [q, qd]` and measured output `y = q`.

The continuous-time Luenberger observer is

```text
qhat_dot  = vhat + L1*(q - qhat)
vhat_dot  = (tau - B*vhat - K*qhat)/J + L2*(q - qhat)
```

The implementation integrates these equations with forward Euler. `L1` has units `1/s`; `L2` has units `1/s^2`.

For a desired observer error polynomial

```text
s^2 + 2*zeta*omega_n*s + omega_n^2
```

the corresponding gains are

```text
L1 = 2*zeta*omega_n
L2 = omega_n^2
```

when the nominal plant is represented in the same position/velocity coordinates.

## API

`mca::control::StateObserverConfig` contains:

- inertia `J` (> 0)
- viscous damping `B` (>= 0)
- stiffness `K` (>= 0)
- position correction gain `L1` (>= 0)
- velocity correction gain `L2` (>= 0)

`StateObserver::update(torque, measured_position, dt, output)` updates the estimate and returns position, velocity and corrected model acceleration.

`reset(position, velocity)` initializes the observer state without dynamic allocation.

## Validation and limitations

- Non-finite configuration and input values are rejected.
- Positive finite inertia and filter-independent mechanical parameters are required.
- Negative observer gains are rejected; pole-placement tuning is the caller's responsibility.
- The observer assumes position measurements and a reasonably accurate nominal mechanical model.
- Forward Euler requires a sufficiently small control/sample period relative to the observer bandwidth.
- Measurement noise is not explicitly filtered; excessive observer bandwidth will amplify noisy position measurements through the correction terms.
- This is a model-based Luenberger observer, not a Kalman filter or a nonlinear observer.

## Tuning guidance

1. Select an observer bandwidth above the mechanical closed-loop bandwidth so velocity converges faster than the controlled plant.
2. Do not make the bandwidth unnecessarily high when the position sensor is noisy.
3. A damping ratio around `0.7` is a practical starting point; compute `L1/L2` from the desired natural frequency or tune the gains directly.
4. Verify convergence using the same sample period and quantization/noise characteristics expected on hardware.
5. If model uncertainty or strong disturbance rejection is the dominant problem, combine the state estimate with the P1-12 disturbance observer rather than forcing excessive observer gain.

## Acceptance coverage

The P1-13 tests cover invalid configuration/input handling, pre-configuration behavior, velocity estimation from a simulated second-order plant, convergence from a large initial estimation error, reset behavior, and model-term acceleration calculation. A Google Benchmark target measures the per-update execution path.
