# P1-12 Disturbance Observer

## Scope

P1-12 provides a platform-independent mechanical disturbance observer for a single-axis servo. It estimates an external torque disturbance from the model residual and provides an optional compensated torque output. The core implementation has no ROS2 or Gazebo dependency.

## Model

For the mechanical model

`J * alpha = tau_command - B * omega + tau_disturbance`

the disturbance residual is

`tau_disturbance = J * alpha - tau_command + B * omega`.

Acceleration is estimated from the sampled velocity difference. The residual is passed through a first-order low-pass filter:

`d_hat[k] = d_hat[k-1] + alpha_f * (d_residual[k] - d_hat[k-1])`

with

`alpha_f = dt / (T_filter + dt)`.

The compensation output is

`tau_compensated = tau_command - d_hat`.

## Engineering behavior

- positive estimated disturbance is opposed by subtracting it from the commanded torque;
- configurable inertia and viscous damping are part of the observer model;
- disturbance estimate limits prevent unbounded output;
- invalid/non-finite configuration and runtime inputs are rejected;
- reset can initialize both the estimate and previous velocity;
- runtime execution performs no dynamic allocation;
- the observer is deliberately a low-order mechanical observer, not a high-bandwidth sensor fusion or nonlinear observer.

## Validation

Tests cover invalid configuration/input, analytical residual estimation, compensation sign, estimate limits and reset behavior. A Google Benchmark target measures the runtime update path.

The observer should be tuned against the expected velocity measurement noise and sampling period: reducing the filter time constant improves disturbance response but increases sensitivity to acceleration-estimation noise.
