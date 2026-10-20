# P1-01 First/Second-Order Plant Models

## Scope

P1-01 provides deterministic continuous-time plant models for single-axis controller experiments. The models are deliberately independent of ROS 2 and Gazebo and use fixed-size state only.

## First-order plant

`mca::plant::FirstOrderPlant` represents

`tau * y_dot + y = gain * u`

with exact zero-order-hold discretization for each `step(input, dt)` call:

`y[k+1] = exp(-dt/tau) y[k] + gain (1-exp(-dt/tau)) u[k]`.

This avoids an integration-step approximation for the first-order state and is useful as a reference plant for controller tests.

## Second-order plant

`mca::plant::SecondOrderPlant` represents

`J*qdd + B*qd + K*q = effort`.

The state is position and velocity. Acceleration is evaluated from the current state and effort, then semi-implicit Euler updates velocity followed by position.

## Validation and behavior

Both models reject non-finite parameters and invalid positive time constants/inertia. Runtime `step()` calls reject non-finite inputs and non-positive/non-finite time steps. `reset()` validates supplied state.

## Acceptance tests

The test suite verifies invalid configuration rejection, first-order convergence, the exact one-time-constant response, second-order static equilibrium, and free acceleration from the inertia equation. The benchmark target exercises repeated stepping for both models when Google Benchmark is available.

## Design boundary

These models are simulation/reference components only. They do not contain ROS 2, Gazebo, hardware I/O, timing threads, or dynamic allocation. Higher-level experiments can compose them with controllers and trajectories without changing the algorithm core.
