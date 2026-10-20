# P1-14 Scalar Trajectory Generators

## Scope

P1-14 provides lightweight, deterministic scalar reference trajectories for single-axis motion-control experiments. The implementation is header-only and has no ROS 2, Gazebo, or dynamic-allocation dependency.

## API

`mca::trajectory::ScalarSample` contains:

- `position`
- `velocity`
- `acceleration`

`ISampledScalarTrajectory` defines the common `sample(time_s)` interface.

### Step

`StepTrajectory(target, initial)` returns `initial` for negative time and `target` at and after zero. Velocity and acceleration are zero because this is an ideal reference command, not a physically smoothed motion profile.

### Ramp

`RampTrajectory(start, finish, duration)` is linear during `(0, duration)` and clamps to the start/finish states outside that interval. The in-ramp velocity is `(finish-start)/duration`; acceleration is zero.

For a non-positive duration, the implementation returns the finish state. This keeps the sample operation deterministic and avoids division by zero.

### Sine

`SineTrajectory(amplitude, frequency_hz, offset, phase_rad)` evaluates

`q = offset + amplitude * sin(2*pi*f*t + phase)`

`dq = amplitude * 2*pi*f * cos(2*pi*f*t + phase)`

`ddq = -amplitude * (2*pi*f)^2 * sin(2*pi*f*t + phase)`.

The analytical derivatives are returned together with position, making the generator suitable as a reference for position, velocity, and acceleration controllers.

## Boundary conventions

- Step switches at `t = 0`.
- Ramp returns its start state for `t <= 0` and finish state for `t >= duration`.
- Sine is continuous for all finite time values.
- The current API is a value-oriented reference generator; parameter validation is intentionally lightweight and callers should provide finite physical parameters.

## Acceptance

P1-14 is accepted only when the implementation, analytical unit tests, benchmark target, documentation, and Linux/Windows CI are all present and passing. Tests cover endpoint behavior, midpoint slope, analytical sine derivatives, and phase/offset behavior.
