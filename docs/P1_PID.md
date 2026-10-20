# P1-02 PID / PI / PD

## Scope

`mca::control::PidController` provides a fixed-size discrete PID controller that also represents PI and PD controllers by setting unused gains to zero. The implementation is independent of ROS 2 and Gazebo and performs no dynamic allocation in the runtime update path.

## Control law

For setpoint `r`, measurement `y`, and sample time `dt`:

- error: `e = r - y`
- proportional term: `P = Kp e`
- integral state: `I[k+1] = clamp(I[k] + Ki e dt)`
- derivative-on-measurement: `D = -Kd (y[k] - y[k-1]) / dt`
- unsaturated command: `u = P + I + D`
- output: `u_sat = clamp(u, output_min, output_max)`

The derivative is taken from the measurement rather than the setpoint, avoiding a derivative kick on a setpoint step.

## Anti-windup

The integral candidate is clamped to the configured integral limits. Conditional integration then rejects the candidate when the output is saturated and the current error would push it farther into saturation. When the error reverses direction, integration resumes so the controller can recover from a saturated state.

The current sample's candidate integral is still used to compute the output; only its persistent accumulation is rejected during the windup condition.

## PI / PD usage

- P: `Ki = 0`, `Kd = 0`
- PI: `Kd = 0`
- PD: `Ki = 0`
- PID: all three gains nonzero

All gains and limit values must be finite. Output and integral lower bounds must not exceed their corresponding upper bounds.

## State and reset

`reset(integral, measurement)` establishes a deterministic initial integral state and previous measurement. The measurement argument is important for derivative-on-measurement so the first update does not create an artificial derivative term.

## Acceptance tests

The P1-02 test suite covers:

- invalid configuration rejection;
- proportional and integral reference calculations;
- output saturation and conditional anti-windup;
- recovery when the error reverses;
- derivative-on-measurement and setpoint-kick avoidance;
- deterministic reset behavior;
- PI tracking of a second-order plant;
- PD damping and final position tracking.

`benchmarks/pid_benchmark.cpp` measures PID, PI and PD update throughput with the same fixed-size controller implementation.
