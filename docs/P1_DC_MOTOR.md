# P1-04 DC Motor Model

## Scope

`mca::plant::DcMotor` is the deterministic single-axis electromechanical plant used by the servo experiments. It models the averaged electrical and mechanical dynamics without PWM switching details.

The model is intentionally independent of ROS 2, Gazebo and hardware SDKs so it can be used in unit tests, controller benchmarks and later adapter-based simulations.

## Equations

Electrical dynamics:

\[
L\dot i = V - Ri - K_e\omega
\]

Mechanical dynamics:

\[
J\dot\omega = K_t i - B\omega - \tau_{load}
\]

Position:

\[
\dot\theta = \omega
\]

Where:

- `V`: applied motor voltage [V]
- `i`: armature current [A]
- `R`: winding resistance [ohm]
- `L`: winding inductance [H]
- `K_e`: back-EMF constant [V/(rad/s)]
- `K_t`: torque constant [N·m/A]
- `J`: rotor/load inertia [kg·m²]
- `B`: viscous damping [N·m/(rad/s)]
- `tau_load`: external load torque [N·m]
- `omega`: angular velocity [rad/s]
- `theta`: angular position [rad]

For the SI convention used by the project, `K_e` and `K_t` are numerically consistent in the idealized SI model.

## Numerical integration

The implementation uses semi-implicit Euler:

1. Integrate current from the electrical equation.
2. Compute electromagnetic torque from the updated current.
3. Integrate angular velocity from the mechanical equation.
4. Integrate position from the updated angular velocity.

This is appropriate for controller/plant experiments. It is **not** a switching-level inverter, PWM or FOC electrical simulation.

## Validation coverage

`tests/plant/dc_motor_test.cpp` covers:

1. invalid physical parameters are rejected;
2. at zero speed, current converges toward the Ohmic steady state `I = V/R`;
3. positive voltage produces positive current, torque and motion;
4. back EMF reduces current when the rotor is already spinning.

The model is also exercised by the deterministic `CascadedServoExperiment`, which closes the loop as:

`position PID -> velocity PID -> torque PID -> DC motor`.

The experiment reports position/velocity/torque tracking metrics and actuator constraint violations, providing the integration-level benchmark for the model.

## Acceptance baseline

P1-04 is accepted when all of the following hold:

- configuration validation rejects non-physical or non-finite parameters;
- analytical DC motor steady-state behavior is covered by unit tests;
- current, torque and mechanical motion have the expected signs and units;
- back EMF behavior is covered by a regression test;
- the model participates in a deterministic closed-loop servo benchmark;
- Linux and Windows CI pass the complete test suite.

## Real-time boundary

The class contains only fixed-size state and performs no dynamic allocation during `step()`. It can therefore be used as a deterministic plant model inside experiments or simulation adapters. It should not be interpreted as a replacement for a drive/MCU current loop or switching model in a hard-real-time motor controller.

## Next step

P1-05 extends the plant model toward BLDC/PMSM dynamics, preserving the same separation between plant/model, controller and external simulation adapters. The expected progression is:

`DC motor -> BLDC/PMSM -> Clarke/Park -> PMSM dq -> FOC current loop -> decoupling/feedforward -> SVPWM`.
## Checked power diagnostics

`electrical_power(voltage_v, output)` reports terminal electrical input power `V * i` using the supplied terminal voltage and the model's current state. `mechanical_power(output)` reports shaft mechanical power `T_e * omega`. Both APIs return a status, preserve the output argument on failure, and do not mutate plant state. Non-finite inputs are rejected; non-finite intermediate or final arithmetic is reported as `NumericalError`. These diagnostics are useful for deterministic energy/power checks but do not model inverter losses or switching.
