# P4-03 Cascaded position/attitude PID

## Purpose

mca::uav::CascadedPositionAttitudePid provides a dependency-light baseline for UAV position and attitude feedback. It composes six existing scalar PID controllers and leaves actuator allocation to a separate layer.

## Control law and frames

- Three outer-loop position PIDs compare world-frame position setpoints and measurements. Their outputs are desired world-frame linear accelerations in m/s².
- The controller computes F_w = m(a_cmd - g_w), which adds gravity compensation, then rotates that force into the current body frame.
- Three inner-loop attitude PIDs compare ZYX Euler angles extracted from body-to-world quaternions. The attitude setpoint error is wrapped to [-π, π] so yaw transitions across the ±π boundary take the short path.
- The inner-loop outputs are body-frame torque commands. PID output limits can be used to bound desired acceleration and torque independently.

The output contains desired acceleration in world coordinates, force in body coordinates, and torque in body coordinates. SI units are used; angles are radians.

## Configuration and runtime behavior

Set the vehicle mass and world gravity, then configure the three position and three attitude PID axes with mca::control::PidConfig. Every PID should have physically meaningful output and integral limits. update(...) requires finite states/setpoints, non-zero finite attitude quaternions, and a positive finite timestep. Invalid inputs return a non-OK mca::core::Status and leave the output object unchanged. reset() clears all six PID states.

This is a baseline controller, not a complete flight stack. It does not enforce rotor thrust limits, tilt limits, aerodynamic compensation, motor dynamics, or actuator mixing; those are separate follow-on tasks. Euler feedback also inherits the usual ZYX singularity near pitch ±π/2.

## Validation

Unit tests cover gravity-compensated hover, expected position/attitude command signs and scaling, yaw error wrapping at the ±π boundary, invalid input with transactional output behavior, and invalid configuration. A deterministic micro-benchmark measures the composed update path.
