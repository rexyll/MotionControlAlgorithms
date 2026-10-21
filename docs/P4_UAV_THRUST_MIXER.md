# P4-07 Quadrotor thrust and mixer allocation

## Scope

Header-only, fixed-size X-configuration quadrotor wrench allocation. It maps collective thrust and body-axis roll/pitch/yaw moments to four nonnegative rotor thrust commands without ROS 2, Gazebo, or hardware dependencies.

## Conventions and desaturation

Rotor order is front-left, rear-right, front-right, rear-left. The configured arm length and yaw torque-per-thrust coefficient define the mixer. Collective thrust is held fixed and requested moments are uniformly scaled by a common factor in [0, 1], found by bounded bisection, until all rotor commands lie within [0, max_rotor_thrust]. This preserves the relative moment request when saturation occurs.

The method rejects collective thrust outside [0, 4 * max_rotor_thrust], non-finite wrench inputs, and invalid configuration. Failed allocation leaves the caller's output unchanged. Output includes rotor thrust, reconstructed achieved wrench, and the moment scale factor.

## Limitations

This is a baseline symmetric quadrotor mixer, not a general control-allocation optimizer. If the requested collective thrust itself is infeasible, it returns an error rather than silently changing altitude demand. Real vehicles must validate rotor ordering, rotation directions, frame conventions, thrust calibration, and actuator dynamics against hardware.

## Validation

Tests cover collective-only allocation, wrench reconstruction, moment desaturation with collective preservation, configuration/input validation, and transactional failure behavior. A deterministic micro-benchmark is available when Google Benchmark is installed.
