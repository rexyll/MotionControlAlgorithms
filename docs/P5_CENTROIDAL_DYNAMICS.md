# P5-05 Centroidal dynamics

The centroidal dynamics helper aggregates the resultant force and moment for a floating-base/legged robot. It is a compact Newton-Euler force aggregation primitive, not a substitute for articulated-body inverse or forward dynamics.

## Coordinate and sign conventions

All vectors are expressed in the world frame. The center of mass is `center_of_mass_world`; each applied force has a world-frame application point and force vector. Gravity is included separately and assumed to act through the center of mass, so it contributes no moment about the COM. `external_torque_world` is an additional free moment about the COM.

For total mass (m), gravity (g), applied forces (F_i) at points (p_i), and COM position (c):

- Net force and linear momentum rate: (F_{net} = \dot{P} = m g + \sum_i F_i).
- COM acceleration: \(\ddot{c} = F_{net}/m\).
- Angular momentum rate about the COM: \(\dot{L}_c = \tau_{ext} + \sum_i (p_i-c) \times F_i\).

The input forces can represent contact forces, external pushes, or other non-gravitational forces. Do not include gravity again in `applied_forces`, or it will be double-counted.

## API

- `CentroidalDynamicsInput`: positive total mass, COM position, world gravity, applied point forces, and optional free external torque.
- `CentroidalDynamicsOutput`: net force, linear momentum rate, COM acceleration, and angular momentum rate.
- `evaluate_centroidal_dynamics`: validates inputs and computed outputs; writes the output only on success.

The implementation checks non-finite vectors and arithmetic overflow in the computed result. The caller remains responsible for supplying physically consistent contact forces and force application points. The routine does not estimate mass properties, angular momentum itself, contact state, or solve contact constraints.

## Verification

Tests cover gravity-only acceleration, multiple force and moment aggregation, COM-relative moment arms, invalid input transaction safety, and non-finite computed output. A Google Benchmark target measures repeated evaluation with four contact forces.
