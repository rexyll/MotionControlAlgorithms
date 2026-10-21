# Penalty contact model

P5-02 adds a deterministic point-contact model against a stationary plane.

## Conventions and equations

The plane normal points into the admissible half-space. Gap is positive when separated and negative when penetrating. The normal is normalized internally. The contact force is expressed in world coordinates.

Let (g) be gap, (v_n) normal velocity, (k_n) normal stiffness and (d_n) normal damping:

`F_n = max(0, k_n max(0, -g) - d_n v_n)`

This unilateral penalty law never pulls the point toward the plane. Normal damping can produce an impact response at zero gap when approaching the plane. Tangential force opposes tangential velocity and is regularized by tangential damping, capped by the Coulomb bound `mu * F_n`.

## API and limitations

- `ContactPointState`: gap, world velocity and plane normal.
- `ContactModelParameters`: non-negative stiffness/damping and friction coefficient.
- `ContactWrench`: total world force, normal-force magnitude and active flag.
- `evaluate_contact`: validates inputs and preserves output on failure.

This is a penalty approximation for simulation and algorithm development, not a rigid complementarity solver. Stiffness affects numerical stability and should be tuned to the integrator/time step. It models one point against one plane and does not calculate contact moments.
