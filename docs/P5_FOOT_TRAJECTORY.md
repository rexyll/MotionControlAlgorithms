# P5-03 Foot trajectory generation

The swing-foot generator interpolates world-frame start and finish positions with a quintic smoothstep and adds a smooth clearance arc along world Z.

## Position profile

For normalized time (u=t/T), clamped to the swing interval:

`s(u) = 10u^3 - 15u^4 + 6u^5`

`p(u) = p_start + s(u)(p_finish-p_start)`

The vertical coordinate receives an additional clearance term:

`h(u) = 64 h_{swing} u^3(1-u)^3`

The arc reaches the configured clearance at mid-swing and has zero first and second derivatives at both endpoints. The generated sample includes position, velocity and acceleration, with time derivatives scaled by the swing duration.

## API behavior

- `FootTrajectoryParameters`: world-frame endpoints, non-negative swing clearance and positive duration.
- `FootTrajectorySample`: world-frame position, velocity and acceleration.
- `evaluate_foot_trajectory`: returns false for non-finite/invalid inputs and preserves output on failure.
- Times before/after the swing clamp to stationary endpoint samples.

This is a deterministic geometric swing trajectory, not a gait scheduler or terrain planner. The clearance arc is world-Z aligned; terrain-relative normals and obstacle avoidance are future extensions.
