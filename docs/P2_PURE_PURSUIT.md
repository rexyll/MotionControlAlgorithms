# P2-04 Pure Pursuit

## Scope

This module provides a ROS2/Gazebo-independent geometric pure-pursuit controller for a planar polyline path.

The controller accepts the current planar pose and a sequence of path points, selects a forward lookahead target, transforms that target into the vehicle frame, and returns curvature plus a commanded linear/angular velocity pair.

## Convention

- `x` forward and `y` left in the vehicle frame.
- Positive yaw and angular velocity are counter-clockwise.
- Positive lateral target displacement therefore produces positive curvature.
- The command uses `angular_z = linear_x * curvature`.

## Algorithm

1. Validate the configuration, pose and path.
2. Find the path vertex nearest to the current position.
3. Walk forward from that vertex by the configured lookahead distance.
4. If the path ends first, use the final path point.
5. Transform the target into the vehicle frame.
6. Compute

   `kappa = 2 * y_vehicle / (x_vehicle^2 + y_vehicle^2)`.

7. Return the configured linear speed and `angular_z = v * kappa`.

The lookahead target is interpolated within a path segment, avoiding unnecessary quantization to path vertices.

## API

```cpp
mca::path_tracking::PurePursuitConfig config{
    .lookahead_distance,  // meters
    .linear_speed         // meters/second
};

mca::path_tracking::PurePursuit controller(config);
std::vector<mca::path_tracking::PathPoint2D> path = ...;
mca::path_tracking::PurePursuitCommand command;
controller.compute(pose, path, command);
```

## Design boundary

The primitive intentionally does not own:

- wheel kinematics or wheel-speed allocation;
- vehicle dynamics;
- path resampling or smoothing;
- localization, odometry, TF or sensor interfaces;
- ROS2/Gazebo message types.

Those concerns belong to higher-level adapters or later motion-control tasks.

## Verification

`tests/path_tracking/pure_pursuit_test.cpp` covers:

- invalid configuration and inputs;
- straight-path zero curvature;
- lateral-error turn direction;
- heading-frame transformation;
- lookahead interpolation and end-of-path behavior;
- closed-loop straight-path convergence from a lateral offset.

`benchmarks/pure_pursuit_benchmark.cpp` measures command-generation cost against a deterministic sinusoidal polyline.
