# P3-09 Jerk-limited S-curve trajectory

`SCurveTrajectory` generates a symmetric rest-to-rest point-to-point profile with seven phases: positive jerk, constant acceleration, negative jerk, cruise, negative jerk, constant deceleration, and positive jerk. It chooses a triangular-jerk, acceleration-limited, velocity-limited, or cruise-containing profile from the move distance and configured limits.

## API and behavior

- Constructor: `SCurveTrajectory(start, finish, max_velocity, max_acceleration, max_jerk)`.
- `valid()` reports finite inputs and strictly positive limits.
- `duration()` reports the computed profile duration in seconds.
- `sample(t)` returns position, velocity and acceleration; reverse moves preserve signed velocity/acceleration.
- The trajectory starts and finishes at rest. Samples outside the profile clamp to the corresponding endpoint.
- The implementation is header-only and independent of ROS 2/Gazebo. Sampling uses a fixed seven-phase array and no dynamic allocation.

## Verification

Tests cover endpoints, velocity/acceleration/jerk bounds, reverse moves, short and long moves, invalid limits and zero distance. GitHub Actions Run #321 passed Configure, Build and Test on both Linux and Windows: https://github.com/rexyll/MotionControlAlgorithms/actions/runs/39580810849. Implementation and benchmark integration were validated at commit `a9cf8e5865c1133aac8aae0cc0d4f2d7a82fac84`.
