# mca_gazebo_adapter

Optional Gazebo Harmonic system plugin implementing the simulator-independent
`mca::simulation::IScalarGazeboSimulationAdapter` contract.

## Current scope

- Resolves one model joint by the SDF `joint_name` parameter.
- Caches a coherent scalar position/velocity snapshot in `PostUpdate`.
- Applies a finite scalar effort command through `JointForceCmd` in `PreUpdate`.
- Rejects invalid steps/commands and preserves caller output when state is unavailable.
- Resets command output to zero when no command for the expected step is pending.
- Exposes a Gazebo Transport command channel and scalar state channels so a
  ROS 2 bridge can connect without coupling the core simulation interfaces to ROS.

Joint acceleration is currently reported as zero because the generic joint position
and velocity components do not provide a portable acceleration field. This is an
explicit limitation, not a measured acceleration.

## Gazebo Transport contract

The optional SDF element `<topic_prefix>` selects the transport prefix. If omitted,
the default is `/mca/<model>/joint/<joint>`. The following `gz.msgs.Double` topics are
created:

| Direction | Topic suffix | Meaning |
| --- | --- | --- |
| ROS/Gazebo -> adapter | `cmd_effort` | scalar effort command, consumed for the current simulation step |
| adapter -> ROS/Gazebo | `state/position` | joint position |
| adapter -> ROS/Gazebo | `state/velocity` | joint velocity |
| adapter -> ROS/Gazebo | `state/acceleration` | currently always `0.0` |
| adapter -> ROS/Gazebo | `state/time` | simulation time in seconds |
| adapter -> ROS/Gazebo | `state/step_index` | MCA simulation step index encoded as a double |

A transport command is finite-checked and placed into the same step-matched mailbox
used by the adapter contract. Commands are therefore not latched indefinitely: if
no fresh command is received for the expected step, zero effort is applied.

The topics intentionally use standard Gazebo `Double` messages so `ros_gz_bridge`
can map them to `std_msgs/msg/Float64` without a custom message dependency. The
five state channels are separate transport messages; consumers that require a
coherent vector should correlate them by `state/step_index` and treat a changed
step as a new snapshot.

## Build (Ubuntu 24.04 / Gazebo Harmonic)

Install the Gazebo Harmonic development packages, including `gz-transport13` and
`gz-msgs10`, then:

```bash
cmake -S gazebo/mca_gazebo_adapter -B build/gazebo-adapter
cmake --build build/gazebo-adapter --parallel 2
```

The plugin is deliberately outside the root CMake project so core-only builds do
not require Gazebo. Load the shared library using Gazebo's plugin search path and
configure it with an SDF `<plugin>` element containing `<joint_name>...</joint_name>`.

## Evidence boundary

The Gazebo workflow verifies dependency installation, plugin compilation, headless
plugin loading, and a live ROS 2 ↔ Gazebo bridge smoke test. Run #22 on commit
`42e62a35cc8e881b9f4509a3887749956d05c880` publishes a ROS 2 effort command, bridges it
to Gazebo Transport, and observes a velocity response through the reverse bridge.
See [workflow Run #22](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/40456187215).

This is integration-smoke evidence, not full acceptance. Reset/time rollback, stale
command timing, malformed/non-finite transport input, missing-joint diagnostics,
safe shutdown, repeated launch, and measured acceleration remain open robustness or
modeling limitations.
