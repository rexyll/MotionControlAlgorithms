# mca_gazebo_adapter

Optional Gazebo Harmonic system plugin implementing the simulator-independent
`mca::simulation::IScalarGazeboSimulationAdapter` contract.

## Current scope

- Resolves one model joint by the SDF `joint_name` parameter.
- Caches a coherent scalar position/velocity snapshot in `PostUpdate`.
- Applies a finite scalar effort command through `JointForceCmd` in `PreUpdate`.
- Holds transport effort between messages with a steady wall-clock watchdog (default 0.1 s); timeout or simulation-time rollback clears the effort to zero.
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
| ROS/Gazebo -> adapter | `cmd_effort` | scalar effort command, held until a newer command or watchdog expiry |
| adapter -> ROS/Gazebo | `state/position` | joint position |
| adapter -> ROS/Gazebo | `state/velocity` | joint velocity |
| adapter -> ROS/Gazebo | `state/acceleration` | currently always `0.0` |
| adapter -> ROS/Gazebo | `state/time` | simulation time in seconds |
| adapter -> ROS/Gazebo | `state/step_index` | MCA simulation step index encoded as a double |

Transport commands are finite-checked and held until replaced. The default watchdog
is `0.1` wall-clock seconds; override it with the optional SDF element
`<command_timeout_s>0.1</command_timeout_s>`. The timeout must be finite and greater
than zero. A steady wall clock is used intentionally: headless simulation may advance
simulation time much faster than real time, so a simulation-time watchdog can expire
between ordinary ROS 2 publisher messages. If no new transport command arrives before
the watchdog expires, zero effort is applied. Simulation-time rollback also clears the
active transport command.
This transport watchdog is separate from the core adapter's step-matched
`write_command` mailbox.

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

This is integration-smoke evidence, not full acceptance. The strengthened Run #24
assertion (commit `96aa3c759217c0c5ac034608ea6b9672869876e0`) failed: the velocity
samples collected while the ROS 2 publisher was active remained near zero, exposing
that the previous one-step transport mailbox did not reliably actuate the joint.
The transport hold/watchdog was added in commit
`b8676f91deee932cc8a40d958689bf36f425b5c3`. Run #26 then showed that a simulation-time
watchdog expires too quickly under unthrottled headless simulation; the watchdog was
changed to steady wall-clock time in commit `0811896b2894fc58d55fbe14f0998594cd1d41c7`.
The follow-up Gazebo runtime result is still pending. Missing-joint diagnostics, malformed/non-finite transport input,
safe shutdown, repeated launch, and measured acceleration remain open robustness or
modeling limitations.
