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
- Implements Gazebo Sim's `ISystemReset` contract: clears cached state, step index,
  pending and transport commands, acceleration history, and writes zero
  `JointForceCmd` while keeping the plugin's Transport endpoints alive.
- Exposes a Gazebo Transport command channel and scalar state channels so a
  ROS 2 bridge can connect without coupling the core simulation interfaces to ROS.

Joint acceleration is estimated from consecutive post-physics velocity samples
using (velocity[k] - velocity[k-1]) / simulation_dt. The first sample after startup
or a simulation reset/time rollback is a zero baseline because no prior sample exists.
This is a finite-difference estimate, not a direct physics-engine acceleration
measurement; consumers should account for the sample interval and reset boundary.

## Gazebo Transport contract

The optional SDF element `<topic_prefix>` selects the transport prefix. If omitted,
the default is `/mca/<model>/joint/<joint>`. The following `gz.msgs.Double` topics are
created:

| Direction | Topic suffix | Meaning |
| --- | --- | --- |
| ROS/Gazebo -> adapter | `cmd_effort` | scalar effort command, held until a newer command or watchdog expiry |
| adapter -> ROS/Gazebo | `state/position` | joint position |
| adapter -> ROS/Gazebo | `state/velocity` | joint velocity |
| adapter -> ROS/Gazebo | `state/acceleration` | finite-difference joint acceleration estimate |
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

World reset is provided by Gazebo's `/world/<world_name>/control` service
(`gz.msgs.WorldControl`, reset all). The adapter's `ISystemReset::Reset`
implementation invalidates the previous state snapshot, returns the step index
to zero, clears command freshness and the prior-velocity sample, and writes
zero effort to the joint. It intentionally retains the existing Transport node,
subscription and state publishers so the same plugin instance can resume
command/state exchange after the reset.

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

The Gazebo workflow builds the plugin against Gazebo Harmonic and runs a live ROS 2 Jazzy
`ros_gz_bridge` ↔ Gazebo Transport ↔ `ScalarJointSystem` smoke test in the pinned
Ubuntu 24.04 CI container.

Final acceptance evidence:

- [Gazebo workflow Run #223](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232810)
  passed Configure, Build, and the live command/state smoke on code SHA
  `b07b0d1c883428a96bbab975b5b3ec80bc7f22dd`.
- [Root CI Run #1312](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232803)
  passed Linux and Windows Configure/Build/Test and P8-39 through P8-46 regressions
  on the same SHA.

The live smoke verifies ROS 2 effort forwarding, typed plugin callback receipt,
non-zero `JointForceCmd`, measured velocity and finite-difference acceleration,
wall-clock watchdog expiry with zero applied effort, rejection of NaN and positive
infinity without refreshing the watchdog, missing-joint and missing-`joint_name`
diagnostics, two Reset cycles during an active command, post-reset zero-effort and
fresh-motion recovery, graceful SIGINT shutdown without SIGKILL, and relaunch of
the same world/plugin in the same Gazebo Transport partition with endpoint
rediscovery. The CI watchdog fixture uses a 0.5 s timeout to tolerate scheduling
jitter; the adapter default remains 0.1 s.

The evidence is integration-smoke acceptance, not proof of production hardware
behavior, safety certification, local VMware/Ubuntu performance, an absent-model
entity case, or an explicit simulation-time-rollback injection test. Earlier failed
runs (#24 and #26) were useful diagnostic evidence: sustained actuation required a
held Transport command, and the watchdog needed steady wall-clock rather than
simulation time under unthrottled headless execution. Those failures are superseded
by the passing final run for the behaviors listed above.
