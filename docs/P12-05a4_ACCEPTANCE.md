# P12-05a.4 Acceptance — live Gazebo + ros2_control closed loop

Date: 2026-11-02

## Result

**Accepted for live Gazebo Harmonic + ROS 2 Jazzy simulation integration only.**

The pluginlib-exported `mca_gz_ros2_control_adapter` system plugin was built,
loaded by Gazebo, initialized against the two joints in the test world, and
connected to ROS 2 Jazzy `controller_manager`. The test exercised live Gazebo
joint state feedback and effort command flow, then explicitly deactivated and
reactivated the hardware component and its dependent controllers.

This is simulation-runtime evidence. It is not evidence for a physical actuator,
vendor driver, safety-rated stop, real-time guarantees, or hardware-in-the-loop
behavior.

## Implementation and regression fixed

Commit `dc43644d0e82384f8c96ad2725a52ec7dfd1e17e`
(`fix(gazebo): preserve command sequence across reactivation`) removes the
reset of `next_command_sequence_` from `GazeboSystem::on_activate()`.

The backend retains its last accepted command sequence across hardware
deactivation/reactivation. Resetting the adapter-side sequence to zero caused
the first post-reactivation command to be rejected as stale
(`InvalidArgument`), after which `controller_manager` automatically
deactivated the hardware. Keeping the sequence monotonic removes that mismatch.

## Live acceptance evidence

Gazebo Harmonic / ROS 2 Jazzy workflow
[Run #28](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41393338877)
(job `125109264996`) completed successfully on the commit above.

The job log confirms:

1. Gazebo resolved URDF joints `joint_a` and `joint_b` to Gazebo entities
   11 and 12 and successfully bound both joints.
2. `McaGazeboSystem` initialized, configured and activated successfully.
3. `joint_state_broadcaster` and `effort_controller` loaded, configured and
   became active.
4. The live joint-state capture collected 2,938 valid samples. The first
   position was `[0.0, 0.0]`, the last was approximately
   `[3.14, 3.14]`, and the absolute deltas were approximately
   `[3.14, 3.14]` rad. The existing motion threshold
   `max(delta) >= 1e-4` was not weakened.
5. Both controllers were explicitly deactivated before hardware deactivation.
   `McaGazeboSystem` successfully transitioned to `inactive`.
6. Hardware successfully transitioned back to `active`; then both
   `joint_state_broadcaster` and `effort_controller` were successfully
   reactivated and reported `active`.
7. No stale-sequence rejection (`backend_status_ok=false`) appeared in the
   run log. The workflow's final marker was
   `Live Gazebo + controller_manager closed-loop acceptance passed`.

The run did not report a Gazebo fatal runtime failure, segmentation fault,
AddressSanitizer failure, termination, or uncaught exception. Two non-fatal
ROS control diagnostics were present: hardware initialization occurred before
an executor was available, so optional node creation was skipped; read/write
statistics were not initialized, so statistics registration was skipped.
Neither prevented hardware initialization, controller operation, motion
feedback, or lifecycle recovery.

Root CI [Run #1398](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41393338912)
also completed successfully on the same SHA. Both Linux and Windows jobs
passed, including the existing Configure, Build, Test and P8-39 through P8-46
regression coverage.

## Scope and remaining work

This acceptance closes P12-05a.4: live Gazebo Harmonic + ROS 2 Jazzy
`ros2_control` simulation integration, including joint motion and hardware /
controller lifecycle recovery.

It does not close the concrete vendor device backend task or HIL task. P12-05b
still requires selection of a real actuator/drive and verification against its
official SDK, transport, control modes and fault documentation. P12-06 still
requires access to that physical target and separate HIL evidence for timing,
watchdog, fault propagation, stop behavior, startup/shutdown and reconnect.
