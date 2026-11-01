# P11 — Concrete ROS 2 and Gazebo runtime integration

P10 establishes simulator-independent ports and deterministic fake-adapter acceptance. P11 turns those contracts into optional, real middleware/simulator integrations while keeping the core library buildable without ROS 2 or Gazebo.

## Scope and non-goals

- Keep `include/mca/**` core interfaces independent of `rclcpp`, ROS messages, Gazebo headers, and simulator-owned types.
- Put concrete integrations in separate optional packages; default root CMake configure/build must remain dependency-free apart from existing core dependencies.
- Target ROS 2 Jazzy and Gazebo Harmonic. Verify installed package names and APIs in the pinned target environment before binding plugin APIs.
- Make timing, command freshness, invalid-message handling, reset behavior, and shutdown explicit.
- Do not claim runtime integration from fake-adapter tests. Record each real integration smoke test separately.
- Prefer headless deterministic launch/smoke tests first. Request local Ubuntu/ROS 2/Gazebo validation only for behavior that cannot be proven in CI.

## Staged tasks

- [x] P11-01 integration workspace and dependency boundary audit.
- [x] P11-02 concrete ROS 2 adapter: optional ament package at `ros2/mca_ros2_adapter`; scalar state publication and effort command subscription; rejects invalid state/non-finite command; missing or stale reads preserve caller output; dedicated ROS 2 Jazzy CI builds, installs, runs tests, and reports results.
- [x] P11-03 concrete Gazebo adapter: optional Gazebo Harmonic plugin builds and loads in the pinned headless runtime; reads joint state, estimates acceleration from consecutive velocity samples, applies validated `JointForceCmd`, enforces a steady-wall-clock command watchdog, and clears command/state/acceleration history on Reset. Final runtime evidence: Gazebo workflow [Run #223](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232810), code SHA `b07b0d1c883428a96bbab975b5b3ec80bc7f22dd`.
- [x] P11-04 launch-level single-axis integration: live ROS 2 Jazzy `parameter_bridge` ↔ Gazebo Harmonic ↔ `ScalarJointSystem` command/state path passes, including non-zero `JointForceCmd`, measured velocity and acceleration, watchdog expiry, active-command Reset, and post-reset motion. Final evidence is Gazebo workflow [Run #223](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232810) on code SHA `b07b0d1c883428a96bbab975b5b3ec80bc7f22dd`; root [CI Run #1312](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232803) passed Linux and Windows on the same SHA.
- [x] P11-05 runtime robustness and acceptance: live CI exercises NaN and positive-infinity rejection (including repeated rejection counting and proof that malformed traffic does not keep the watchdog alive), missing joint and missing required `<joint_name>` diagnostics, command timeout to zero effort, two Reset cycles while a valid command is active, post-reset zero-effort and fresh-motion recovery, graceful SIGINT shutdown without SIGKILL, and relaunch of the same world/plugin in the same Gazebo Transport partition with command-subscriber rediscovery. Final evidence: Gazebo workflow [Run #223](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232810), plus Linux/Windows core regression CI [Run #1312](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232803), both on code SHA `b07b0d1c883428a96bbab975b5b3ec80bc7f22dd`. Scope is pinned Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic CI smoke acceptance; it is not production hardware, safety certification, a missing-model-entity test, or local VMware performance evidence.

## Acceptance gates

1. Root core Configure/Build/Test must pass on Linux and Windows without ROS 2 or Gazebo installed.
2. ROS 2-specific build/tests run only in an explicitly enabled ROS 2 environment and must not silently skip when that workflow claims integration coverage.
3. Gazebo plugin build and runtime smoke tests run in a pinned Ubuntu/ROS 2 Jazzy/Gazebo Harmonic environment; a fake adapter alone is not runtime acceptance evidence.
4. All middleware callbacks translate into core values at the boundary; core algorithms never depend on middleware types.
5. Every failed read/write or invalid command has defined output/state preservation behavior.
6. Documentation distinguishes unit, adapter-contract, package-build, and live-runtime evidence.

## P11-01 acceptance

Audit result (2026-10-26): the root `CMakeLists.txt` requires Eigen3 and conditionally finds GoogleTest when `MCA_BUILD_TESTS` is enabled; benchmarks are opt-in. It does not discover ROS 2 or Gazebo and therefore keeps the default core build isolated from middleware/simulator dependencies. `tests/simulation/step_test.cpp` is registered in `mca_core_tests`. This audit is documentation/build-boundary evidence only; it does not claim ROS 2 or Gazebo runtime behavior.

## P11-02 acceptance — ROS 2 Jazzy scalar adapter

Accepted 2026-10-26.

Implementation: `ros2/mca_ros2_adapter` publishes [simulation time, dt, step index, position, velocity, acceleration] via `std_msgs/Float64MultiArray` and consumes scalar effort via `std_msgs/Float64`. It rejects invalid state, ignores non-finite commands, and refuses missing/stale commands without modifying caller output. Freshness uses `std::chrono::steady_clock`.

Evidence:
- ROS 2 Jazzy workflow Run #13, commit `18e7460515b304cc5b9677044c3cb634f2268cc5`: package configure/build/install succeeded; `colcon test` ran the adapter gtest target; `colcon test-result` reported 4 tests, 0 errors, 0 failures, 0 skipped. The four cases cover invalid state, missing command/output preservation, stale command/output preservation, and finite command topic flow.
- Root CI Run #1073, same commit: Linux and Windows Configure/Build/Test and dedicated P8-39 through P8-46 regressions passed on both platforms.
- Evidence class: ROS 2 package build/install + focused ROS 2 tests, and independent core cross-platform CI. This is not Gazebo integration or production deployment evidence.

Known limitation: the array wire format is prototype-only; step index is encoded as a double. Before external deployment, replace it with a versioned custom message containing an integer step index, explicit units, and a schema compatibility policy. The topic test verifies ROS 2 topic command flow in CI, not end-to-end Gazebo behavior.


## P11-03 implementation status (2026-10-26)

The first concrete Gazebo Harmonic package is present under `gazebo/mca_gazebo_adapter`:
- CMake/package metadata targets Gazebo Sim 8 (Harmonic), plugin registration, and the simulator-independent scalar Gazebo adapter contract.
- `ScalarJointSystem` resolves a model joint by SDF `joint_name`, captures position/velocity in `PostUpdate`, and applies finite step-matched core-adapter commands in `PreUpdate`. Gazebo Transport commands use a separate active command with a steady wall-clock watchdog (default 0.1 s, configurable via `<command_timeout_s>`); timeout and simulation-time rollback clear effort to zero.
- Dedicated workflow `.github/workflows/gazebo-adapter.yml` installs Harmonic dependencies, builds the plugin, and runs the headless world `gazebo/mca_gazebo_adapter/test/worlds/single_axis_smoke.sdf`.
- Gazebo workflow Run #4, commit `21af979501eb81469da38e7a9716c45b6f38461a`, passed: CMake configure/build succeeded and logs confirmed `mca_gazebo_adapter::ScalarJointSystem` loaded into the smoke world. Root CI Run #1086 on the same commit passed Linux and Windows core Configure/Build/Test.
- Initial evidence (Run #4, commit `21af979501eb81469da38e7a9716c45b6f38461a`): plugin configure/build and load/startup smoke passed; root CI Run #1086 passed Linux and Windows core tests.
- Updated runtime evidence (2026-10-27): Gazebo workflow Run #14 passed plugin build/load and transport command/state smoke; Run #16 exposed an unsupported `gz topic -r` CLI flag; after replacing it with repeated ordinary publishes, Run #17 passed the headless plugin smoke and transport command/state smoke. Root CI Run #1101 passed Linux and Windows Configure/Build/Test, including P8-39 through P8-46 regressions.
- ROS 2 ↔ Gazebo bridge evidence (2026-10-27): Gazebo workflow [Run #22](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/40456187215), commit `42e62a35cc8e881b9f4509a3887749956d05c880`, passed plugin build, headless load, and a live `ros_gz_bridge` command/state smoke. The workflow launches the single-axis SDF world, bridges `std_msgs/Float64` effort commands into Gazebo Transport, bridges velocity state back to ROS 2, publishes effort, and checks that the observed velocity stream contains a magnitude above `1e-5`. Main CI [Run #1107](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/40456187196) passed Linux and Windows Configure/Build/Test, including P8-39 through P8-46 regression targets.
- Evidence boundary: this proves the basic ROS 2 ↔ Gazebo command/state path in the pinned Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic CI container. It is a smoke test, not proof of a production controller, safety-rated behavior, or local VMware/Ubuntu performance.
- Strengthened bridge test result (2026-10-27): Gazebo workflow [Run #24](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/40458399944), commit `96aa3c759217c0c5ac034608ea6b9672869876e0`, failed at the stricter assertion that requires non-trivial velocity while the ROS 2 effort publisher is active. The captured velocity samples stayed near zero (about `-1.1e-9`), despite the bridge confirming that ROS effort messages reached Gazebo. This exposed that the old one-step transport mailbox did not reliably produce sustained actuation; Run #24 is not acceptance evidence for motion.
- Corrective implementation history: commit `b8676f91deee932cc8a40d958689bf36f425b5c3` separated Gazebo Transport commands from the step-matched core mailbox and added command hold/expiry; after Run #26 exposed premature expiry under unthrottled simulation, commit `0811896b2894fc58d55fbe14f0998594cd1d41c7` moved freshness to `steady_clock`. The adapter now also estimates acceleration from consecutive post-physics velocity samples; the first sample after startup/reset is intentionally a zero baseline.
 
## P11-03 / P11-04 / P11-05 final acceptance — 2026-11-01

All three tasks are accepted from live, version-pinned CI evidence.

- Gazebo workflow [Run #223](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232810), run ID `41281232810`, completed successfully on code SHA `b07b0d1c883428a96bbab975b5b3ec80bc7f22dd`. Configure, Build, and the live ROS 2 ↔ Gazebo smoke step all passed.
- Root [CI Run #1312](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41281232803), run ID `41281232803`, completed successfully on the same SHA. Both Linux and Windows Configure/Build/Test and the dedicated P8-39 through P8-46 regression targets passed.

The Gazebo workflow assertions cover:
1. ROS 2 `std_msgs/msg/Float64` effort forwarding through `ros_gz_bridge`, typed Gazebo Transport callback receipt, non-zero `JointForceCmd`, and non-trivial measured velocity and finite-difference acceleration observed through the ROS 2/Gazebo state paths.
2. Watchdog expiry after the configured 0.500000 s wall-clock deadline, with measured expiry age 0.500566 s and applied effort exactly zero.
3. NaN and positive-infinity effort rejection at the plugin's real Transport callback boundary. The test verifies each malformed message is rejected, repeated rejection diagnostics are counted, and the previously accepted valid command still expires instead of malformed traffic refreshing its watchdog.
4. Actionable configuration diagnostics for a model whose requested joint does not exist and for a plugin instance missing the required `<joint_name>` SDF element.
5. Two Reset stress cycles while the valid `single_axis_smoke/axis_joint` plugin has an active Transport command. Both callbacks report `had_transport_command=true`, `applied_effort=0`, `step_index=0`, invalidated state, and an available force component. The workflow also verifies post-reset zero-effort application and fresh post-reset motion on Gazebo Transport and ROS 2.
6. Graceful SIGINT shutdown of both the initial and relaunched Gazebo processes without SIGKILL, followed by relaunch of the same world in the same Transport partition and rediscovery of its command subscriber/state publisher.

Evidence boundary: this accepts the adapter's tested runtime and lifecycle smoke contract in Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic CI. It does not establish production hardware behavior, safety certification, local VMware performance, an absent-model-entity test, or an explicit simulation-time-rollback injection test. The adapter's documented default watchdog remains 0.1 s; the CI fixture overrides it to 0.5 s to tolerate container scheduling jitter.
