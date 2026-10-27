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
- [ ] P11-03 concrete Gazebo adapter: implement the smallest supported Gazebo Harmonic system/plugin that reads a coherent state snapshot and applies validated actuator commands, with explicit simulation-step mapping and reset behavior.
- [ ] P11-04 launch-level single-axis integration: wire a minimal single-axis model/controller path through ROS 2 and Gazebo, expose deterministic startup/reset/shutdown, and verify command/state flow in a headless smoke test.
- [ ] P11-05 runtime robustness and acceptance: cover stale commands, malformed/non-finite inputs, missing entities, simulator reset, shutdown, and repeated launch; record CI and target-environment evidence independently.

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
- `ScalarJointSystem` resolves a model joint by SDF `joint_name`, captures position/velocity in `PostUpdate`, and applies finite, step-matched scalar effort in `PreUpdate`; absent commands default the joint force command to zero.
- Dedicated workflow `.github/workflows/gazebo-adapter.yml` installs Harmonic dependencies, builds the plugin, and runs the headless world `gazebo/mca_gazebo_adapter/test/worlds/single_axis_smoke.sdf`.
- Gazebo workflow Run #4, commit `21af979501eb81469da38e7a9716c45b6f38461a`, passed: CMake configure/build succeeded and logs confirmed `mca_gazebo_adapter::ScalarJointSystem` loaded into the smoke world. Root CI Run #1086 on the same commit passed Linux and Windows core Configure/Build/Test.
- Initial evidence (Run #4, commit `21af979501eb81469da38e7a9716c45b6f38461a`): plugin configure/build and load/startup smoke passed; root CI Run #1086 passed Linux and Windows core tests.
- Updated runtime evidence (2026-10-27): Gazebo workflow Run #14 passed plugin build/load and transport command/state smoke; Run #16 exposed an unsupported `gz topic -r` CLI flag; after replacing it with repeated ordinary publishes, Run #17 passed the headless plugin smoke and transport command/state smoke. Root CI Run #1101 passed Linux and Windows Configure/Build/Test, including P8-39 through P8-46 regressions.
- ROS 2 ↔ Gazebo bridge evidence (2026-10-27): Gazebo workflow [Run #22](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/40456187215), commit `42e62a35cc8e881b9f4509a3887749956d05c880`, passed plugin build, headless load, and a live `ros_gz_bridge` command/state smoke. The workflow launches the single-axis SDF world, bridges `std_msgs/Float64` effort commands into Gazebo Transport, bridges velocity state back to ROS 2, publishes effort, and checks that the observed velocity stream contains a magnitude above `1e-5`. Main CI [Run #1107](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/40456187196) passed Linux and Windows Configure/Build/Test, including P8-39 through P8-46 regression targets.
- Evidence boundary: this proves the basic ROS 2 ↔ Gazebo command/state path in the pinned Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic CI container. It is a smoke test, not proof of a production controller, safety-rated behavior, or local VMware/Ubuntu performance.
- Remaining gaps before P11-03/P11-04/P11-05 acceptance: acceleration is currently reported as zero; reset/time rollback, missing-joint diagnostics, malformed/non-finite transport input, safe shutdown and repeated launch, stale-command timing, and stronger deterministic assertions still need explicit tests. Keep task checkboxes open until those criteria are exercised.
