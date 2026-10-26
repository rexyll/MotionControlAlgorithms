# P11 — Concrete ROS 2 and Gazebo runtime integration

P10 establishes simulator-independent ports and deterministic fake-adapter acceptance. P11 turns those contracts into optional, real middleware/simulator integrations while keeping the core library buildable without ROS 2 or Gazebo.

## Scope and non-goals

- Keep `include/mca/**` core interfaces independent of `rclcpp`, ROS messages, Gazebo headers, and simulator-owned types.
- Put concrete integrations in separate optional packages; default root CMake configure/build must remain dependency-free apart from existing core dependencies.
- Use ROS 2 Jazzy and Gazebo Harmonic as the target stack. Confirm installed package names and API versions in the target environment before binding plugin APIs.
- Make timing, command freshness, invalid-message handling, reset behavior, and shutdown explicit.
- Do not claim runtime integration from fake-adapter tests. Record each real integration smoke test separately.
- Prefer headless deterministic launch/smoke tests first. Request local Ubuntu/ROS 2/Gazebo validation only for behavior that cannot be proven in standard CI.

## Staged tasks

- [x] P11-01 integration workspace and dependency boundary audit: document the optional package layout, supported ROS 2/Gazebo versions, root-build isolation, and test strategy before adding middleware-specific code.
- [ ] P11-02 concrete ROS 2 adapter: implement a ROS 2 Jazzy package that maps typed core state/command values to messages, validates incoming commands, preserves core output on unavailable/stale input, and has ROS-enabled compile/unit tests.
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

Before implementation, inspect root CMake and CI to verify optional packages are excluded from default builds. Document package names, dependency gates, and which test runner owns each layer. No local Gazebo run is required for this planning/audit task.

Audit result (2026-10-26): the root `CMakeLists.txt` requires Eigen3 and conditionally finds GoogleTest when `MCA_BUILD_TESTS` is enabled; benchmarks are opt-in. It does not discover ROS 2 or Gazebo and therefore keeps the default core build isolated from middleware/simulator dependencies. `tests/simulation/step_test.cpp` is registered in `mca_core_tests`. The next implementation step should create separately built optional packages and add explicitly enabled ROS 2/Gazebo CI rather than adding those dependencies to root CMake. This audit is documentation/build-boundary evidence only; it does not claim ROS 2 or Gazebo runtime behavior.
