# mca_gazebo_adapter

Optional Gazebo Harmonic system plugin implementing the simulator-independent
`mca::simulation::IScalarGazeboSimulationAdapter` contract.

## Current scope

- Resolves one model joint by the SDF `joint_name` parameter.
- Caches a coherent scalar position/velocity snapshot in `PostUpdate`.
- Applies a finite scalar effort command through `JointForceCmd` in `PreUpdate`.
- Rejects invalid steps/commands and preserves caller output when state is unavailable.
- Resets command output to zero when no command for the expected step is pending.

Joint acceleration is currently reported as zero because the generic joint position
and velocity components do not provide a portable acceleration field. This is an
explicit limitation, not a measured acceleration.

## Build (Ubuntu 24.04 / Gazebo Harmonic)

Install the Gazebo Harmonic development packages, then:

```bash
cmake -S gazebo/mca_gazebo_adapter -B build/gazebo-adapter
cmake --build build/gazebo-adapter --parallel 2
```

The plugin is deliberately outside the root CMake project so core-only builds do
not require Gazebo. Load the shared library using Gazebo's plugin search path and
configure it with an SDF `<plugin>` element containing `<joint_name>...</joint_name>`.

## Evidence boundary

The package is under active development. Compilation is not yet acceptance of a
live Gazebo simulation. P11-04 will add a reproducible headless model and plugin
smoke test; P11-05 will cover reset, shutdown, missing joints and repeated launch.
The scalar ROS 2 adapter is a separate package; connecting ROS 2 commands to this
system and proving end-to-end state/command flow are P11-04 work.
