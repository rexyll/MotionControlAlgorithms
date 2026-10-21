# Floating-base state model

The P5-01 state type provides a dependency-light generalized state for floating-base and legged-robot algorithms.

## Conventions

- Base position and linear velocity are expressed in the world frame.
- The unit convention is SI; base orientation is a quaternion mapping body-frame vectors to world.
- Base angular velocity is expressed in the body frame.
- Joint positions and velocities use matching ordered coordinate arrays.
- This layer models state and validates it; it does not impose a particular joint topology or integrate dynamics.

## Validation

`validate_floating_base_state` rejects non-finite base position/velocity/angular velocity, a non-finite or zero-norm orientation quaternion, mismatched joint-vector sizes, and non-finite joint coordinates. A finite non-unit quaternion is accepted because callers may normalize at a dynamics/kinematics boundary; validation does not mutate the input. On failure it returns false and optionally provides an explanatory string.

## Example

```cpp
#include "mca/legged/floating_base_state.hpp"

mca::legged::FloatingBaseState state;
state.joint_positions = {0.1, -0.2};
state.joint_velocities = {0.0, 0.3};
if (!mca::legged::validate_floating_base_state(state)) {
  // Reject the state before passing it to a controller or model.
}
```
