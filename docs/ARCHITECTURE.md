# Motion Control Platform Architecture

## 1. Core decision

Use a **layered + ports/adapters + composition** architecture.

Do not expose one giant `IRobotController` interface. That design couples unrelated domains and makes testing difficult. Instead, define small stable interfaces around capabilities:

- state estimation
- trajectory generation
- feedback control
- feedforward
- kinematics
- dynamics
- actuator/servo control
- optimization
- constraints
- command allocation

Robot-specific controllers compose these capabilities.

## 2. Dependency direction

```text
apps
  -> domain controllers
      -> algorithms
          -> models
              -> core

adapters (ROS2/Gazebo/hardware/logging) -> ports -> core/domain
```

Core code must not depend on ROS 2, Gazebo, vendor SDKs or filesystem/network APIs.

## 3. Stable data model

Use explicit value types rather than `Eigen::VectorXd` everywhere. Dynamic Eigen vectors remain available for generic optimization modules, but fixed-size vectors/matrices are preferred in hard real-time paths.

Representative concepts:

```cpp
struct Time {
    double seconds;
};

struct ScalarState {
    double position;
    double velocity;
    double acceleration;
};

struct JointState {
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd acceleration;
    Eigen::VectorXd effort;
};

struct JointCommand {
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd acceleration;
    Eigen::VectorXd effort;
};
```

Domain-specific strongly typed structures are added where semantics matter, e.g. `Pose3`, `Twist6`, `Wrench6`, `MotorDQ`, `VehicleState`, `FloatingBaseState`.

## 4. Controller interface

The primary abstraction should be capability-oriented:

```cpp
class IController {
public:
    virtual ~IController() = default;
    virtual bool configure(const ControllerConfig&) = 0;
    virtual bool reset(const ControlState&) = 0;
    virtual ControlCommand update(
        const ControlState& state,
        const ControlReference& reference,
        const ControlContext& context) = 0;
};
```

More specialized interfaces can refine this contract:

```text
IPositionController
IVelocityController
ITorqueController
ICartesianController
IOptimalController
```

Avoid making application code depend on concrete PID/LQR/MPC classes.

## 5. Algorithm composition

A production robot controller is assembled as a pipeline:

```text
Reference
  -> TrajectoryGenerator
  -> StateEstimator
  -> FeedbackController
  -> Feedforward
  -> Compensation
  -> ConstraintHandler
  -> CommandLimiter
  -> CommandAllocator
  -> Actuator Interface
```

Example manipulator:

```text
Quintic/S-curve trajectory
        -> PD
        + dynamics feedforward
        + friction compensation
        -> torque limit
        -> joint servo
```

Example wheel robot:

```text
Path planner
 -> Pure Pursuit / Stanley / LQR / MPC
 -> body velocity command
 -> differential/omni wheel kinematics
 -> wheel velocity controller
```

Example quadruped:

```text
Gait + foot trajectories
 -> state estimator
 -> centroidal/floating-base MPC
 -> WBC/QP
 -> joint impedance/torque controller
```

## 6. Real-time boundary

The platform separates three execution classes:

### RT-0: hard real-time

Typical 1–20 kHz. Motor current/FOC and servo loops. No ROS 2 executor, blocking I/O or uncontrolled heap allocation.

### RT-1: soft real-time

Typical 100–1000 Hz. Joint/vehicle/body controllers, estimators and trajectory tracking.

### RT-2: non-real-time

Typical 1–100 Hz. Planning, optimization with larger budgets, logging, visualization, parameter updates and ROS 2 orchestration.

## 7. Model interfaces

Models are separated from algorithms:

```text
IK              -> KinematicsModel
Jacobian        -> KinematicsModel
ComputedTorque  -> DynamicsModel
LQR             -> LinearizedModel
MPC             -> PredictiveModel
FOC             -> MotorModel
```

This permits manual reference implementations and later adapters to Pinocchio or other validated libraries without rewriting controllers.

## 8. Backend strategy

Initial reference implementations should be written manually with Eigen so that the mathematics is transparent and testable. Later, optional backends can be added:

- Pinocchio: rigid-body kinematics/dynamics
- OSQP / qpOASES / other QP solver: MPC/WBC
- ROS 2 control: hardware/control integration
- Gazebo Harmonic: simulation

Backend selection must occur outside the algorithm API.

## 9. Scenario abstraction

Each domain gets a scenario package rather than a separate framework:

```text
ActuatorScenario
WheelScenario
ManipulatorScenario
UAVScenario
LeggedScenario
FloatingBaseScenario
MultiRobotScenario
```

A scenario supplies model, state source, reference generator and controller composition. The same algorithm implementation can therefore be reused across scenarios.

## 10. Benchmark contract

Every controller exposes measurable results through a common benchmark layer:

- RMSE
- maximum absolute error
- overshoot
- settling time
- tracking delay
- control effort
- RMS current/torque
- constraint violations
- deadline misses
- computation time
- disturbance recovery time

Benchmark outputs must be machine-readable so that CI can detect regressions.

## 11. Configuration

Use versioned YAML/TOML configuration at the application layer. Core algorithms receive validated typed configuration objects. Algorithm implementations must not parse YAML directly.

## 12. Error handling

Configuration/model errors return explicit status values. Runtime control loops should avoid exceptions. Fatal initialization errors can use exceptions at the application boundary.

## 13. Numerical policy

- SI units internally.
- Radians, seconds, meters, kilograms, Newtons, Newton-meters.
- Explicit conversion at external boundaries.
- Angle wrapping utilities are centralized.
- Singularities and ill-conditioned matrices must be detected explicitly.
- Controller outputs are saturated before crossing actuator boundaries.

## 14. Testing pyramid

```text
Math/reference tests
      |
Algorithm unit tests
      |
Model/controller integration tests
      |
Scenario simulation tests
      |
Cross-controller benchmarks
      |
Hardware-in-the-loop (future)
```

Acceptance thresholds belong to each task and are stored with the benchmark configuration.
