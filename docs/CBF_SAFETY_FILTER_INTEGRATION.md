# Discrete-Time CBF Safety-Filter Integration

The public umbrella header `mca/mca.hpp` exports four discrete-time CBF safety-filter variants:

- `DiscreteTimeCbfFilter`: nominal one-step barrier prediction.
- `RobustDiscreteTimeCbfFilter`: bounded additive prediction error.
- `CbfSlewRateFilter`: actuator magnitude and per-update slew limits.
- `RobustCbfSlewRateFilter`: robust prediction together with slew limits.

The integration test configures and runs all four variants through the umbrella header. It checks that zero uncertainty and inactive slew bounds reduce to the nominal result, and that infeasible constraints preserve the caller's output across all variants.

Build and run the suite using the repository's normal CMake test workflow on Linux or Windows.

## Numerical boundary behavior

The filters reject non-finite input data as invalid arguments and report `NumericalError` when finite inputs overflow during prediction-constraint arithmetic. Failed filtering is transactional: the caller-provided output is unchanged. Constraints within the configured feasibility tolerance may be accepted with a small negative reported margin; applications should choose the tolerance in the units and scale of their barrier model. The active-set projection is intended for small control dimensions and is not a general-purpose large-scale QP solver.

The integration regression suite also checks an exactly active barrier boundary across all four variants and verifies that arithmetic overflow produces `NumericalError` without modifying output.
