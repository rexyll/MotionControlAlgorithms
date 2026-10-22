# Discrete-Time CBF Safety-Filter Integration

The public umbrella header `mca/mca.hpp` exports four discrete-time CBF safety-filter variants:

- `DiscreteTimeCbfFilter`: nominal one-step barrier prediction.
- `RobustDiscreteTimeCbfFilter`: bounded additive prediction error.
- `CbfSlewRateFilter`: actuator magnitude and per-update slew limits.
- `RobustCbfSlewRateFilter`: robust prediction together with slew limits.

The integration test configures and runs all four variants through the umbrella header. It checks that zero uncertainty and inactive slew bounds reduce to the nominal result, and that infeasible constraints preserve the caller's output across all variants.

Build and run the suite using the repository's normal CMake test workflow on Linux or Windows.
