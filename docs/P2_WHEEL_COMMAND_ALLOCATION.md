# P2-08 Wheel command allocation

## Scope

This utility applies actuator-level absolute wheel-speed limits to a requested wheel-command vector. It is a small, allocation-free, header-only component and has no ROS 2, Gazebo, hardware, or controller dependency.

## Method

Given requested wheel commands u_i and positive per-wheel limits m_i, compute the largest common scale factor:

```text
alpha = min(1, min(m_i / abs(u_i) for each wheel where abs(u_i) > m_i))
allocated_i = alpha * requested_i
```

A common factor preserves the vector's ratios and avoids independently clipping wheels in a way that would change the requested chassis motion. Per-wheel limits can differ.

## API

WheelCommandAllocator<N>::allocate(requested, absolute_limits, allocated, scale_factor) supports any compile-time wheel count N > 0.

- Rejects non-finite commands, non-finite limits, and non-positive limits.
- Does not modify outputs on invalid input.
- Uses fixed-size std::array values; no dynamic allocation.
- Returns a scale factor in [0, 1]; 1 means no limiting was required.

This is proportional wheel-speed limiting, not a general constrained optimizer. It does not solve for a different feasible chassis twist when independent wheel constraints conflict.

## Validation

Unit tests cover unchanged commands, heterogeneous limits, proportional scaling, zero commands, invalid inputs and transactional failure behavior. A deterministic four-wheel micro-benchmark is built when Google Benchmark is available.
