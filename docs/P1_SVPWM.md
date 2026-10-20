# P1-10 SVPWM

## Scope

P1-10 adds a platform-independent space-vector PWM modulator for the PMSM/FOC control path:

`dq voltage -> inverse Park -> alpha/beta voltage -> SVPWM -> 3-phase duty`

The module is C++17 header-only and has no ROS 2, Gazebo, device-driver, or operating-system dependency.

## Conventions

The implementation reuses the project's Clarke/Park convention from `mca/transform/clarke_park.hpp`:

- `d = alpha*cos(theta) + beta*sin(theta)`
- `q = -alpha*sin(theta) + beta*cos(theta)`

SVPWM consumes the resulting stationary-frame `(alpha, beta)` voltage command.

## Algorithm

For DC bus voltage `Vdc`, PWM period `Ts`, requested vector magnitude `Vref`, and electrical angle `theta`:

1. Normalize `theta = atan2(beta, alpha)` to `[0, 2*pi)`.
2. Determine one of six 60-degree sectors.
3. Project the requested magnitude to the linear SVPWM limit:
   `Vlinear = Vdc / sqrt(3)`.
4. Within the selected sector calculate active-vector dwell times:

   `T1 = Ts * sqrt(3) * Vref/Vdc * sin(pi/3 - theta_s)`

   `T2 = Ts * sqrt(3) * Vref/Vdc * sin(theta_s)`

   `T0 = Ts - T1 - T2`

   where `theta_s` is the angle relative to the start of the sector.
5. Split the zero-vector time symmetrically as `T0/2` at both ends of the centered seven-segment sequence.
6. Convert the dwell times to three normalized duties in `[0, 1]`.

The six sector duty mappings are implemented explicitly so sector boundaries and phase ordering are deterministic.

## Linear modulation and overmodulation

The linear region is bounded by:

`|Vref| <= Vdc/sqrt(3)`

When a larger vector is requested, the implementation preserves the vector angle and clamps the magnitude to this boundary. `SvpwmOutput::overmodulated` reports whether this projection occurred. This deliberately avoids entering nonlinear six-step/overmodulation behavior in P1-10; that can be added later as a separate feature with explicit tests.

## API

- `SvpwmConfig`: DC bus voltage and PWM period.
- `SvpwmInput`: alpha/beta voltage command in volts.
- `SvpwmOutput`: phase duties, `T1/T2/T0`, requested/applied magnitude, sector, and overmodulation status.
- `Svpwm::compute(...)`: stateless modulation calculation.

A zero voltage vector returns 50% duty on all phases and the full PWM period as zero-vector time.

## Validation

The test suite covers:

- invalid configuration and non-finite input rejection;
- zero-vector behavior;
- all six sectors;
- sector boundaries and axis vectors;
- non-negative dwell times and `T1+T2+T0=Ts`;
- duty bounds;
- linear modulation limit and overmodulation reporting;
- reconstruction of the alpha/beta voltage vector from generated duties;
- negative-angle wrapping.

A Google Benchmark target measures the complete `compute()` path.
