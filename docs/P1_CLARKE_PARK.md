# P1-06 Clarke/Park Transforms

## Scope

P1-06 provides the reference-tested three-phase coordinate transforms needed by the later PMSM dq and FOC stages. The implementation is header-only, C++17, and has no ROS 2 or Gazebo dependency.

## Clarke transform

The implementation uses the **amplitude-invariant** Clarke convention:

```text
alpha = 2/3 * (a - b/2 - c/2)
beta  = 2/3 * (sqrt(3)/2) * (b - c)
zero  = (a + b + c) / 3
```

For a balanced sinusoidal set

```text
a = cos(theta)
b = cos(theta - 2*pi/3)
c = cos(theta + 2*pi/3)
```

the transform produces

```text
alpha = cos(theta)
beta  = sin(theta)
zero  = 0
```

The inverse is

```text
a = alpha + zero
b = -alpha/2 + sqrt(3)*beta/2 + zero
c = -alpha/2 - sqrt(3)*beta/2 + zero
```

The zero-sequence term is retained explicitly so arbitrary abc vectors round-trip without information loss.

## Park transform

The stationary alpha-beta vector is rotated into the rotor dq frame using electrical angle `theta`:

```text
d = alpha*cos(theta) + beta*sin(theta)
q = -alpha*sin(theta) + beta*cos(theta)
```

The inverse is

```text
alpha = d*cos(theta) - q*sin(theta)
beta  = d*sin(theta) + q*cos(theta)
```

With this convention, a vector `[cos(theta), sin(theta)]` is aligned with the d axis and therefore becomes `(d,q) = (1,0)`. A stationary alpha-axis vector at `theta = +pi/2` becomes `(0,-1)`.

## Validation

`tests/transform/clarke_park_test.cpp` validates:

- balanced three-phase abc to alpha-beta-zero conversion;
- arbitrary abc round-trip including zero sequence;
- rotating balanced vectors at several electrical angles;
- Park alignment with the rotor frame;
- Park/inverse-Park round-trip;
- explicit positive-angle/sign convention.

The tests use tight floating-point tolerances and are registered in the common CMake test target, so Linux and Windows CI exercise the same reference cases.

## Design boundary

These transforms are coordinate transformations only. They do not implement motor dynamics, current control, decoupling, SVPWM, or inverter switching. P1-07 will build the PMSM dq plant model on this convention, and P1-08/P1-09 will use the same convention for FOC current control and feedforward/decoupling.
