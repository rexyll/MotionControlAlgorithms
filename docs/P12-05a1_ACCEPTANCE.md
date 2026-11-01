# P12-05a.1 Acceptance — catalog-backed motor simulation profile

Date: 2026-11-01

## Result

**Accepted for the defined software/profile scope.**

The optional maxon EC-i 40, 48 V, Hall-sensor, part 496653 profile records verified catalog values and requires explicit caller mapping for the existing uncoupled `BldcPmsm` model. The profile test validates catalog values, SI conversions, and acceptance of the explicitly supplied model configuration.

This acceptance does **not** claim physical motor validation, vendor SDK compatibility, communication compatibility, safety validation, or hardware-in-the-loop evidence.

## Evidence

- Root CI Run #1349: https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41299225537
- ROS 2 Jazzy Run #29: https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41299212847
- Code SHA: `6d9a07d1f802e01ecbe8e43d498d5074afc134b4`

Root CI passed Linux and Windows Configure/Build/Test and the retained P8-39 through P8-46 regression configure/build/test steps. ROS 2 Jazzy passed both build and test steps.

## Parameter provenance

Official maxon references:

- Product page: https://www.maxongroup.com/maxon/view/product/motor/ecmotor/EC-i/496653
- Datasheet: https://www.maxongroup.com/medias/sys_master/root/9344338165790/Datasheet-EC-i-303-EN.pdf

Catalog values are stored in SI units. Terminal resistance and inductance are phase-to-phase values; the code deliberately does not silently treat them as per-phase values. The test's `R_terminal/2` and `L_terminal/2` mapping is explicitly documented as an illustrative star-connected, zero-mutual-coupling assumption, while viscous damping is synthetic.

## Next scope

Proceed to an actual dynamic simulation scenario using the catalog profile, with startup, acceleration, load change, steady-state and parameter-boundary checks. Results must remain labeled as software simulation evidence until physical hardware is available.
