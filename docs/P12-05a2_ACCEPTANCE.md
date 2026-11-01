# P12-05a.2 Acceptance — fault-injectable virtual-device diagnostics

Date: 2026-11-01

## Result

**Accepted for deterministic software/simulation scope only.**

The generic diagnostics capability and fault-injectable virtual-joint decorator
are implemented and verified. This is not a physical drive fault decoder,
watchdog test, safety validation, or hardware-in-the-loop result.

## Implementation

- `IHardwareDiagnostics` exposes fixed-capacity fault snapshots and explicit
  clearing without coupling the core HAL to a vendor SDK.
- Each `HardwareFault` carries a normalized category, opaque vendor raw code,
  active/latched flags, sequence and caller-provided timestamp.
- `FaultInjectableVirtualJointDevice<N>` composes the virtual plant and the
  diagnostics interface.
- Fault injection requests software safe-stop, inhibits effort writes and
  simulation stepping, and exposes faulted lifecycle state.
- Multiple independent fault categories can coexist. A duplicate active
  category is rejected.
- Recovery requires explicit resolution of all active faults, clearing the
  retained records, then reactivation.
- State reads remain available while faulted for post-stop inspection.
- Clearing an empty fault set is a no-op and does not disrupt a healthy active
  device.

## Test evidence

Focused tests cover:
1. Empty clear does not deactivate a healthy device.
2. Fault injection immediately zeroes applied effort and blocks commands.
3. Vendor raw code and active/latched semantics are retained.
4. Active faults cannot be cleared; explicit resolve/clear/reactivation works.
5. Multiple categories coexist and duplicate active categories are rejected.

Root CI on code SHA
`b0c5d85c6ebe86bacaf5c625db7cda9f30ac2d1f`:
- [Run #1356](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41313455621)
- Linux: 628/628 core tests passed.
- Windows: 628/628 core tests passed.
- P8-39 through P8-46 regression suites passed on both platforms.

ROS 2 Jazzy:
- [Run #32](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41313455557)
- Both mock hardware plugin tests and the scalar adapter test passed.

## Limitations

This is deterministic software fault injection only. It does not establish
physical watchdog latency, hardware fault-code decoding, independent safety
circuit behavior, real-time transport guarantees, drive disable/brake timing,
physical plant response, or HIL acceptance.

P12-05 (concrete vendor device backend) and P12-06 (physical HIL acceptance)
remain open pending selection and access to a real target device.
