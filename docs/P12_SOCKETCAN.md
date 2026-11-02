# Optional Linux SocketCAN transport

## Scope

`mca::hardware::SocketCanTransport` is an optional Linux implementation of the existing `ICanopenTransport` seam. It is deliberately separate from `mca/mca.hpp` and is not part of the portable core API surface. The default build does not include Linux kernel headers or enable the feature.

This adapter provides standard 11-bit CAN frames only. Extended, RTR, error frames as application data, and payloads outside the CAN classic 8-byte contract are not forwarded as CANopen frames.

## Build

On Linux:

```sh
cmake -S . -B build-socketcan -DMCA_BUILD_TESTS=ON -DMCA_ENABLE_SOCKETCAN=ON
cmake --build build-socketcan --parallel
ctest --test-dir build-socketcan --output-on-failure
```

The option defaults to OFF. Enabling it on a non-Linux platform is a CMake configuration error. Consumers that include `mca/hardware/socketcan_transport.hpp` must enable `MCA_ENABLE_SOCKETCAN` and compile on Linux.

## Runtime behavior

Construct the transport with an existing SocketCAN interface name such as `can0` or `vcan0`. Check `initialization_status()` before use. An empty name, embedded NUL, or name too long for Linux `IFNAMSIZ` is rejected without opening a socket. Socket creation, interface lookup, error-filter setup, or bind failures return `RuntimeError`; inspect host diagnostics when initialization fails.

- Uses a non-blocking raw CAN socket.
- Sends and receives standard CAN identifiers and classic CAN payloads (0–8 bytes).
- An empty receive queue returns success with `has_frame=false`.
- CAN error notifications update transport state for Bus-Off and controller restart.
- While Bus-Off, sends are rejected; a restart notification returns state to Online.
- The adapter owns its file descriptor and is neither copyable nor movable.

## Validation boundary

The root Linux CI enables the option and runs the construction-validation test. It also attempts to provision `vcan0` (kernel module plus virtual CAN link) when the hosted runner permits it. When provisioned, a second test opens independent sender/receiver sockets and verifies non-blocking empty receive plus a complete classic-CAN frame round trip. If the runner lacks the required kernel module or network capabilities, that runtime test is explicitly skipped rather than reported as passed.

A successful `vcan0` round trip validates Linux socket binding and basic frame transport only. Root CI [Run #1477](https://github.com/rexyll/MotionControlAlgorithms/actions/runs/41432214810), SHA `bc8983ed90f3d2d4d8e9c962855532b21c5308a1`, provisioned `vcan0` and executed the round-trip test successfully as part of 662/662 passing Linux tests; it was not skipped. The Windows job and P8-39 through P8-46 regressions also passed. This does not emulate physical CAN controller error frames, electrical faults, arbitration under load, timing/jitter, vendor interoperability, physical hardware or HIL acceptance. Bus-Off/restart behavior still requires a controller or dedicated fault-injection fixture.
