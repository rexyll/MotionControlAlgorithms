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

The root Linux CI enables the option and compiles/runs a focused construction-validation test. It does not create a `vcan` interface, require `CAP_NET_ADMIN`, or claim live bus traffic, controller recovery, timing/jitter, vendor interoperability, physical hardware, or HIL acceptance. Those require a privileged Linux fixture or selected physical CAN interface and remain later validation work.
