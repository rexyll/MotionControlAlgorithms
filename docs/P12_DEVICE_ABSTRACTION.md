# P12-05b — Vendor-neutral servo-device abstraction and integration strategy

## Purpose

The current hardware boundary covers fixed-size joint state, effort commands, lifecycle and diagnostics. It has already been exercised through a two-joint ROS 2/Gazebo simulation, but real servo drives require additional concepts: multiple control modes, drive state machines, command freshness, clock synchronization, scaling, homing/brakes and safety status.

This design expands the abstraction incrementally without selecting a physical motor or claiming vendor compatibility. Initial validation is software-in-the-loop (SIL): deterministic fake drives, protocol fixtures/emulators, offline ESI/object-dictionary checks and SDK wrapper tests. Hardware-in-the-loop (HIL) remains a separate milestone.

## Layered architecture

Keep five boundaries distinct:

1. **MCA core hardware contracts** (include/mca/hardware): fixed-capacity values, capability interfaces, normalized units, error/fault values and command semantics. No OS, ROS, fieldbus, vendor headers, dynamic allocation or transport calls.
2. **Protocol/profile layer** (optional packages): CiA 301/CANopen, CiA 402 drive profile, EtherCAT CoE, PDO/SDO, NMT, heartbeat/EMCY, EtherCAT AL state, distributed clocks and process-data mapping.
3. **Transport/master layer** (optional packages): SocketCAN, SOEM, IgH EtherCAT Master or vendor communication library. Owns sockets, NIC access, bus scheduling, reconnect and transport diagnostics.
4. **Vendor adapter layer** (optional packages): model/firmware-specific object dictionaries, scaling, mode support, fault mapping, brake/encoder features, SDK calls and vendor extensions.
5. **Robot integration layer** (ROS 2 ros2_control, Gazebo, application): maps runtime robot topology and ROS interfaces to fixed-capacity MCA backends.

Dependency direction is one-way: application/ROS -> vendor adapter -> protocol/master -> transport. All adapters implement core capabilities. A vendor SDK adapter and a generic CiA 402 adapter are alternative backends, not dependencies of each other.

## Core capability model

Retain the existing small interfaces as composable capabilities. Do not create one monolithic IDevice with dozens of optional methods. Add capabilities only when multiple realistic backends or a concrete profile require them.

| Capability | Responsibility | Contract direction |
|---|---|---|
| Identity / topology | vendor, product, revision, serial, firmware, axis count and axis identity | fixed-size identifiers and bounded axis descriptors; no heap on cyclic path |
| Lifecycle | initialize, configure, activate, deactivate, shutdown | distinguish communication-ready, power-enabled and motion-enabled states; migrate without breaking current ports |
| State source | coherent state snapshot | retain validity mask; add monotonic timestamp, sample age/clock domain and per-axis validity |
| Command sink | submit setpoints | extend beyond effort to position, velocity and torque/current with explicit mode and units |
| Mode capability | discover/select supported control modes | report modes per axis and whether switching requires disabled state |
| Diagnostics | read/clear faults and counters | preserve raw vendor code, normalized category, severity, active/latched state and timestamp |
| Limits / configuration | query configured ranges and scaling | distinguish drive-enforced limits from application-side limits; change only while safe/inactive |
| Synchronization | report cycle, clock and sample alignment | expose requested/actual cycle, timestamp source, distributed-clock lock and synchronization quality |
| Homing / brake | optional axis functions | separate optional capabilities; never assume every drive has a brake or homing method |
| Safety status | report STO/SS1/SS2 and external interlocks when available | status/request is not a certified safety function; never replace hardwired safety |

Initial command modes are Position, Velocity, Torque and Current. Profile position/velocity/torque and cyclic synchronous position/velocity/torque (CSP/CSV/CST) have distinct timing semantics and must not be flattened into an ambiguous scalar. Current and torque are not interchangeable without explicit motor/drive conversion and calibration.

Use SI units at the MCA boundary: rad, rad/s, rad/s^2, N·m, A, V, degrees C and seconds. Preserve raw drive counts, encoder increments, vendor units and object values inside the adapter. Position wrapping, gear ratio, encoder direction, torque constant, current scaling and motor-side/load-side reference must be explicit configuration with provenance.

## Fixed-capacity and real-time rules

- Keep cyclic data in std::array-backed fixed-capacity structures. Runtime axis count is allowed only within a compile-time maximum and must be validated before activation.
- No allocation, string formatting, filesystem access, blocking logging, SDK discovery or object-dictionary lookup in cyclic read/write callbacks.
- Resolve PDO offsets, object indexes, scaling, handles and ROS interface pointers during configure/activate; cache immutable mappings.
- Define one coherent sample boundary for all axes. If the backend cannot provide an atomic multi-axis sample, expose per-axis timestamps/sequences and report incoherence instead of pretending it is synchronous.
- Commands carry a sequence and optional monotonic deadline. Reject stale, duplicate, out-of-order or expired commands before transport output. Sequence ownership survives deactivate/reactivate unless a new transport session explicitly resets it.
- State sequence and command sequence are separate domains.
- A multi-axis write states its commit semantics: atomic at drive, synchronized PDO frame, best-effort per axis, or potentially partial. Partial/unknown application is a control fault.
- Transport threads and cyclic threads need explicit ownership and bounded synchronization. Prefer single-writer cyclic ownership and bounded snapshots; no unbounded mutex in the control loop.
- Communication loss, watchdog expiry and drive faults inhibit future command submission and report the last confirmed output state. Software zero torque is not STO or a safety-rated stop.

## CiA 402 and protocol design

CiA 402 is a common drive profile, not a guarantee that all products implement identical optional features. Implement common behavior as a protocol-level state machine and capability discovery; keep vendor-specific objects and scaling in model profiles.

Initial common objects to evaluate:
- 0x6040 Controlword and 0x6041 Statusword for the PDS finite-state automaton;
- 0x6060 Modes of operation and 0x6061 Modes of operation display;
- 0x607A Target position / 0x6064 Position actual value;
- 0x60FF Target velocity / 0x606C Velocity actual value;
- 0x6071 Target torque / 0x6077 Torque actual value;
- 0x6072 Max torque and vendor-supported limit/configuration objects;
- PDO mapping and transmission type configured/validated before cyclic operation.

Do not assume every drive supports every mode/object or identical PDO defaults. At discovery/configure time validate identity, EDS/ESI and firmware profile, supported modes, scaling, PDO mapping, cycle limits, watchdog behavior and fault-reset semantics. SDO is for configuration/diagnostics; cyclic PDO is the normal process-data path. Preserve NMT state, heartbeat/EMCY and bus-off/recovery information for CANopen.

For EtherCAT CoE, keep EtherCAT master state (INIT/PRE-OP/SAFE-OP/OP), slave AL status, mailbox SDO, process-data mapping, working counter and distributed-clock health distinct from the CiA 402 drive state. An EtherCAT slave may be OP while its drive is not Operation Enabled. Never collapse these state machines.

### Offline ESI normalization boundary

The developer utility `scripts/ethercat_esi/esi_to_profile.py` converts an ESI XML file into a normalized JSON inventory using only Python's standard library. It is deliberately outside `include/mca/**`, the root C++ build, and all cyclic runtime paths.

```bash
python scripts/ethercat_esi/esi_to_profile.py vendor-device.xml --output vendor-device.profile.json
python scripts/ethercat_esi/esi_to_profile.py vendor-device.xml --product-code '#x00001001'
python -m unittest discover -s tests/ethercat_esi -v
```

The inventory preserves the vendor ID, each device's product code/revision, ordered SyncManager declarations, RxPDO/TxPDO declarations, PDO index, declared SyncManager association, Fixed/Mandatory flags, and each entry's index/sub-index/bit length/name/data type. Padding entries (index zero) are retained explicitly. The input file's SHA-256 is recorded so a fixture can be tied to the exact source bytes.

This is an **offline declaration parser**, not an ESI XSD validator, online CoE scanner, active PDO assignment readback, or proof that a drive supports a declared feature at runtime. ESI files may contain multiple devices. The current normalizer intentionally does not resolve modular-device slots, vendor-specific XML extensions, scaling, firmware compatibility, CoE object access attributes, or dynamic PDO remapping. It does not generate a production adapter or infer CiA 402 capability solely from a matching object index. Those checks remain separate gates.

Do not commit vendor ESI files until the exact model/firmware revision, official source URL, file hash, and redistribution terms are recorded and reviewed. Until then, synthetic fixtures exercise parser behavior; vendor-specific data must remain an explicitly sourced external input. A normalized PDO inventory is only a candidate input to a later generic CiA 402 mapping resolver, not an online configuration.

## Technology and vendor evaluation shortlist

This is a research shortlist, not a claim that adapters or SDKs are implemented.

| Candidate | Officially documented interface | Evaluation focus | Priority |
|---|---|---|---|
| maxon EPOS4 family | CiA 301/CiA 402 over CANopen; optional EtherCAT on supported variants; official EPOS Command Library for Windows/Linux and selected interfaces | SDK API/ABI and license, OS/CPU support, transport, mode coverage, call latency, reconnect and fault mapping | High |
| Synapticon SOMANET / ACTILINK | EtherCAT CoE + CiA 402; CSP/CSV/CST; ESI and vendor object dictionary; OBLAC Drives commissioning tools | generic CiA 402 interoperability, DC sync, PDOs, firmware objects, whether a distributable application SDK exists | High |
| Elmo Gold family | CANopen and EtherCAT variants; EASII commissioning and firmware tools | CiA 402 coverage, ESI/EDS, PDO map, vendor diagnostics and public runtime SDK availability | Medium |
| Beckhoff AX8000 | EtherCAT CoE with CiA 402; TwinCAT integration | ESI import, PDO mapping, modes, distributed clocks and TwinCAT integration boundary | Medium |
| Open-source EtherCAT masters | SOEM (portable lightweight master library) and IgH EtherCAT Master (Linux master) | portability, NIC access, cyclic scheduling, DC support, diagnostics, licensing and deployment | High |
| CANopen transport | Linux SocketCAN plus CiA 301/CiA 402 implementation | SDO expedited/segmented, PDO, NMT, heartbeat, EMCY, bus-off and reconnect | High |

Selection notes:
- Prefer one shared protocol/profile implementation across compatible vendors, with small vendor model profiles. Do not write four full drivers if devices expose interoperable CiA 402 behavior.
- maxon EPOS Command Library is a concrete SDK candidate, but vendor exchange-rate guidance reports typical function calls around 10–20 ms in evaluated USB/RS232/CAN interfaces. Treat it as commissioning/supervisory API until exact hardware measurements prove suitability for the required cyclic loop. Do not assume a 1 kHz hard-real-time API.
- SocketCAN is a Linux transport, not the CiA 402 abstraction. Keep transport, CANopen communication profile and CiA 402 drive profile separate.
- SOEM and IgH are EtherCAT masters, not vendor drive SDKs. SOEM is attractive for a portable prototype; IgH is a Linux-focused master with mature deployment documentation. Benchmark jitter, working-counter handling, DC behavior, recovery and NIC support before production selection.
- Synapticon, Elmo and Beckhoff public product documentation establishes protocol/profile support; it does not by itself establish a public redistributable C++ runtime SDK. Verify SDK availability, redistribution terms and firmware before adding an SDK-specific adapter.
- Do not select a final master/drive until target OS, NIC/CAN hardware, cycle time, axis count, safety requirements, budget and vendor availability are known.

Official references:
- CiA 402 profile and optional-feature caveat: https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control
- maxon EPOS4 controller interface selection and supported modes: https://support.maxongroup.com/hc/en-us/articles/360015382019-EPOS4-ESCON2-ESCON-What-controller-should-I-select
- maxon EPOS Command Library documentation: https://www.maxongroup.com/medias/sys_master/root/9157360353310/EPOS-Command-Library-En.pdf
- maxon Command Library exchange-rate guidance: https://support.maxongroup.com/hc/en-us/articles/360006122334-EPOS-Command-Library-Data-exchange-rates
- Synapticon CoE/CiA 402 integration: https://doc.synapticon.com/circulo_safe_motion/system_integration/coe_cia_402.html
- Synapticon firmware modes and distributed clocks: https://doc.synapticon.com/actilink_s/sw5.4/overview.html
- Elmo Gold Twitter CANopen/EtherCAT: https://www.elmomc.com/product/gold-twitter/
- Beckhoff AX8000 CoE/CiA 402: https://www.beckhoff.com/en-us/products/motion/servo-drives/ax8000-multi-axis-servo-system/ax8108-0100-0000.html
- SOEM master project: https://github.com/OpenEtherCATsociety/SOEM
- IgH EtherCAT Master documentation: https://etherlab.org/en_GB/documentation

## Validation gates

### Gate A — dependency-free core contracts
Add fixed-capacity axis descriptors, command modes/setpoints, timestamp/clock metadata, command deadlines, capability discovery and synchronization status. Test invalid enums, unsupported modes, non-finite values, out-of-range axes, expired/stale sequences and sequence overflow; verify output preservation on failure and absence of optional capabilities. Run full Linux/Windows root Configure/Build/Test. Root CMake remains free of ROS, CAN, EtherCAT and vendor SDK discovery.

### Gate B — deterministic drive-profile simulator
Implement a virtual CiA 402 drive with independent EtherCAT AL and CiA 402 PDS state machines. Exercise transitions, quick-stop/fault reaction/fault reset, mode negotiation, profile/cyclic modes, scaling, limits, PDO mapping, heartbeat/watchdog and injected communication loss. Test coherent and incoherent multi-axis samples and partial writes.

### Gate C — protocol SIL
CANopen: virtual CAN or deterministic transport fixture for NMT, heartbeat, EMCY, SDO, PDO, timeout and bus-off recovery.
EtherCAT: evaluate SOEM and IgH separately; use an EtherCAT slave simulator or deterministic master/slave fixture where feasible; verify ESI/PDO, AL state, working counter, DC lock/loss and reconnect. A protocol mock is not a physical bus test. Pin dependencies and licensing; optional builds cannot affect root build.

### Gate D — vendor SDK adapter SIL
**maxon EPOS Command Library is not currently cleared as a repository/CI dependency.** Official EPOS4 product documentation lists Windows DLLs and Linux shared-object libraries for x86/ARM, but the May 2026 Command Library license declaration applies a non-exclusive, non-transferable limited license for internal business use with eligible maxon products, restricts transfer/third-party use and reserves rights not expressly granted. The EPOS4 product page also lists the library and a separate license-declarations PDF. Do not commit, mirror, automatically download, or redistribute the proprietary package; do not add it to public CI or build a distributable adapter until maxon provides written confirmation covering the intended application-development, deployment, CI and redistribution model. A locally authored fake ABI test is not a substitute for that permission and must not embed proprietary headers/binaries. Official references: [EPOS4 70/15 product/software downloads](https://www.maxongroup.com/maxon/view/product/594385) and [EPOS Command Library License Declarations, edition 2026-05](https://www.maxongroup.com/medias/sys_master/root/9578789732382/EPOS-Command-Library-License-Declarations-En.pdf).

In the meantime, prefer vendor-neutral CiA 402 over CANopen/CoE using public protocol specifications and vendor EDS/ESI/object dictionaries. For Synapticon, validate SOMANET/ACTILINK CiA 402/CoE against its official documentation and ESI; for Elmo Gold and Beckhoff AX8000, use official product manuals/ESI and standard CoE behavior. Do not claim a vendor SDK adapter unless an official redistributable runtime SDK and its license are verified. Any eventual vendor SDK build remains opt-in/version-pinned; no proprietary binaries, credentials or license keys enter the repository.

### Gate E — hardware-in-the-loop
Requires selected drive/motor, bus interface, safe test fixture, independent STO/E-stop and configured motor/encoder. Verify real cycle jitter, state coherence, command freshness, watchdog, faults, stop, reconnect and thermal/current limits. Simulation or SDK compilation is not HIL.

## Immediate implementation order

1. P12-05b.0: accept this architecture and vendor/protocol decision matrix.
2. P12-05b.1: add capability and metadata values without breaking existing ports.
3. P12-05b.2: add mode-aware command values and safe command lifecycle semantics.
4. P12-05b.3: add deterministic CiA 402 drive-state/profile model and conformance tests.
5. P12-05c.1: add CANopen transport seam and virtual CiA 402 node tests.
6. P12-05c.2: compare SOEM and IgH through optional EtherCAT SIL builds and fixtures.
7. P12-05c.3: vendor SDK eligibility gate first. maxon EPOS Command Library is blocked pending written permission for the intended development/deployment/CI/distribution model under its 2026-05 EULA; until cleared, validate EPOS4 and other vendors through generic CiA 402/CoE plus official EDS/ESI/object dictionaries. Only then consider an opt-in fake-ABI SDK adapter.
8. P12-05d / P12-06: select hardware and perform HIL only when hardware is available.

This design document accepts architecture/research scope only. No physical device backend is accepted and existing P12-05a.4 remains simulation-only evidence.
