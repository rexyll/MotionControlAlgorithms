# MotionControlAlgorithms Task Plan

> 2026-11-05 continuation checkpoint. Historical acceptance details remain in the individual design/acceptance documents and Git history; this file records the active milestone state so the plan stays maintainable.

## P8 — Advanced optimization/control

- [x] P8-39 callback exception safety
- [x] P8-40 projected-gradient underflow safety
- [x] P8-41 callback exception phase coverage
- [x] P8-42 representable finite differences
- [x] P8-43 representable line-search updates
- [x] P8-44 projected-gradient representability
- [x] P8-45 boundary-aware finite differences
- [x] P8-46 boundary evaluation efficiency
- [x] P8-47 permanent representability/boundary regression coverage

Acceptance evidence: Linux/Windows Configure/Build/Test and the dedicated P8-39 through P8-46 regression suites have remained green through the current main-line CI.

## P9 — Controller robustness and simulation validation

- [x] P9 controller/plant numerical transaction safety, checked diagnostics, power diagnostics and experiment rollback/status-propagation audits through P9-45.

Acceptance evidence: deterministic Linux/Windows CI coverage for the controller, plant, experiment and diagnostic contracts. No physical-device or safety-certification claim is implied.

## P10 — Simulation/runtime adapter contracts

- [x] Dependency-free ROS 2/Gazebo adapter contracts and deterministic in-process closed-loop acceptance.

## P11 — Concrete ROS 2 and Gazebo runtime integration

- [x] ROS 2 Jazzy adapter integration.
- [x] Gazebo Harmonic adapter and launch-level ROS 2 ↔ Gazebo integration.
- [x] Runtime robustness/relaunch/watchdog/reset acceptance.

Acceptance rule: root Configure/Build/Test remains independent of ROS 2 and Gazebo; middleware/simulator workflows are optional and version-pinned. Simulation is not HIL evidence.

## P12 — Physical hardware abstraction and device integration

- [x] P12-01 to P12-04b hardware boundary, dependency-free ports and ROS 2 control reference adapters.
- [x] P12-05a virtual N-axis drive/device simulation and catalog-backed motor profile.
- [x] P12-05a.4 live Gazebo Harmonic + ROS 2 Jazzy two-joint `ros2_control` simulation.
- [x] P12-05b.0 vendor-neutral servo abstraction and technology evaluation design.
- [x] P12-05b.1 fixed-capacity capability/timing metadata.
- [x] P12-05b.2 mode-aware position/velocity/torque/current command contracts.
- [x] P12-05b.3 CiA 402 PDS profile/state/controlword/mode model and independent EtherCAT AL state model.
- [x] P12-05c.1 CANopen SIL: transport seam, virtual CiA 402 node, NMT, heartbeat, EMCY, expedited/segmented SDO, PDO and optional Linux SocketCAN/vcan0 coverage.

### P12-05c.2 — EtherCAT SIL/master comparison

- [ ] Full EtherCAT SIL/master acceptance remains open; the KickCAT two-slave daisy-chain link break/heal baseline is now accepted, while target-NIC timing, broader topology/fixture coverage and physical-bus/HIL remain open.
- [x] Master-neutral fixed-capacity topology/PDO/WKC/DC/cycle-timing contracts and deterministic fake-master tests implemented.
- [x] Bounded asynchronous SDO request/result and mailbox queue lifecycle implemented.
- [x] IgH EtherCAT Master 1.6.13 adapter implemented behind `MCA_ENABLE_IGH`.
- [x] IgH 1.6.13 API compile passed.
- [x] Deterministic IgH fake-API coverage passed for cyclic exchange, topology/health, PDO mapping and asynchronous SDO lifecycle, including busy-slot isolation, timeout quarantine, terminal-buffer relocation and response-capacity checks.
- [x] IgH SyncManager/PDO/PDO-entry inspection distinguishes API read failures from malformed successful data; fake API fault injection covers fail-closed behavior and transactional discovery output.
- [x] KickCAT wire-level two-slave SIL baseline: CoE PDO assignment/entry readback, independent per-slave domains, DS402 Operation Enabled, downstream link isolation, link healing, IgH topology rescan recovery, 10 stable health cycles and safe return to Switch On Disabled. Accepted from operator run `/tmp/mca-ethercat-sil-20261105-132751`; see `P14_ETHERCAT_MULTI_SLAVE_HEALTH.md`.
- [ ] Broader slave-simulator/device-profile coverage, target-NIC timing/jitter, physical-bus fault behavior and physical-bus/HIL validation remain open.
- [ ] SOEM 2.0 integration remains blocked pending licensing/redistribution approval and separate dependency packaging; no SOEM dependency enters the root build.

### P12-05c.3 — Vendor SDK adapters

- [ ] maxon EPOS Command Library is blocked pending written licensing clarification for development, CI, deployment and redistribution.
- [ ] Until licensing is cleared, validate EPOS4, Synapticon SOMANET/ACTILINK, Elmo Gold and Beckhoff AX8000 through generic CiA 402/CoE plus official EDS/ESI/object dictionaries.
- [ ] Do not commit proprietary SDK binaries, headers, credentials or license keys.

### P12-05d — Concrete vendor device backend

- [ ] Select the actual drive/motor/bus/safety fixture and implement the model-specific backend only after hardware is available.

### P12-06 — Hardware-in-the-loop acceptance

- [ ] Validate real sensor coherence, cycle jitter, command freshness, watchdog, fault propagation, stop behavior, startup/shutdown and reconnect on selected hardware.
- [ ] Fake tests, API compilation and Gazebo simulation are not HIL substitutes.

## Architecture rules

1. Only `main` is used for development and acceptance.
2. `include/mca/**` remains free of vendor SDK, ROS 2, EtherCAT-master and transport-specific dependencies.
3. Cyclic control paths use fixed-capacity data and never block on SDO/mailbox work.
4. CiA 402 PDS state, EtherCAT AL state, master health, distributed-clock health and transport health remain separate state machines/signals.
5. SDO/mailbox operations are supervisory/asynchronous; cyclic PDO/process-data exchange remains the real-time path.
6. Every acceptance result explicitly distinguishes dependency-free contract tests, adapter/API tests, simulation, protocol SIL and real-device HIL.
7. CI failures must be diagnosed from the actual logs before any corrective change; no blind threshold lowering or skipped tests.

## Current next gate

1. IgH adapter audit: SyncManager/PDO/PDO-entry query failures have explicit RuntimeError handling and fail-closed regression coverage. PDO registration byte/bit offsets are range-checked before narrowing into the master-neutral layout. Fake-IgH per-registration offset injection covers same-domain unaligned field isolation and multi-domain image isolation; master-neutral layout tests cover partial overlap rejection, adjacent-range acceptance and declared-domain extent rejection. Domain image size is now checked after activation against the maximum registered PDO bit end, with undersized-image rejection, exact-boundary acceptance and cleanup/release coverage. Linux/Windows full CI #1982 (`42043936326`), #1983 (`42046118602`) and #1988 (`42048290506`) pass; IgH 1.6.13 API compile #434 (`42046118701`) and #439 (`42048290552`), Adapter State #442 (`42048290737`), PDO Image #428 (`42048290396`) and KickCAT Fault Patch #197 (`42048290584`) pass. These are fake-API/master-neutral tests, pinned API compilation and SIL harness checks, not physical-bus HIL. Unequal-size multi-Domain extent tests are now implemented and accepted: a later Domain undersized by one byte is rejected with master deactivation/release and Offline cleanup; independent 2-byte/3-byte Domain images are accepted and read back isolated values 2933 and 181. Latest full CI #1991 (`42054930391`) passes Linux and Windows; IgH 1.6.13 API compile #442 (`42054930282`), PDO Image #431 (`42054930232`), Adapter State #445 (`42054930245`) and KickCAT Fault Patch #200 (`42054930343`) pass. This evidence covers Fake-IgH API tests, pinned API compilation and KickCAT SIL only; it is not physical EtherCAT HIL. PDO image bit helpers now calculate per-bit absolute indices in uint64_t, avoiding uint32_t wraparound after the initial range check; required-byte bounds avoid multiplying byte_size by eight. Regression coverage verifies the final bit of a one-byte image, read/write round-trip, cross-boundary rejection without mutation, unchanged output on failed reads, and UINT32_MAX offset rejection. Latest full CI #1992 (`42057412686`) passes Linux and Windows; IgH 1.6.13 API compile #443 (`42057431840`), PDO Image #432 (`42057432122`) and KickCAT Fault Patch #201 (`42057433061`) pass. Evidence remains software tests, pinned API compilation and SIL; no physical EtherCAT HIL. Next audit all adapter-side conversions and Domain extent arithmetic (including signed IgH return values, bit-end calculation, required-byte rounding and per-direction image sizing), then add targeted regression tests only for any demonstrated gap.
2. Build generic CiA 402/CoE interoperability fixtures from public EDS/ESI/object-dictionary data for representative vendor profiles; keep vendor identity/scaling/fault extensions explicit and testable.
   - Offline ESI parser now supports explicit RxPDO/TxPDO selection and emits a CiA 402 candidate with object identity, PDO/SyncManager provenance, bit offsets and mode-pair capabilities. Synthetic tests cover velocity-only selection and malformed mappings; actual SDO assignment readback and vendor files remain separate pending gates.
   - [x] Offline ESI normalizer and explicit PDO-selection CiA 402 candidate resolver; synthetic parser/error fixtures committed.
   - [x] Python-to-C++ golden fixture parity for velocity-only and full CSP/CSV/CST PDO maps; CI validates object identity, direction, width, PDO/SyncManager order, padding, bit offsets and C++ adapter binding indices.
   - [ ] Vendor ESI files remain excluded pending exact firmware revision and redistribution review; actual CoE assignment/entry SDO readback remains a separate online acceptance gate. The normalized inventory is not an active PDO assignment or runtime capability claim.
3. Keep SOEM and proprietary vendor SDKs out of the root build until their licensing/package constraints are resolved.
4. When a real drive is selected, add the model-specific EDS/ESI/scaling/fault mapping and then perform HIL with an independent STO/E-stop fixture.
