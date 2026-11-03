# P12-05c.2 — EtherCAT master evaluation and SIL boundary

## Decision summary

Do not couple EtherCAT master APIs to `include/mca/**`, and do not choose a
single master as the universal backend. Define a small MCA-owned EtherCAT
master seam and keep SOEM and IgH as separate optional adapters.

- **SOEM** is the preferred first integration candidate for a portable
  development/SIL adapter because its upstream project describes it as a
  library for developing EtherCAT MainDevices and explicitly supports Linux
  and Windows. The application owns its cycle thread, process-data exchange,
  scheduling policy and recovery orchestration.
- **IgH EtherCAT Master** is the preferred Linux deployment candidate where
  its kernel-module/device-driver architecture, domain model and real-time
  deployment options match the target. It provides a common application
  interface, process-data domains, explicit domain working-counter state,
  SDO configuration and distributed-clock configuration.
- Neither choice is a safety function or evidence of hard-real-time behavior
  on an arbitrary host/NIC. Actual cycle jitter, NIC support, DC quality,
  watchdog and recovery must be measured on the target machine and slaves.

This is an architecture/evaluation decision, not a production master selection.
No EtherCAT physical bus, slave, HIL or timing result is claimed.

## Master comparison

| Concern | SOEM | IgH EtherCAT Master | MCA consequence |
|---|---|---|---|
| Integration shape | User-space C library; application owns master loop and scheduling | Linux master module plus application interface/library; master/device drivers and application are separate | Two optional adapters implement one MCA-owned contract |
| OS scope | Upstream project documents Linux and Windows support | Linux-focused kernel/module deployment | Keep SOEM candidate for portable SIL/dev; IgH candidate for Linux deployment |
| NIC access | Opens/uses a selected network adapter for EtherCAT frames; permissions and adapter support are host-specific | Native EtherCAT-capable NIC drivers plus generic driver path; driver compatibility is deployment-critical | Never auto-select a production NIC; require explicit interface and ownership checks |
| Process data | Maps slave PDOs into an IOmap; application sends/receives process data and interprets returned WKC | Domains group process data and can use different task periods; master computes mapping and domain state | MCA exposes immutable bit/byte mappings and per-domain cycle/WKC health |
| CoE mailbox | SOEM provides CoE/SDO services in its stack | Application interface exposes SDO configuration/request facilities | SDO is configure/diagnostic path, never cyclic command path |
| AL state / recovery | SOEM exposes slave state and state-check/recovery operations; application orchestrates policy | Master/slave state and application control are exposed via its API/tools | Preserve raw AL state + AL status code and make recovery asynchronous/observable |
| Working Counter | Process-data receive path returns WKC; application compares it with expected topology-dependent WKC | Domain state reports working-counter state (zero/incomplete/complete) | Store expected and observed WKC per domain/cycle; incomplete is a control fault, not a warning-only metric |
| Distributed clocks | Supports DC configuration/time synchronization and SYNC0/SYNC1 configuration | Explicit DC configuration and application-time/reference-clock synchronization APIs | Report DC capability, configured cycle/shift, reference-clock status and measured deviation separately |
| Real-time ownership | Application owns cyclic thread and deadline discipline | Master supports cyclic application use; native drivers and RT extensions are deployment options | MCA must own cycle deadline/missed-cycle counters; master API alone does not guarantee deterministic timing |
| Recovery | Application handles state polling and slave recovery policy | Master FSM and application interface provide state/control facilities | Recovery must not block cyclic read/write; expose Recovering and require revalidation before motion |
| Portability / deployment | Stronger for cross-platform integration; still needs raw NIC access and host-specific setup | Linux deployment model with kernel module/device drivers | Keep optional targets out of root portable build |
| License / redistribution | Verify exact upstream LICENSE and obligations for pinned revision | Verify exact EtherLab release/license and obligations | Legal review is a release gate; do not infer license compatibility from API availability |

## Recommended layering

```text
ROS 2 / robot application
        |
MCA drive adapter (CiA 402 + vendor profile)
        |
MCA EtherCAT contract (master-neutral)
        |
  +-----+------------------+
  |                        |
SOEM adapter           IgH adapter
  |                        |
SOEM library           IgH application API
  |                        |
host NIC access         IgH device driver / NIC
```

The EtherCAT contract must not expose `ec_master_t`, `ec_domain_t`,
`ec_slave_config_t`, SOEM `ecx_contextt`, raw IOmap pointers, or vendor
structures. Those remain private to each adapter.

## Contract boundary to implement

Keep the first interface deliberately small and fixed-capacity:

- **Topology snapshot:** bounded slave count; station position; vendor ID,
  product code, revision, serial where available; AL state and AL status code.
- **Process-data layout:** bounded domains; each entry carries object index,
  subindex, bit length, direction, bit offset and mapping provenance. Reject
  overlap, overflow, unsupported bit lengths and layouts beyond configured
  capacity before activation.
- **Cyclic exchange:** submit output image and acquire input image using
  caller-owned fixed buffers; return transport status, observed WKC, expected
  WKC, cycle sequence and monotonic timestamp. Never allocate or format strings
  in the cyclic path.
- **Clock health:** distinguish unsupported, not configured, synchronizing,
  locked, lost and degraded; include application/reference time and signed
  phase deviation when the backend can measure it. Do not turn a boolean DC
  flag into a precision claim.
- **Lifecycle/recovery:** explicit Offline, Discovered, Configured, Active,
  Recovering and Fault states. AL state remains per slave and independent of
  CiA 402 PDS state.
- **Mailbox operations:** bounded asynchronous SDO request/result objects.
  A cyclic thread must not wait on mailbox completion.
- **Diagnostics:** raw master/slave error, AL status code, expected/observed
  WKC, missed cycle count, last successful exchange and recovery attempts.

Do not place master-specific APIs in the core header. Adapter callbacks must
validate output/input capacities and topology before touching caller buffers;
failed reads preserve caller output. Define whether an output exchange may
have been partially transmitted when a timeout occurs.

## SIL strategy and acceptance gates

### Gate 1 — master-neutral contract and deterministic fake master

No EtherCAT NIC, kernel module, root permission, ROS or vendor SDK required.
Use a deterministic fake master to test:

1. bounded topology discovery and identity mismatch;
2. PDO bit mapping including non-byte-aligned entries, overflow, overlap,
   input/output direction and capacity rejection;
3. AL INIT → PRE-OP → SAFE-OP → OP progression, illegal transitions,
   AL error status and independent CiA 402 PDS state;
4. expected WKC calculation and observed WKC states: zero, incomplete,
   complete, unexpected/over-count;
5. DC unsupported/unconfigured/locked/lost/degraded transitions and clock
   domain mismatch;
6. cycle sequence, deadline expiry, missed-cycle detection and output
   preservation on failed input;
7. slave loss, timeout, recovery attempts, re-discovery and revalidation
   before returning to Active;
8. asynchronous SDO completion, timeout, abort and mailbox congestion without
   blocking the cyclic path.

This validates MCA semantics and deterministic adapter behavior. It is not a
wire-level EtherCAT simulation.

### Gate 1 recovery and topology-revalidation rules

The portable contract now distinguishes per-slave health from the aggregate
master lifecycle:

- Health transitions are explicit: Unknown -> Present/Lost/Fault,
  Present -> Lost/Fault, Lost -> Recovering/Fault,
  Recovering -> Present/Lost/Fault, and Fault -> Recovering.
  Lost or Fault cannot jump directly to Present.
- A health snapshot is fixed-capacity, sequence-tagged, non-empty and rejects
  duplicate station positions or Unknown entries.
- A rediscovered topology is compared with the configured topology by
  station position plus vendor ID, product code, revision and serial number.
  Snapshot sequence and current AL state/status are not identity fields.
  Entry ordering in the returned snapshot is irrelevant; station positions
  are matched explicitly.
- Any missing/extra station or identity-field change rejects reuse of the
  previous PDO layout. The adapter must leave cyclic operation inactive and
  require explicit reconfiguration. An unchanged identity set permits the
  adapter to proceed with its revalidation/configuration policy; it does not
  by itself prove that the slave is Operational.
- Aggregate WKC loss is not treated as proof of which slave disappeared.
  Per-slave presence and identity must come from topology/AL observations.

These are master-neutral policy/value contracts, not a wire-level recovery algorithm. The fixed-capacity mailbox queue lifecycle primitive and its deterministic timeout/backpressure tests are now implemented; root CI Run #1519 passed Linux and Windows plus the P8 regression suites. The dependency-free Gate 1 contract is validated, but it does not claim a real SDO backend, wire-level EtherCAT simulation or physical bus recovery. Master adapters remain responsible for bounded polling, protocol requests, cancellation and protocol-specific timeout/error handling.

### Asynchronous SDO/mailbox contract

`EthercatSdoRequest<MaxBytes>` and `EthercatSdoResult<MaxBytes>` are fixed-capacity values. Requests carry a unique nonzero request ID, slave position, CoE object index/subindex, read/write operation, bounded payload or response capacity, and an absolute deadline. Results distinguish Pending, Completed, Aborted (with a nonzero CoE abort code), TimedOut and Failed.

`IEthercatMailbox<MaxBytes>` exposes only nonblocking enqueue/poll operations. The reusable `EthercatMailboxQueue<QueueCapacity, MaxBytes>` primitive provides fixed-capacity storage, duplicate-live-ID rejection, caller-supplied monotonic-time deadline checks, timeout terminal state, late-completion rejection, explicit terminal retirement and immediate queue-full backpressure. It has no clock, allocation, lock or blocking operation. The adapter must call it from a suitable scheduling context and still implement actual CoE/SDO transport, segmented transfer progression, protocol-specific abort handling and safe backend cancellation. Queue-full is a runtime/backpressure condition, not permission to block the caller. The cyclic `exchange()` API does not accept mailbox work and must never wait for an SDO result.

### Gate 2 — SOEM optional adapter

**Upstream candidate pinned for evaluation (not yet approved for redistribution):** `OpenEtherCATsociety/SOEM` commit `49fc34923b973c17700592319159af22157350c9` (upstream `master`, commit dated 2026-10-14; CMake project version 2.0.0). Pinning this immutable SHA makes API/build investigation reproducible; it is not a claim that the dependency is license-cleared or production-selected.

**License is a blocking release decision.** The pinned upstream `LICENSE.md` states dual licensing under GPLv3 and a commercial license, and explicitly warns that commercial products likely need a purchased license. The GitHub repository metadata reports `NOASSERTION`, so it must not be treated as permissive. This repository currently has no root `LICENSE`/`LICENSE.md` file to establish an obvious GPL-compatible project license. Do not vendor SOEM, redistribute binaries, or add an automatic FetchContent dependency until the project owner confirms either (a) a compatible open-source licensing path after legal review, or (b) a commercial SOEM license. This is a licensing gate, not a technical assumption.

**Build integration constraints:** SOEM 2.0.0's top-level CMake requires 3.28, while MCA root CMake requires 3.20. Do not raise the root minimum or FetchContent SOEM into the default build. Prefer a separately installed/pinned SOEM build and an opt-in adapter package that consumes its exported CMake target. Upstream Linux integration links `pthread` and `rt`; upstream Windows integration links bundled WinPcap `wpcap.lib`/`Packet.lib` plus `ws2_32` and `winmm`, so Windows CI needs a documented/approved packet-capture driver/runtime strategy. Raw NIC access and privileges must be explicit on both OSes.

**Mailbox/cycle isolation is a separate technical gate.** At the pinned revision, the public `ecx_SDOread()`/`ecx_SDOwrite()` implementation calls `ecx_mbxsend()` and `ecx_mbxreceive()` with timeout-driven polling/sleep loops; these are synchronous calls, not an asynchronous SDO API. SOEM also exposes `ecx_mbxhandler()` and a bounded per-group mailbox transmit queue, but the inspected queue path handles mailbox frame transmission and does not by itself turn the CoE SDO upload/download state machine into a nonblocking request/result API. Therefore the adapter must not call `ecx_SDOread/write` from the cyclic `exchange()` path. Before implementing the mailbox bridge, prove a safe single-owner scheduling design or implement a bounded CoE SDO state machine over SOEM's mailbox primitives; do not assume concurrent access to one `ecx_contextt`/port is thread-safe. A worker thread is not automatically safe if it races the cyclic process-data calls on the same context.

After license, dependency packaging and mailbox/cycle isolation are cleared, start with discovery, topology identity, PDO map extraction, process-data exchange, expected/observed WKC, AL state polling, DC configuration/health and controlled recovery. Keep SOEM headers, context and IOmap private to the adapter. Test against an EtherCAT slave simulator or dedicated virtual fixture if available; otherwise report only compile/API integration, not EtherCAT protocol SIL.

### Gate 3 — IgH optional adapter

**Current candidate:** official EtherLab EtherCAT Master **1.6.13** release, immutable commit `61cc654f5b721ddd54df0f58bdd34106d91c5359` (tag `1.6.13`, dated 2026-09-15) on `stable-1.6`. The official project recommends `stable-1.6` for production systems; 1.6.13 is newer than the earlier 1.6.8 candidate. Pin the full commit SHA, not the moving branch. This is a Linux-only kernel-module/userspace-library architecture, not a Windows alternative.

**License split (must be reviewed for the actual distribution model):** the master/kernel-side code is GPLv2; the official manual states the userspace application library `libethercat` is LGPLv2.1 to permit static linking of userspace applications. This is materially different from SOEM 2.0's GPLv3/commercial dual license, and makes IgH worth evaluating for a Linux robot controller with a proprietary userspace application. Do not infer that the complete deployment is automatically cleared: kernel module packaging/modification, static vs dynamic linking, notices/source obligations, and any EtherCAT technology/trademark requirements need project/legal review.

**Technical fit:** official EtherLab documentation exposes `ecrt_sdo_request_read()` / `ecrt_sdo_request_write()` to schedule transfers, `ecrt_sdo_request_state()` to poll state, `ecrt_sdo_request_timeout()` for request deadlines, and data/data-size accessors. The API marks these operations as usable in realtime context after master activation, so this is a substantially better fit for the mailbox contract than SOEM's synchronous `ecx_SDOread/write`. Keep one IgH request object reserved per active MCA request; do not reissue it while `EC_REQUEST_BUSY`. Critically, `ecrt_sdo_request_data()` may return an invalid pointer while a read is busy because the internal buffer can be reallocated; copy response bytes only after the request reaches a terminal successful state, and verify the reported data size against the caller's maximum. The documented timeout marks a request failed, but the public API pages do not establish a general immediate-cancel operation; treat cancellation/release of a busy request as unresolved until source-level verification and a targeted test. Polling SDO state must not cause the cyclic process-data path to wait for completion.

Pin an exact 1.6.x release/commit and build only in an opt-in Linux environment. Validate master request/release, domain registration, process-data mapping, domain WKC state, SDO configuration/request, AL state, DC application-time calls and recovery. Kernel module, kernel version, NIC-driver and real-time scheduling compatibility must be recorded. A successful compile is not runtime bus validation.

### Gate 4 — target deployment / HIL

Only after the target host, NIC, slave and safe fixture are selected: measure
cycle-period distribution, worst-case jitter, missed deadlines, WKC anomalies,
DC phase deviation, watchdog reaction and reconnect behavior under induced
faults. Validate STO/E-stop independently. This gate cannot be passed by a
fake master or a simulator-only CI job.

## Initial implementation sequence

1. [x] Add dependency-free fixed-capacity EtherCAT value types and the master
   interface under `include/mca/hardware`; export only those portable types.
2. [x] Add deterministic fake master + tests for Gate 1, including negative
   mapping/lifecycle cases. Root CI acceptance is still pending.
3. [x] Add the optional IgH 1.6.13 userspace adapter behind `MCA_ENABLE_IGH`,
   with pinned API compile CI; runtime kernel/NIC acceptance remains separate.
4. Evaluate SOEM 2.0 only if its GPLv3/commercial license path and
   Windows WinPcap deployment requirements are explicitly cleared.
5. Compare both adapters against the same contract and fixture before choosing
   a production deployment path.

## Official references

- SOEM upstream project and platform/library scope:
  https://github.com/OpenEtherCATsociety/SOEM
- Open EtherCAT Society SOEM/SOES project and supported profiles:
  https://openethercatsociety.github.io/
- SOEM reference manuals:
  https://docs.rt-labs.com/soem
- IgH EtherCAT Master feature summary and deployment architecture:
  https://docs.etherlab.org/ethercat/1.6/pdf/ethercat_doc.pdf
- IgH application interface (domain WKC, AL state, PDO registration, SDO and DC):
  https://docs.etherlab.org/ethercat/1.6/doxygen/group__ApplicationInterface.html
- IgH application interface, current 1.7 reference:
  https://docs.etherlab.org/ethercat/1.7/doxygen/group__ApplicationInterface.html

## Open questions deliberately deferred

- Production master choice depends on target Linux distribution/kernel, NIC
  chipset/driver, required cycle time, axis count, slave topology and support
  commitment. No user hardware choice is assumed.
- Exact license and redistribution obligations must be checked against pinned
  upstream release files before adding either dependency. For SOEM candidate
  commit `49fc34923b973c17700592319159af22157350c9`, the upstream license is
  GPLv3/commercial dual licensing; project-owner/legal approval and a selected
  licensing path are mandatory before adapter integration or redistribution.
- Real WKC/DC/recovery quality requires a real slave or a validated EtherCAT
  simulator; fake-master results must be labeled separately.

## Gate 2A — IgH ↔ KickCAT wire-level SIL feasibility

### Finding (2026-11-03)

KickCAT is technically suitable for a first external-slave SIL experiment with
the existing IgH adapter, without linking KickCAT into MCA:

- KickCAT's `network_simulator` is a separate process and its Linux
  `Socket` implementation opens `PF_PACKET/SOCK_RAW`, binds EtherCAT
  EtherType `0x88A4` to a named network interface, and reads/writes raw
  Ethernet frames.
- IgH's `ec_generic` driver also uses a kernel `PF_PACKET/SOCK_RAW` socket
  bound to its selected Linux interface. The IgH generic driver documentation
  describes it as operating on ordinary registered Ethernet interfaces.
- Therefore a Linux `veth` pair is a plausible transport bridge:
  IgH master + `ec_generic` on one endpoint, KickCAT
  `network_simulator` on the peer endpoint. This is a real EtherCAT frame
  exchange between two independent processes, not an in-process fake API.
- KickCAT's simulator can build its slave EEPROM and CoE dictionary from ESI
  XML, has an optional CiA-402 motor device application, and supports SDO,
  PDO, AL state transitions and network wire-break/heal injection.
- KickCAT's current feature matrix still lists DC synchronization as
  experimental and does not emulate interrupts. This SIL must not be used to
  claim DC timing accuracy, interrupt behavior, hard-real-time performance or
  conformance certification.

Evaluated KickCAT source snapshot: `leducp/KickCAT@cb2e1a192ac11ffaa58f08945053b8129abf94cb` (upstream master HEAD observed 2026-11-03; pin this immutable SHA in the harness, do not follow master at runtime).

Upstream references:
- KickCAT: https://github.com/leducp/KickCAT/tree/cb2e1a192ac11ffaa58f08945053b8129abf94cb
- Simulation guide: https://github.com/leducp/KickCAT/blob/master/docs/SIMULATION.md
- Feature matrix: https://github.com/leducp/KickCAT/blob/master/docs/FEATURES.md
- Linux raw socket: `lib/src/OS/Linux/Socket.cc`
- IgH generic driver: https://gitlab.com/etherlab.org/ethercat/-/blob/stable-1.6/devices/generic.c
- IgH device driver documentation: https://docs.etherlab.org/ethercat/1.6/pdf/ethercat_doc.pdf

### Proposed isolated topology

```text
MCA IgH adapter
      |
IgH userspace API (ecrt)
      |
IgH ec_master + ec_generic
      |
veth-mca (AF_PACKET / EtherCAT 0x88A4)
      |
Linux veth peer
      |
veth-kickcat (AF_PACKET / EtherCAT 0x88A4)
      |
KickCAT network_simulator
      |
EmulatedNetwork + EmulatedESC + CoE/CiA402 device app
```

The two endpoints must be dedicated to this test and must not carry host IP
traffic. Create the pair directly with `ip link add mca-ecatA type veth peer
name mca-ecatB` (KickCAT's helper script is not required by the harness), bring
both endpoints up, and record their MAC addresses. Configure the IgH master to
select only the MCA endpoint, and verify that KickCAT is bound only to the
peer. The harness must clean up the pair in a trap even on failure. Never point IgH at a host management NIC.
The test requires Linux root/CAP_NET_RAW/CAP_NET_ADMIN and permission to load
the IgH kernel modules; it is not a normal unprivileged root-CI test.

### Executable transport/discovery harness

An opt-in harness is now available at
`scripts/ethercat_sil/run_kickcat_igh_veth_discovery.sh`. It deliberately
stops at the first real wire-level gate: one-slave discovery plus captured
master-to-slave and slave-to-master EtherCAT frames. It does not configure
PDOs or claim an MCA cyclic adapter test.

Build KickCAT at the pinned revision in an isolated checkout, then run:

```bash
git clone https://github.com/leducp/KickCAT.git /opt/KickCAT
git -C /opt/KickCAT checkout cb2e1a192ac11ffaa58f08945053b8129abf94cb
cd /opt/KickCAT
./scripts/configure.sh build
./scripts/setup_build.sh build
cmake --build build --target network_simulator -j"$(nproc)"

cd /path/to/MotionControlAlgorithms
sudo -E env \
  KICKCAT_SIMULATOR=/opt/KickCAT/build/simulation/network_simulator \
  KICKCAT_SLAVE_CONFIG=/opt/KickCAT/simulation/slave_configs/ecat402-drive.json \
  bash scripts/ethercat_sil/run_kickcat_igh_veth_discovery.sh
```

Prerequisites: IgH `ec_master` and `ec_generic` modules built for the
running kernel, the matching `ethercat` CLI, `iproute2`, `tcpdump`,
`kmod`, and root privileges. Run only on a disposable Linux VM/host where
loading/unloading IgH modules is acceptable. The script refuses to run if
either IgH module or either veth interface already exists; it does not unload
or reconfigure an existing master. It selects the IgH master by the dedicated
veth endpoint's MAC, starts KickCAT on the peer, captures one outbound and one
inbound EtherCAT frame, and verifies the simulator ESI identity:
Vendor ID `0x0000029c`, ProductCode `0x03b11002`, Revision `0x00050005`.
It records logs under `/tmp/mca-ethercat-sil-*` and removes only resources
created by this invocation.

The harness syntax is checked in the IgH API compile workflow. A green GitHub
Actions run validates shell syntax only; actual veth/module/frame acceptance
requires executing the harness on a Linux machine with IgH kernel modules.

### First fixture

Use KickCAT's existing `simulation/slave_configs/ecat402-drive.json`
configuration and its matching `ecat402-drive.xml` ESI fixture. The JSON
enables the built-in `ds402_motor` behavior. The ESI currently identifies
Vendor ID `0x0000029c`, ProductCode `61935618`, RevisionNo `327685`.
These are simulator fixture identity values, not a claim that this is an
available physical drive or a vendor-approved ESI for a real product.

Start with exactly one slave and one PDO domain. Bind the minimum DS402
objects required for a controlled CSP smoke test:
- RxPDO: Controlword `0x6040:00`, Modes of operation `0x6060:00`,
  Target position `0x607A:00`.
- TxPDO: Statusword `0x6041:00`, Modes display `0x6061:00`,
  Position actual value `0x6064:00`.

Before configuring the MCA binding, parse the selected ESI and verify the
actual PDO assignment/mapping entries and bit lengths. Do not assume object
presence in the CoE dictionary means it is mapped into the active PDO. If the
ESI's default mapping differs, explicitly configure it through CoE in PRE-OP
or create a dedicated, reviewed fixture; do not silently hard-code guessed
offsets. Start with SDO identity reads (0x1018) and a non-motion read-only
round-trip before enabling the DS402 controlword sequence.

### Acceptance ladder

1. **Transport smoke:** verify EtherCAT frames cross the veth pair in both
   directions; capture EtherType 0x88A4 and correlate master send/receive
   counters. A successful process start is not acceptance.
2. **Discovery/identity:** IgH discovers exactly one slave and MCA validates
   station position, vendor ID, product code and revision against the ESI.
3. **AL lifecycle:** INIT → PRE-OP → SAFE-OP → OP succeeds; inject a simulator
   link break and verify loss is detected; heal the link and require explicit
   revalidation before returning Active.
4. **CoE SDO:** read 0x1018 identity and selected CiA-402 objects; test a
   bounded write/readback and a deliberate abort/timeout. Ensure SDO work
   does not block cyclic exchange.
5. **PDO/WKC:** exchange mapped RxPDO/TxPDO data for a bounded cycle count;
   require expected WKC to be derived from the configured mapping/topology,
   every observed WKC complete, monotonic cycle sequence, and no stale input
   publication on failed exchange.
6. **CiA-402 behavior:** perform Shutdown → Switch On → Enable Operation,
   verify statusword transitions, set CSP mode, issue a small bounded target
   change, and verify actual position follows with finite lag. Disable voltage
   and verify the simulator returns to Switch-on-disabled.
7. **Fault/recovery:** break/heal the simulated wire while cycling; prove
   WKC loss, AL/online state change, no motion command during recovery,
   topology identity re-check and PDO revalidation after recovery.
8. **Evidence:** preserve KickCAT commit SHA, IgH commit/version, kernel
   version, veth configuration, ESI hash, simulator config, logs, packet
   capture, test report and all thresholds.

### Known limitations and stop conditions

- The existing MCA IgH adapter is behind `MCA_ENABLE_IGH` and calls
  `ecrt_request_master()`; the adapter does not itself create veth devices,
  load kernel modules, bind a NIC or configure `ec_generic`. Those belong in
  a dedicated opt-in Linux harness.
- The present adapter API compile workflow is not a kernel-module/runtime
  test. The fake IgH API tests do not prove packet transmission.
- IgH generic-driver support for a veth endpoint is a testable hypothesis,
  not yet an observed result. First prove module binding and bidirectional
  EtherCAT frames on a disposable Linux host/VM. If `ec_generic` refuses or
  cannot bind veth, stop; do not substitute KickCAT's in-process loopback and
  label it IgH wire-level SIL.
- KickCAT's simulator uses raw packet sockets; its TAP socket is a separate
  shared-memory transport used only when the master is also a KickCAT process.
  MCA IgH cannot connect to KickCAT's private TAP queue directly.
- KickCAT's built-in DS402 motor app is useful for functional protocol
  interaction, but its plant is not a validated model of a particular drive.
- KickCAT's CeCILL-C license requires a separate distribution/compliance
  review before bundling or redistributing it. The proposed first use is an
  external test dependency downloaded at a pinned revision; no KickCAT source,
  binary or headers enter the MCA root build.

### Status

- [x] Upstream source inspection confirms a raw-Ethernet simulator transport
  and a DS402 ESI/device fixture.
- [x] IgH generic driver source/documentation confirms packet-socket-based
  operation on registered Linux Ethernet interfaces.
- [x] Add an opt-in harness for IgH generic/veth/KickCAT one-slave discovery and bidirectional frame capture.
- [ ] Execute the harness on a Linux host and prove IgH `ec_generic` binds the veth endpoint.
- [ ] Prove bidirectional EtherCAT frame exchange with KickCAT over veth.
- [ ] Complete AL/CoE/PDO/WKC/DS402/fault-recovery acceptance.
- [ ] Add an opt-in Linux SIL workflow only after the local proof is stable;
  keep it separate from dependency-free root CI.

