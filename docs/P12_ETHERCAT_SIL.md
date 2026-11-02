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

**Current candidate:** official EtherLab EtherCAT Master 1.6 series; upstream recommends the `stable-1.6` branch for production systems. This is a Linux-only kernel-module/userspace-library architecture, not a Windows alternative.

**License split (must be reviewed for the actual distribution model):** the master/kernel-side code is GPLv2; the official manual states the userspace application library `libethercat` is LGPLv2.1 to permit static linking of userspace applications. This is materially different from SOEM 2.0's GPLv3/commercial dual license, and makes IgH worth evaluating for a Linux robot controller with a proprietary userspace application. Do not infer that the complete deployment is automatically cleared: kernel module packaging/modification, static vs dynamic linking, notices/source obligations, and any EtherCAT technology/trademark requirements need project/legal review.

**Technical fit:** official EtherLab documentation lists userspace SDO requests/handlers, domain process data and WKC state, AL state monitoring, DC application-time APIs, and bus scanning/reconfiguration. Its userspace API is a natural candidate for keeping SDO request progression outside the hard cyclic loop. Verify exact asynchronous SDO lifecycle and cancellation semantics against the pinned 1.6 API before mapping it to `IEthercatMailbox`; never block `exchange()` on a completed SDO transfer.

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
3. Add an optional SOEM adapter behind a separate CMake option/package; keep
   root CI unaffected.
4. Add an optional IgH adapter only after its Linux build/runtime environment
   can be pinned reproducibly.
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
