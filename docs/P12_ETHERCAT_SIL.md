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

### Gate 2 — SOEM optional adapter

Pin a SOEM release/commit and build only in an opt-in package/CI job. Start with
discovery, PDO map extraction, process-data exchange, expected/observed WKC,
AL state polling, DC configuration/health and controlled recovery. Keep SOEM
headers and IOmap private to the adapter. Test against an EtherCAT slave
simulator or a dedicated virtual fixture if one is available; otherwise report
only compile/API integration, not EtherCAT protocol SIL.

### Gate 3 — IgH optional adapter

Pin an EtherLab release and build only in an opt-in Linux environment. Validate
master request/release, domain registration, process-data mapping, domain WKC
state, SDO configuration/request, AL state, DC application-time calls and
recovery. Kernel module and NIC-driver compatibility must be recorded. A
successful compile is not runtime bus validation.

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
  upstream release files before adding either dependency.
- Real WKC/DC/recovery quality requires a real slave or a validated EtherCAT
  simulator; fake-master results must be labeled separately.
