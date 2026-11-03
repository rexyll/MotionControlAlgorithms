# P12 — IgH EtherCAT Adapter Design

## Status

This document defines the implementation boundary for the optional Linux IgH
adapter. It is an implementation design, not a claim of physical EtherCAT
validation.

**Pinned upstream:** EtherLab EtherCAT Master 1.6.13, `stable-1.6`, commit
`61cc654f5b721ddd54df0f58bdd34106d91c5359`.

The adapter remains optional. The portable MCA core must compile without IgH,
and no IgH header or type may leak into `include/mca/hardware/ethercat_master.hpp`.

## 1. Ownership and lifecycle

The adapter owns exactly one IgH master context and its configured domains.
The lifecycle is:

```text
Offline
  -> Discovered
  -> Configured
  -> Active
  -> Recovering
  -> Configured/Active
  -> Fault
```

The adapter must not report `Active` until topology identity, PDO mapping and
backend configuration have been validated. Slave AL state and the MCA master
lifecycle are separate state machines.

Recommended non-cyclic lifecycle:

1. `ecrt_request_master(0)` (or the configured master index).
2. Discover/validate expected slave identity.
3. Obtain each slave configuration object.
4. Configure PDOs/sync managers and static SDO configuration.
5. Create one `ec_sdo_request_t` per concurrently active MCA mailbox slot.
6. Configure each request timeout before activation.
7. Configure distributed clocks when required by the slave profile.
8. Register domain PDO entries and obtain process-data offsets.
9. Activate the master.
10. Obtain domain process-data pointer(s).
11. Enter the cyclic exchange loop.
12. On shutdown, leave cyclic operation, deactivate/release the master and
    discard all adapter-owned references.

The public IgH documentation states that realtime SDO request objects are
created before `ecrt_master_activate()` and are automatically freed when the
master is released. citeturn694293search0

### Activation failure policy

A non-zero return from `ecrt_master_activate()` is treated as a terminal
configuration failure for this adapter instance. The adapter releases the
master and returns to `Offline`; it does not retry activation on the same
master. In the IgH 1.6 implementation, activation finalizes domains in order
and can fail while finalizing a domain or starting the operation thread. The
public API documents an error return but does not promise that all partially
completed activation work is rolled back into a retryable configuration.
Releasing the master is the conservative cleanup boundary.

After successful backend activation, if domain process-data access is
invalid, the adapter attempts `ecrt_master_deactivate()` and then releases the
master regardless of the deactivation result. This is required because IgH
deactivation destroys the domain, slave-configuration and SDO-request objects;
the previous `Configured` state cannot be reused after that operation. The
same fail-closed rule applies if the MCA state transition to `Active`
unexpectedly fails: deactivate, clear backend references and release the
master.


## 2. Cyclic process-data path

The cyclic path must be bounded and allocation-free:

```text
MCA output image
    -> validate configured layout/capacity
    -> write mapped PDO entries
    -> ecrt_domain_queue()
    -> ecrt_master_send()
    -> ecrt_master_receive()
    -> ecrt_domain_process()
    -> capture each domain's WKC and completeness
    -> ecrt_domain_queue() for every domain
    -> ecrt_master_send()
    -> read mapped input PDO entries
    -> publish observed WKC/domain health
```

The adapter must never perform an SDO wait, sleep, blocking allocation, string
formatting, topology rediscovery or recovery transaction inside `exchange()`.

For each domain retain:

- expected working counter when the backend exposes it (otherwise mark it unavailable);
- last observed working counter;
- complete/incomplete/zero state;
- cycle sequence;
- last successful exchange timestamp;

IgH 1.6.13 exposes `working_counter` and `wc_state` through
`ecrt_domain_state()`, but does not expose the configured expected counter as a
public `ec_domain_state_t` field. The IgH adapter therefore reports the
per-domain observed counter and `EC_WC_COMPLETE` result; it does not invent an
expected value. A zero expected-counter accessor means unavailable, not an
expected WKC of zero. The aggregate observed counter is a saturating sum, while
per-domain values remain authoritative.

An incomplete WKC is a health fault, not a reason to stop the cyclic datagram
pipeline. The adapter must still attempt to queue every domain and send the
next frame, then report the degraded cycle to its caller. Returning before
`ecrt_domain_queue()`/`ecrt_master_send()` can prevent transient communication
faults from recovering.
- missed/deadline count.

IgH exposes domain state including working-counter information, and its
application interface is explicitly intended to monitor process-data exchange
in realtime. citeturn694293search0turn694293search2

A domain with incomplete/zero process-data exchange is a transport-health
failure. It must not be silently converted into a valid drive state.

## 3. PDO mapping and topology

MCA owns the semantic PDO map. IgH owns the concrete domain registration.
The adapter translates the validated MCA map into `ec_pdo_entry_reg_t`
registrations and stores the resulting byte/bit offsets privately.

`EthercatPdoEntry::slave_position` is a bounded topology-array slot (0 to
`slave_count - 1`), not the EtherCAT station address. IgH adapter bindings
carry the actual station address separately. The adapter must resolve each
binding's station address to a topology slot before constructing the portable
layout; sparse station addresses must never be used as array indices or copied
into this bounded field.

Before activation:

- reject missing expected slaves;
- reject vendor/product/revision/serial identity changes;
- reject duplicate station positions;
- reject unsupported bit widths and offset overflow;
- reject overlapping MCA entries;
- reject insufficient caller-owned input/output image capacity;
- reject an identity match that has not completed backend revalidation.

No old PDO offset is reusable after a topology identity change.

## 4. Asynchronous SDO bridge

The MCA mailbox queue is the ownership layer. Each live MCA request is mapped
to a reserved IgH `ec_sdo_request_t` object. A request object is never reused
while IgH reports `EC_REQUEST_BUSY`.

Mapping:

| MCA state | IgH state/action |
|---|---|
| queued | MCA queue owns request; no IgH operation yet |
| submitted | set index/subindex and prepare write/read |
| pending | `EC_REQUEST_BUSY`; poll every cycle or bounded diagnostic tick |
| completed | `EC_REQUEST_SUCCESS`; copy data and retire MCA request |
| failed | `EC_REQUEST_ERROR`; capture bounded diagnostic status |
| timed out | MCA deadline expires; do not block cyclic exchange; backend cleanup/recovery must be handled explicitly |

IgH provides `ecrt_sdo_request_read()`, `ecrt_sdo_request_write()`,
`ecrt_sdo_request_state()`, `ecrt_sdo_request_timeout()`,
`ecrt_sdo_request_data()` and `ecrt_sdo_request_data_size()` for this model.
The API explicitly forbids starting a new operation while the request is
`EC_REQUEST_BUSY`. citeturn694293search0

### Critical buffer rule

After `ecrt_sdo_request_read()` is issued, the pointer returned by
`ecrt_sdo_request_data()` is considered invalid while the request remains
`EC_REQUEST_BUSY`, because IgH may reallocate its internal receive buffer.
Only after `EC_REQUEST_SUCCESS` may the adapter read the pointer and copy at
most the MCA result capacity. citeturn694293search0turn694293search3

The adapter must therefore never cache that pointer across cycles. It must
copy terminal data into the fixed-capacity `EthercatSdoResult` and then retire
the MCA request.

### Fixed-size request-object constraint

The `size` argument to `ecrt_slave_config_create_sdo_request()` reserves the
request object's data buffer. `ecrt_sdo_request_data_size()` is a getter, not a
public setter. Consequently, one IgH request object cannot safely represent
arbitrary variable-length writes by merely changing the MCA payload length.
The adapter must use a configuration-time SDO profile that declares the
station, object index/subindex and exact transfer capacity for each request
slot. A write must match that configured transfer size; a read may return no
more than the reserved capacity. Concurrent requests require distinct IgH
request objects, each owned by the corresponding slave configuration.

The portable contract represents this profile as
`EthercatSdoProfile<MaxSlots, MaxBytes>` with one
`EthercatSdoSlotConfig` per preallocated backend request object. Repeated
station/index/subindex tuples are permitted only when their capacity and
timeout agree; they represent independent concurrency slots. The profile
rejects writes whose payload length is not exactly the configured capacity,
reads whose response bound exceeds it, and requests for unconfigured objects.

The IgH adapter now exposes `configure_sdo_profile()`,
`enqueue_sdo()` and `poll_sdo()`. The profile is materialized into distinct
IgH request objects after PDO configuration and before activation. Enqueue
copies fixed-size write data into the reserved buffer and schedules a read or
write; polling copies read data only after `EC_REQUEST_SUCCESS`. Duplicate
live request IDs are rejected. An MCA deadline observed while IgH is BUSY
returns `TimedOut` but keeps the backend slot quarantined until IgH reaches a
terminal state. Since IgH exposes no completion timestamp, a result first
observed after the MCA deadline is conservatively reported as timed out.

The adapter methods are a single-owner API: callers must serialize
`configure_sdo_profile()`, `enqueue_sdo()`, `poll_sdo()`, `exchange()` and
`shutdown()` on one control/RT execution context. The adapter does not add
locks, and concurrent calls from independent threads are unsupported. The
enqueue path also checks the live IgH request state before submission rather
than trusting only MCA slot bookkeeping. Shutdown is idempotent and clears
adapter-owned handles/bookkeeping even if the master pointer was already null.

The current implementation has focused Fake C-API regression coverage for
independent concurrent slots, duplicate live IDs, backend BUSY-slot skipping,
deadline quarantine, terminal ERROR mapping and slot reuse, SUCCESS-only read
buffer access (including pointer relocation), response-size bounds, exact-size
writes, and write-payload copying. Setup rollback is tested for both request
creation failure and timeout-configuration failure; shutdown is tested for
deactivation/release exactly once and repeated-call idempotence.

These tests exercise the adapter's state and resource-management logic against
a deterministic C-API test double. They do not emulate EtherCAT frames, CoE
protocol behavior, kernel scheduling, NIC DMA, bus timing, or physical slave
firmware. They are not a substitute for simulator/fixture testing or HIL.
### Error and Abort-code observability

The realtime request API exposes only `EC_REQUEST_UNUSED`,
`EC_REQUEST_BUSY`, `EC_REQUEST_SUCCESS` and `EC_REQUEST_ERROR`. It does not
expose a CoE SDO abort code for a failed realtime request. The IgH adapter must
therefore map `EC_REQUEST_ERROR` to MCA `Failed` with `abort_code == 0`; it
must not claim `Aborted` or invent an abort code. The MCA `Aborted` result is
reserved for backends that provide an observable abort code. The synchronous
master SDO upload/download functions have a separate abort-code output, but
they are not a substitute for the non-blocking realtime request path.

### Timeout/cancellation rule

The configured `ecrt_sdo_request_timeout()` value is **not an end-to-end
cancellation guarantee for an in-flight mailbox transfer**. In the pinned
IgH 1.6.13 source, `ec_fsm_master_action_process_int_request()` checks
`ec_sdo_request_timed_out()` while the request is still
`EC_INT_REQUEST_QUEUED`; once dispatched, the CoE state machine owns
completion/failure and applies its own datagram/retry behavior. A configured
IgH issue timeout can therefore reject a request that waited too long in the
queue, but must not be treated as a hard upper bound on a request already in
`EC_REQUEST_BUSY`.

In IgH 1.6.13, the public request-state translation maps both internal
`EC_INT_REQUEST_QUEUED` and `EC_INT_REQUEST_BUSY` to
`EC_REQUEST_BUSY`; only `EC_INT_REQUEST_INIT` maps to
`EC_REQUEST_UNUSED`. Since `ecrt_sdo_request_read/write()` changes the
internal state to QUEUED before returning, a successfully enqueued request
cannot be mistaken for UNUSED by the adapter. The CoE transfer FSM also has
bounded retry/error transitions for missing or invalid datagrams, after
which the request becomes terminal ERROR. These source-level guarantees
support the adapter's terminal-state slot-reuse rule; they do not establish
real-device recovery timing or guarantee recovery from every kernel/master
failure mode.

The MCA deadline remains the caller-visible deadline. If it expires while
IgH is BUSY, report `TimedOut` to the caller but quarantine the backend slot.
Do not infer cancellation and do not reuse the request object until IgH
reports a terminal `SUCCESS` or `ERROR`. In the pinned source, an SDO FSM
failure marks the internal request `EC_INT_REQUEST_FAILURE` and restarts the
master FSM; the public state maps that terminal failure to `EC_REQUEST_ERROR`.
That establishes the source-level basis for reusing a slot after observed
`ERROR`, but does not replace simulator/fixture testing of real mailbox
behavior. If a request remains BUSY indefinitely due to an unresolved backend
fault, the safe policy is to keep the slot quarantined and escalate recovery,
not to force-reuse it.

## 5. Distributed clocks

DC configuration is performed before activation. The adapter reports at least:

- unsupported;
- not configured;
- synchronizing;
- locked;
- lost/degraded.

The adapter must expose measured phase deviation separately from capability or
configuration flags. No fixed jitter/precision guarantee may be inferred from
successful DC configuration.

## 6. Recovery

Recovery is asynchronous and cannot block the cyclic exchange.

A detected slave loss or domain WKC failure moves the affected health state to
`Lost`/`Recovering` according to the master-neutral contract. Recovery must:

1. stop motion-producing outputs according to the higher-level safety policy;
2. continue bounded observation;
3. detect slave return and verify identity;
4. rebuild/revalidate PDO configuration when identity or mapping may have
   changed;
5. restore process-data exchange;
6. require valid domain/WKC evidence before returning to `Active`.

A rediscovered slave is never treated as valid solely because its station
position returned.

## 7. Error containment

All backend errors are translated to MCA-owned bounded status values. Do not
store raw `ec_*` pointers or dynamically allocated backend error strings in
portable public objects.

On failed input acquisition, the caller's previous output/input image remains
unchanged. On a failed configuration transaction, the previous active
configuration remains authoritative until the adapter has a complete new
configuration.

## 8. Build integration

IgH is an opt-in Linux dependency. The root portable build must not require:

- IgH headers;
- EtherCAT kernel modules;
- privileged NIC access;
- a realtime kernel;
- a physical EtherCAT adapter.

The adapter should be built as a separate Linux target/package that consumes an
already installed/pinned IgH userspace development package. Do not vendor IgH
sources or silently FetchContent them into the default project.

CI levels should be separated:

1. **portable CI:** fake master and all MCA contract tests;
2. **IgH compile CI:** Linux headers/library available, adapter compiles and
   links, no physical bus required;
3. **IgH runtime fixture CI:** dedicated simulator/fixture if available;
4. **HIL:** bare-metal Linux, dedicated NIC, real slaves and measured timing.

A successful level 2 build must not be reported as level 3/4 validation.

## 9. Acceptance tests before physical HIL

The first adapter implementation is accepted only when tests cover:

- master request/release lifecycle;
- missing and mismatched slave identity;
- deterministic PDO registration and offset capture;
- cyclic send/receive/process ordering;
- expected versus observed WKC;
- SDO read success and terminal data copy;
- SDO write success;
- SDO busy state preventing reissue;
- SDO timeout without blocking the cyclic path;
- late completion after MCA timeout;
- response larger than MCA capacity;
- domain failure and recovery state transition;
- DC unsupported/configured/health reporting;
- clean deactivate/release with no outstanding active request reused.

Where the real IgH library cannot be loaded in ordinary portable CI, use a
small fake C API shim with the same subset of declarations to exercise the
adapter state machine. This fake is a unit-test double, not an EtherCAT
protocol simulator.

### Fake IgH regression coverage (main)

The portable fake-C-API suite now exercises the following SDO and lifecycle
behaviors against the real adapter implementation:

- read completion copies data only after `EC_REQUEST_SUCCESS`;
- the fake can relocate the backend data buffer between BUSY and SUCCESS, and
  the adapter observes/copies the terminal buffer rather than a cached pointer;
- the 256-byte adapter response ceiling and profile transfer-capacity bounds;
- exact-capacity writes, overflow rejection and payload copying before submit;
- independent concurrent slots, duplicate live request-ID rejection and
  dispatch around an unrelated backend BUSY slot;
- timeout quarantine until terminal observation, ERROR mapping and subsequent
  slot reuse;
- injected failure on the second request-object creation and on the second
  timeout configuration, each requiring master release to discard all
  previously created backend request objects;
- shutdown with an outstanding SDO operation, verifying deactivate/release
  occur once and repeated shutdown is idempotent.
- activation failure discards the configured master instead of retrying an
  IgH instance whose partial activation rollback is not guaranteed;
- invalid domain data/size always releases the deactivated master to Offline,
  because IgH deactivation destroys the backend configuration objects.

The fake models API state and memory ownership boundaries only. It does not
emulate EtherCAT frames, CoE mailbox timing, slave firmware, distributed clocks,
NIC DMA, kernel scheduling or electrical bus faults. These tests are portable
unit regressions, not protocol simulation and not hardware-in-the-loop evidence.

## 10. Open gates

The following remain intentionally open until evidence exists:

- exact target Linux kernel/NIC/IgH compatibility matrix;
- source-level confirmation of safe busy-request timeout/reuse semantics;
- broader protocol-level validation of SDO timing and recovery beyond the fake C-API state-machine coverage;
- runtime validation of whether an IgH request that reaches `EC_REQUEST_ERROR`
  can be safely rescheduled under real master/slave timing without recreating the master;
- an explicit production policy for realtime IgH errors that do not expose CoE
  abort codes;
- compile/link validation against the pinned 1.6.13 headers/library;
- simulator or virtual fixture protocol validation;
- physical slave validation;
- cycle jitter and DC phase measurements;
- final licensing/legal review for the chosen distribution model.

These gates prevent the adapter from being marked production-ready based only
on an API-compatible compile.


## Bounded PDO mapping profile (2026-11-03)

The first implementation slice adds
`include/mca/hardware/ethercat_pdo_mapping_profile.hpp`. It is a fixed-capacity,
allocation-free declaration/validator for the intended mapping. The profile
records station position, vendor/product identity, Sync Manager index, PDO
index, object index/subindex, bit length and direction.

The validator rejects:
- duplicate object entries for one station;
- zero PDO/object indices and bit lengths outside 1..64;
- decreasing station or Sync Manager order;
- a direction change inside one Sync Manager;
- a PDO group that reappears after a later PDO group in the same SM.

The profile must be supplied in station/SM/PDO/entry order. It deliberately
does not discover the physical slave's current mapping. This separates
deterministic application intent from backend application and from
independent on-wire verification.

Portable GTest coverage is in
`tests/hardware/ethercat_pdo_mapping_profile_test.cpp`; a dependency-light
smoke test is also run by `ethercat-pdo-image`.

### IgH profile application

The adapter now exposes an overload of `configure(bindings, count, mapping)`.
It validates that every process-image binding resolves to exactly one profile
entry with the same station/vendor/product identity, object index/subindex,
bit width and direction. The profile may additionally contain mapped entries
that the current process image does not consume; those entries are still
included in the configured PDO layout.

For each slave, the adapter groups the profile into bounded
`ec_sync_info_t` -> `ec_pdo_info_t` -> `ec_pdo_entry_info_t` arrays and calls
`ecrt_slave_config_pdos()`. Only after all requested slave mappings have been
applied does it call `ecrt_slave_config_reg_pdo_entry()` for the registered
process-image entries. The arrays are fixed-capacity and no heap allocation is
performed. A failed PDO-configuration call releases the partially configured
inactive master; registration and activation are not attempted afterwards.

The original two-argument `configure(bindings, count)` overload remains
available and intentionally retains "use the slave's existing mapping"
semantics. It does not claim to apply or verify a mapping.

Fake IgH tests now inspect the exact Sync Manager direction, PDO index, entry
index/width and call ordering, cover a complete profile with an unconsumed
mapped entry, verify mapping/binding mismatch rejection, and inject
`ecrt_slave_config_pdos()` failure to check rollback. These are API-contract
unit tests only.

### Remaining protocol verification

The KickCAT test must independently read back the active assignment/mapping
through CoE objects 0x1C12/0x1C13 and 0x1600+/0x1A00+ where supported. A
successful `ecrt_slave_config_pdos()` call proves only that IgH accepted the
configuration request; it is not proof that a physical slave applied the map.
The next protocol gate is therefore SDO readback plus cyclic PDO/WKC exchange
against KickCAT, followed separately by real-slave HIL.
