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

### Timeout/cancellation rule

`ecrt_sdo_request_timeout()` causes an uncompleted request to become failed
when the configured timeout expires. The public API does not provide enough
information to assume that an arbitrary busy request can be synchronously
cancelled and immediately reused. Therefore the first implementation must
keep a timed-out IgH request quarantined until source-level behavior and a
focused regression test establish a safe reuse rule. A timeout must never
cause immediate object reuse merely because the MCA deadline expired.

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

## 10. Open gates

The following remain intentionally open until evidence exists:

- exact target Linux kernel/NIC/IgH compatibility matrix;
- source-level confirmation of safe busy-request timeout/reuse semantics;
- compile/link validation against the pinned 1.6.13 headers/library;
- simulator or virtual fixture protocol validation;
- physical slave validation;
- cycle jitter and DC phase measurements;
- final licensing/legal review for the chosen distribution model.

These gates prevent the adapter from being marked production-ready based only
on an API-compatible compile.
