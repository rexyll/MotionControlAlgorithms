# EtherCAT multi-slave health isolation

## Scope

The IgH adapter already discovers a bounded ordered slave topology, creates a
separate slave configuration for every PDO-bound ring position, and records
per-slave online/operational/AL-state diagnostics. PDO bindings also retain
their ring position while the process-data layout stores the corresponding
topology slot.

The missing layer was per-slave health aggregation. The existing
`EthercatIghHealthMonitor` intentionally reports bus-wide health; a single
missing station can therefore make the bus Lost without identifying which
drive remains usable. This document adds the per-slave health contract without
changing the bus-wide monitor or issuing drive commands.

## Process-data isolation contract

`EthercatIghSlaveHealthMonitor` attributes IgH working-counter completeness
to individual stations using the layout's PDO entry ownership:

- A topology slot is selected by `EthercatPdoEntry::slave_position`.
- A station is identified externally by its EtherCAT `ring_position`.
- Every PDO domain used for per-slave isolation must contain entries for one
  and only one topology slot.
- One station may own more than one process-data domain. All of its domains
  must report complete WKC for the station to be Present.
- A domain containing entries from two different stations is rejected with
  `ConstraintViolation`. Its WKC cannot be attributed to either station.
- Discovered but unconfigured stations are omitted from the per-slave snapshot.
  A configured station with no attributable process-data domain is rejected.

Consequently, applications requiring per-drive WKC fault isolation should
assign a dedicated IgH domain to each drive (or a dedicated set of domains to
that drive). Sharing a domain across drives is valid for aggregate process
data, but not for this monitor's per-drive WKC guarantees.

## Health state behavior

The monitor combines each configured station's IgH online/operational state,
the master link state, and completeness of all domains owned by that station.

| State | Meaning |
| --- | --- |
| Unknown | No trustworthy station observation is available yet |
| Present | Station is online and operational; all owned domains have complete WKC |
| Lost | Station is unavailable for operational control or an owned domain has incomplete WKC |
| Recovering | Healthy station feedback has returned, but the configured recovery interval has not elapsed |
| Fault | An application explicitly latched a drive/station fault |

Defaults are immediate loss detection (`lost_after_cycles=1`) and three
consecutive healthy samples for recovery (`recovery_after_cycles=3`). Both
thresholds are configurable. An explicitly latched Fault remains latched across
healthy transport samples. `clear_fault(ring_position)` only moves the monitor
to Recovering; callers must first complete their own vendor/CiA402 fault reset
and verify safe feedback before clearing the monitor latch.

This is diagnostic state, not an STO, safety-rated function, CiA402 controller,
or command sequencer. It does not itself disable another station or authorize
motion. The application's per-drive motion interlocks must consume the
individual health states.

## Verification

The IgH fake API supports up to four independent station configurations and
process-data domains while keeping the default one-station fixture unchanged.

The multi-station tests cover:

1. Two discovered/configured slaves with independent domains and independent
   online/operational/WKC diagnostics.
2. Loss of station 0 and its domain while station 1 remains Present; the bus
   monitor reports bus-wide Lost while the per-slave monitor preserves station
   1 as Present.
3. Recovery of station 0 through Recovering and back to Present after the
   configured consecutive-good interval.
4. Rejection of a process-data domain shared by two stations.
5. An explicit per-station Fault latch that survives good transport feedback
   and only returns through explicit clear plus healthy recovery samples.

These are API/fake-backend tests. They do not yet constitute multi-slave
wire-level SIL. The existing
`run_kickcat_igh_veth_discovery.sh` acceptance runner still enforces exactly
one discovered slave and the cyclic CiA402 SIL harness currently maps one
drive. A separate multi-slave cyclic harness must be added before claiming
multi-drive wire-level acceptance.

## KickCAT simulator capability and next acceptance gate

The pinned KickCAT revision
`cb2e1a192ac11ffaa58f08945053b8129abf94cb` supports multiple slave JSON
configuration files after `-s/--slaves`, and supports an optional topology
file for explicit slave-to-slave links. The current one-slave runner does not
yet expose these options. The next wire-level gate is to run two DS402 simulator
instances on one veth EtherCAT chain, map each drive into its own domain, inject
a fault into only one drive, and prove that:

- both slaves are independently discovered and identified by ring position;
- both reach Operation Enabled from their own statusword feedback;
- the injected drive reaches Fault Reaction Active/Fault while the peer remains
  Operation Enabled with complete domain WKC;
- the faulted drive's explicit reset does not re-enable it;
- the unaffected peer remains operational throughout;
- master link, each slave's AL state and each domain's WKC are logged separately.

A successful multi-slave fake-backend test must not be reported as this
wire-level gate. Real drive/HIL acceptance remains a separate later phase.
