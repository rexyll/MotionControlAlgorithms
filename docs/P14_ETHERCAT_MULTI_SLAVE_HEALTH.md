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
6. A latched Fault remains Fault when the IgH slave-diagnostic read temporarily
   fails; non-faulted stations become Unknown until diagnostics recover.

These are API/fake-backend tests. They do not constitute multi-slave wire-level
SIL. Linux CI #1856 at commit `da78675a` reported 759/759 core tests passed,
including all six cases above. The multi-slave cyclic harness also passed the
pinned IgH 1.6.13 API syntax-compile workflow (`ethercat-igh-compile` #307).

## Multi-slave veth runner and baseline SIL

The pinned KickCAT revision
`cb2e1a192ac11ffaa58f08945053b8129abf94cb` supports multiple slave JSON
configuration files after `-s/--slaves`. The veth runner now accepts
`KICKCAT_SLAVE_CONFIGS` as a space-separated list and verifies the exact
expected slave count plus vendor/product/revision identity on every station.
CoE PDO assignment/entry readback runs at each IgH ring position.

The new `mca_ethercat_igh_kickcat_multi_slave_cyclic_sil` target independently
maps each discovered drive to a dedicated process-data domain
(`domain_id = topology_slot + 1`). It resolves the CiA 402 PDO binding set
separately by ring position, brings both drives to stable Operation Enabled
while holding each actual position, then disables both and checks feedback.
The runner selects this target when exactly two config files are supplied.
The existing fault-injection mode remains deliberately restricted to one
slave until independent multi-drive fault sequencing is implemented.

Run the two-drive baseline with the same known-good DS402 simulator JSON twice.
KickCAT creates two distinct simulated slaves from the two config arguments;
the config file remains in the directory containing its relative ESI file:

```bash
export KICKCAT_SOURCE_DIR="$HOME/workspace/MotionControlAlgorithms/scripts/ethercat_sil/KickCAT"
sudo -E env \
  KICKCAT_SIMULATOR="$KICKCAT_SOURCE_DIR/build/simulation/network_simulator" \
  KICKCAT_SLAVE_CONFIGS="$KICKCAT_SOURCE_DIR/simulation/slave_configs/ecat402-drive.json $KICKCAT_SOURCE_DIR/simulation/slave_configs/ecat402-drive.json" \
  MCA_ECAT_EXPECTED_SLAVES=2 \
  MCA_ECAT_COE_READBACK=1 \
  MCA_ECAT_DISABLE_EOE=1 \
  MCA_ECAT_CYCLIC_SIL=1 \
  MCA_ECAT_CYCLIC_TIMEOUT_SEC=60 \
  bash scripts/ethercat_sil/run_kickcat_igh_veth_discovery.sh
```

This is the next user-side wire-level gate. Expected acceptance is two
independently discovered ring positions, CoE readback at both positions,
two independent complete-WKC domains, both drives reaching stable Operation
Enabled from their own statusword feedback, then both returning to
Switch On Disabled. The script selects the multi-slave target automatically
from `MCA_ECAT_EXPECTED_SLAVES=2`.

## Independent multi-drive fault gate (not yet implemented)

After the baseline two-drive run is confirmed, extend the fixture runner and
cyclic test to trigger one drive at a time with different deterministic
`fault_after_cycles` and `fault_reaction_cycles` values. The final acceptance
must prove:

- both slaves are independently discovered and identified by ring position;
- the selected drive reaches Fault Reaction Active/Fault while the peer remains
  Operation Enabled with its own complete domain WKC;
- the faulted drive's explicit reset emits exactly one `0x0080` pulse and does
  not re-enable that drive;
- the unaffected peer remains Operation Enabled throughout the first drive's
  fault and reset;
- the second drive can then be faulted/reset independently;
- master link, each slave's AL state and each domain's WKC are logged separately.

A successful multi-slave fake-backend test or two-drive baseline does not prove
independent multi-drive fault recovery. Real drive/HIL acceptance remains a
separate later phase.
