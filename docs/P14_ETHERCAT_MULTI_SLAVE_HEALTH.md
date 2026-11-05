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

The `mca_ethercat_igh_kickcat_multi_slave_cyclic_sil` target independently
maps each discovered drive to a dedicated process-data domain
(`domain_id = topology_slot + 1`). It resolves the CiA 402 PDO binding set
separately by ring position, brings both drives to stable Operation Enabled
while holding each actual position, then disables both and checks feedback.
The multi-slave runner selects this target when exactly two config files are
supplied. The opt-in independent multi-drive fault mode is implemented below;
it does not reuse the single-slave fault harness.

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

## Independent multi-drive fault isolation SIL

The wire-level multi-slave harness now has an opt-in
`MCA_ECAT_MULTI_FAULT_SIL=1` mode. It selects the target by EtherCAT ring
position, expects exactly two DS402 simulator configs, and verifies that the
non-target config has fault injection disabled. The target must have a positive
`fault_after_cycles`; its configured FRA duration is read from that same
fixture and compared with the observed Statusword sequence.

Each test run targets one drive:

1. Bring both drives to stable Operation Enabled while holding actual position.
2. Wait for only the selected drive to transition through the configured
   Fault Reaction Active duration and then Fault.
3. Require the peer to remain Operation Enabled and both independent PDO
   domains to retain complete WKC on every observed cycle.
4. Arm the CiA 402 reset sequencer from a fresh Fault sample.
5. Send exactly one `0x0080` pulse to the target while continuing to send
   Operation Enabled controlword to the peer.
6. Require target Switch On Disabled and peer Operation Enabled feedback.

Run both directions using the same known-good base DS402 JSON. The script
creates derived fixtures beside the source JSON so relative ESI resources
remain resolvable:

```bash
export KICKCAT_SOURCE_DIR="$HOME/workspace/MotionControlAlgorithms/scripts/ethercat_sil/KickCAT"
export KICKCAT_SLAVE_CONFIG="$KICKCAT_SOURCE_DIR/simulation/slave_configs/ecat402-drive.json"
sudo -E env \
  KICKCAT_SIMULATOR="$KICKCAT_SOURCE_DIR/build/simulation/network_simulator" \
  KICKCAT_SLAVE_CONFIG="$KICKCAT_SLAVE_CONFIG" \
  MCA_ECAT_MULTI_FAULT_AFTER_CYCLES=40 \
  MCA_ECAT_MULTI_FAULT_REACTION_CYCLES=3 \
  bash scripts/ethercat_sil/run_kickcat_igh_multi_slave_fault_isolation.sh
```

By default the script runs the FRA-duration matrix `1,2,3,5` for both ring
positions (eight independent runs). Override it with
`MCA_ECAT_MULTI_FAULT_REACTION_MATRIX=3` to run only the three-cycle cases.
Artifacts are stored under `/tmp/mca-ethercat-multi-fault-*/fra-N/ring-M`.

### Ubuntu wire-level results

The following results were executed on Ubuntu 24.04 / Linux 7.0.0-42-generic
with IgH 1.6.13, a veth pair, and two KickCAT DS402 simulator instances. Each
case includes CoE RxPDO/TxPDO assignment and mapping SDO readback at both ring
positions, two independent PDO domains, stable Operation Enabled feedback,
per-cycle WKC checks, the injected FRA/Fault sequence, one reset pulse, target
Switch On Disabled feedback, peer preservation, and final two-drive disable.

| FRA duration | Target ring 0 | Target ring 1 | Result |
| --- | --- | --- | --- |
| 1 cycle | PASS | PASS | Both directions verified |
| 2 cycles | PASS | PASS | Matrix script completed both targets |
| 3 cycles | PASS | PASS | Both directions verified |
| 5 cycles | PASS | PASS | Both directions verified on 2026-11-05 |

For the FRA=2 run on 2026-11-05, the script completed the full two-target
matrix at artifact root
`/tmp/mca-ethercat-multi-fault-20261105-102956`. The captured ring-1 trace
showed target Statusword `0x0F` for cycles 205 and 206, followed by Fault
(`0x08`) at cycle 207. The peer remained Operation Enabled (`0x37`) and
both domains remained at WKC 3. The reset sequencer reported exactly one
reset pulse; target returned to Switch On Disabled (`0x40`), peer remained
`0x37`, and final health reported total WKC 6 with both domains complete,
online, and operational before both drives were confirmed disabled.

The first FRA=5 attempt on 2026-11-05 passed target ring 0 but failed
before fault injection on target ring 1 during the two-drive enable phase.
That first attempt is retained as a transient transport/WKC failure sample,
not a fault-isolation verdict.

The diagnostic-instrumented rerun completed both directions at artifact root
`/tmp/mca-ethercat-multi-fault-20261105-104820`. For target ring 0, the
five consecutive FRA samples were cycles 199–203 (`0x0F`), followed by
Fault (`0x08`) at cycle 204. For target ring 1, the equivalent five FRA
samples were also cycles 199–203, followed by Fault at cycle 204. In both
directions the peer remained Operation Enabled (`0x37`), and each target
and peer domain reported WKC 3 throughout the observed fault sequence. Each
run sent exactly one Fault Reset pulse; the target returned to Switch On
Disabled (`0x40`) while the peer remained `0x37`. Final health in both
runs reported total WKC 6, link up, two responding slaves, and both domains
complete/online/operational, followed by successful disable feedback.

The prior ring-1 attempt's pre-cyclic diagnostics showed 8 lost frames in
the IgH master snapshot (2784 transmitted, 2776 received; 0.4% frame loss),
an AL-state datagram timeout for slave 1 followed by master-link DOWN/UP
transitions, and KickCAT frame-processing spikes up to 23.48 ms. These were
setup-time observations, not proof of the later cyclic failure's cause.
The added failure-path diagnostics remain useful for future intermittent
transport regressions; the subsequent complete FRA=5 rerun establishes that
both ring-position fault-isolation cases now pass without relaxing any WKC or
Operation Enabled acceptance criteria.

The earlier FRA=1 and FRA=3 runs also completed in both ring-position
directions. The previous FRA=1 CoE readback failure was traced to an expired
IgH datagram transaction: the expected SDO response reached the master after
the 500 microsecond datagram timeout and was logged as an unmatched datagram.
The test harness now defaults IgH debug logging to level 0 and captures raw
pcap files during SDO readback, rendering text dumps only after the capture
stops. The subsequent FRA=2 matrix passed, but this correlation alone does
not prove that capture overhead was the sole cause of the earlier timeout.

## Downstream communication-loss and link-recovery SIL

A separate communication-fault mode uses the pinned KickCAT simulator's
out-of-band `SimulatorControl` channel to break and heal the physical link
between node 0 and node 1 while the IgH cyclic application remains active.
This is distinct from the DS402 Fault Reaction Active fixture: it changes the
emulated EtherCAT physical topology rather than injecting a CiA 402 Statusword.

The initial test is deliberately scoped to the default two-node daisy chain:

- ring position 0 is upstream and remains connected to the master;
- ring position 1 is downstream and is isolated by breaking link 0:1;
- while the link is broken, ring 0 must retain WKC 3 and remain Operation
  Enabled, while ring 1 must report WKC 0/incomplete and the per-slave health
  monitor must report Lost;
- after a control-channel acknowledgement for healing link 0:1, both domains
  must return to WKC 3, both drives must report Operation Enabled, and the
  downstream health state must recover to Present for at least ten consecutive
  cycles.

This test does not claim that a break at every physical cable location has the
same outcome, nor does it test cable redundancy. In a daisy chain, a break
between upstream and downstream slaves physically partitions the downstream
segment; the peer-preservation assertion applies only to the upstream drive.

### Build prerequisites

Use the same pinned KickCAT checkout used by the DS402 fault fixture. Its
`network_simulator` must be built with the simulator-control option enabled,
and the static `libkickcat.a` must exist under `$KICKCAT_SOURCE_DIR/build/lib`.
The communication SIL runner creates a per-run shared-memory control channel,
passes its name to `network_simulator --control`, builds a small channel
initialiser, and unlinks the shared-memory object during cleanup.

### Ubuntu wire-level command

After pulling `main`, applying the existing DS402 fixture patch if needed,
and rebuilding the KickCAT simulator:

```bash
cd ~/workspace/MotionControlAlgorithms
git pull --ff-only origin main

export KICKCAT_SOURCE_DIR="$HOME/workspace/MotionControlAlgorithms/scripts/ethercat_sil/KickCAT"
cmake --build "$KICKCAT_SOURCE_DIR/build" --target kickcat network_simulator -j"$(nproc)"

sudo -E env \
  KICKCAT_SOURCE_DIR="$KICKCAT_SOURCE_DIR" \
  KICKCAT_SIMULATOR="$KICKCAT_SOURCE_DIR/build/simulation/network_simulator" \
  KICKCAT_SLAVE_CONFIG="$KICKCAT_SOURCE_DIR/simulation/slave_configs/ecat402-drive.json" \
  MCA_ECAT_CYCLE_TIME_US=1000 \
  MCA_ECAT_DEBUG_LEVEL=0 \
  bash scripts/ethercat_sil/run_kickcat_igh_multi_slave_communication_fault.sh
```

Expected output includes a link-break acknowledgement, ten or more cycles with
ring-1 domain WKC 0 while ring 0 remains WKC 3/Operation Enabled, then a
link-heal acknowledgement and ten or more stable cycles with both domains
WKC 3 and both drives Operation Enabled. The complete cyclic test then
performs its normal safe disable.

The CI syntax/build gate compiles this mode with
`MCA_ENABLE_KICKCAT_SIM_CONTROL=1` and builds the pinned KickCAT core library.
That is not a substitute for the Ubuntu wire-level run. Until an artifact root
and both break/heal traces are recorded, communication-loss SIL remains
pending; no communication fault or recovery pass is claimed here.

This is KickCAT SIL only: vendor-specific fault semantics, distributed-clock
timing, real-drive fault behavior and physical HIL remain unvalidated. Peer
preservation in this simulator is not a safety-rated isolation function.
