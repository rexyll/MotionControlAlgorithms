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
the config file remains in the directory containing its relative ESI file.
The latest successful CoE readback for this simulator confirmed, on both
ring positions, Rx assignment `0x1C12 → 0x1600` and Tx assignment
`0x1C13 → 0x1A00`, with the mapping entries listed in the parity section
below. Use those observed PDO indices rather than selecting an alternative
PDO from the ESI.

The pinned KickCAT `ecat402-drive.xml` marks these base PDO/object indices
`DependOnSlot=true` despite declaring no `Slots` section. The resolver
therefore keeps its default fail-closed behavior; the explicit
`--treat-slot-dependent-indices-as-literal` override is allowed only for
non-modular devices with no `Slots` section. It performs no index arithmetic
and preserves the original ESI flags. This simulator-specific override is
justified here by the independent CoE SDO readback above; do not use it for a
modular device or without matching runtime evidence.

```bash
export KICKCAT_SOURCE_DIR="$HOME/workspace/MotionControlAlgorithms/scripts/ethercat_sil/KickCAT"
python3 scripts/ethercat_esi/esi_to_profile.py \
  "$KICKCAT_SOURCE_DIR/simulation/slave_configs/ecat402-drive.xml" \
  --product-code 0x03b11002 \
  --rx-pdo 0x1600 --tx-pdo 0x1a00 \
  --treat-slot-dependent-indices-as-literal \
  --emit-pdo-snapshot -o /tmp/ecat402-drive-pdo-snapshot.json
export MCA_ECAT_ESI_PDO_SNAPSHOT=/tmp/ecat402-drive-pdo-snapshot.json

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

## ESI-to-runtime PDO parity gate

The three pairwise parity comparators and their combined fail-closed gate are
implemented and covered by offline tests. The earlier implementation gap is
closed at the report/comparison layer; this does **not** mean the comparators
have been run against a newly collected two-slave runtime snapshot.

The combined veth runner uses three independent relationships:

1. **ESI ↔ CoE:** compares the ESI expected assignment order and ordered raw
   mapping entries with CoE SDO readback at each ring position.
2. **ESI ↔ IgH:** compares the ESI expected SyncManager, direction, PDO and
   Entry sequence with IgH's discovered PDO hierarchy.
3. **CoE ↔ IgH:** independently joins by ring position, Vendor ID/Product code
   and PDO index, then compares ordered raw 32-bit mapping words, including
   padding. It does not infer CoE assignment objects or subindices from IgH.

When both `MCA_ECAT_COE_READBACK=1` and `MCA_ECAT_CYCLIC_SIL=1` are set,
`MCA_ECAT_ESI_PDO_SNAPSHOT` is mandatory. The script requires all three
pairwise reports, and `three_way_pdo_parity.py` verifies report schema/source
kind, exact slave cardinality, contiguous ring positions, and every
pairwise/per-slave match. Any missing report or mismatch fails the combined
SIL run. The consolidated artifact is `three-way-pdo-parity.json`.

Generate the expected snapshot from the selected device's ESI XML
using the repository resolver. For the pinned KickCAT Ingenia simulator,
the last verified runtime SDO readback selects RxPDO `0x1600` and TxPDO
`0x1A00`; the exact simulator command above includes the explicit
non-modular literal-index override and exports the resulting snapshot path.
For any other device/profile, select PDO indices only from that device's
verified configuration and runtime readback; never copy the simulator values.

For modular devices, the current slot resolver must not be represented as a
complete runtime PDO mapping unless its output explicitly reports a complete
mapping. Slot-resolved PDO snapshot conversion remains a separate limitation;
external module ESI files must be supplied explicitly with `--module-esi`
when required by the ESI's InfoReference list.

For the pinned KickCAT Ingenia profile, both previously captured
ring positions reported the following exact ordered maps:

| Assignment | PDO | SyncManager | Ordered entries (index:subindex/bit width) |
| --- | --- | --- | --- |
| Rx `0x1C12:1` | `0x1600` | 2 / output | `6040:00/16, 607A:00/32, 60FF:00/32, 6060:00/8` |
| Tx `0x1C13:1` | `0x1A00` | 3 / input | `6041:00/16, 6064:00/32, 606C:00/32, 6061:00/8` |

These are prior runtime observations, not a result from the latest
`three-way-pdo-parity` combined run. The new override makes it possible to
render an ESI expectation for this specific non-modular simulator profile;
the combined run must still collect fresh CoE and IgH snapshots and pass all
three comparators before runtime parity can be accepted.

The combined report is a consolidation of the three pairwise reports, not a
fourth independent source of device evidence. It validates the pairwise
reports' schema, coverage and match flags; the underlying collectors and
comparators remain responsible for validating their own snapshot content.
Offline unit/CI success proves the implementation and test behavior only.
A wire-level pass requires running the combined script with the actual
selected ESI snapshot and two KickCAT slave configurations and preserving all
four JSON reports plus raw CoE/IgH artifacts. No runtime three-way parity
result is claimed until that execution succeeds. This is KickCAT/IgH SIL,
not physical-drive HIL, vendor qualification, distributed-clock acceptance,
or a safety-rated function.

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
remain resolvable. Because the combined CoE + cyclic runner requires an
explicit ESI expectation for three-way PDO parity, the matrix runner now
generates the pinned KickCAT ESI PDO snapshot once under the artifact root
(or validates and reuses a caller-supplied
`MCA_ECAT_ESI_PDO_SNAPSHOT`) and passes it to every matrix case. The 10 ms
KickCAT software-simulation cycle is the default; override it explicitly with
`MCA_ECAT_CYCLE_TIME_US` only when the simulator host can sustain that rate:

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
  MCA_ECAT_COMM_RECOVERY_TIMEOUT_MS=5000 \
  bash scripts/ethercat_sil/run_kickcat_igh_multi_slave_communication_fault.sh
```

Expected output includes a link-break acknowledgement, ten or more cycles with
ring-1 domain WKC 0 while ring 0 remains WKC 3/Operation Enabled, then a
link-heal acknowledgement and ten or more stable cycles with both domains
WKC 3 and both drives Operation Enabled. The complete cyclic test then
performs its normal safe disable.

The first Ubuntu run on 2026-11-05 reached and passed the link-break
isolation phase. Artifact root:
`/tmp/mca-ethercat-sil-20261105-112056`. It confirmed ten isolated cycles
with ring 1 WKC 0 and ring 0 WKC 3 while the upstream peer remained Operation
Enabled. The link-heal command was acknowledged and both Domain WKC values
returned to 3, but the run failed its recovery gate:
`recovery_stable_cycles=0` and the downstream health state was not Present.
This is a partial pass only; the downstream AL-OP/CiA402 recovery is not
established. The captured cyclic log has no per-cycle recovery-state trace,
and the post-failure kernel messages were captured after adapter shutdown,
so they cannot identify whether the downstream slave remained outside AL-OP,
whether the IgH per-slave state read was invalid, or whether a drive feedback
condition blocked the gate.

The recovery loop emits bounded `COMM_RECOVERY_TRACE` samples (first 20
cycles, then every 100 cycles) containing receive/monitor status, master link
and responding count, per-slave online/operational/AL state and state-read
error, per-domain WKC, DS402 Statusword/Mode Display, feedback-read results,
health state, and send result.

### IgH rescan behavior and recovery timeout

The second instrumented run at
`/tmp/mca-ethercat-sil-20261105-124321` exposed the interaction between a
topology change and IgH's master scan FSM. The kernel log records:

- At 12:43:23.435833, the master observed one responding slave and started a
  rescan.
- That scan completed at 12:43:23.992980.
- At 12:43:23.995868, the master observed two responding slaves and immediately
  started another rescan at 12:43:24.005278.
- The cyclic application reached its fixed 1,000-cycle recovery limit and
  shut down at 12:43:24.537124. The second scan did not complete until
  12:43:24.560812, after the application had already begun shutdown.

IgH 1.6.13's master FSM detects a changed responding-slave count in its
broadcast-state phase and requests a topology rescan. The rescan clears the
current slave objects; this detaches the corresponding `ec_slave_config`
references. Once the scan discovers the current topology, IgH reattaches the
configured slave configs. While a config is detached,
`ecrt_slave_config_state()` still returns success but reports
`online=0`, `operational=0`, and AL state `0x00` (UNKNOWN). This explains
why both configured stations changed to offline together even though PDO
WKC and the old process-image contents could still appear valid. It is not
evidence that either drive transitioned to AL INIT.

The prior 1,000-cycle wait was approximately one second at the configured
1 ms cycle and expired about 23 ms before the second scan completed. The
timeout, not a relaxed protocol criterion, was the immediate test-harness
limitation. The recovery phase now uses a monotonic wall-clock deadline of
5 seconds by default, configurable with
`MCA_ECAT_COMM_RECOVERY_TIMEOUT_MS` (valid range 100–60,000 ms). It keeps
the cyclic exchange active while IgH completes its rescan and configuration
reattachment. Acceptance remains strict: both independent domains must have
complete WKC=3, both drives must report DS402 Operation Enabled in CSP mode,
the master must report two responding slaves, and **both** per-slave health
states must be Present for ten consecutive samples. The trace also prints
the peer's configuration/state-read validity, not only the downstream target.

This timing correction is based on the observed IgH rescan chronology; it does
not yet establish that the wire-level recovery test passes. The next Ubuntu
run must show the second scan has completed, both slave configs are online and
operational, and all ten stable acceptance samples have been reached.

The CI syntax/build gate compiles this mode with
`MCA_ENABLE_KICKCAT_SIM_CONTROL=1` and builds the pinned KickCAT core library.
That is not a substitute for the Ubuntu wire-level run. Communication-loss SIL
remains incomplete until both break and heal phases pass on the wire-level
runner.


This is KickCAT SIL only: vendor-specific fault semantics, distributed-clock
timing, real-drive fault behavior and physical HIL remain unvalidated. Peer
preservation in this simulator is not a safety-rated isolation function.


### Successful communication-fault SIL rerun (2026-11-05)

The rerun at `/tmp/mca-ethercat-sil-20261105-132751` passed the full two-slave link-break and link-heal scenario. CoE PDO mapping readback and two-drive Operation Enabled startup passed. Breaking link 0:1 isolated ring 1 for ten cycles (target WKC=0; peer WKC=3 and Operation Enabled). After healing, both independent domains returned to WKC=3, both drives were Operation Enabled in CSP mode, and both per-slave health states reached Present for ten consecutive samples. Recovery completed after 1,513 cycles (about 1.5 s at 1 ms cycle time). Both drives then returned to Switch On Disabled; final total WKC=6, two slaves responding, and both stations online/operational. The temporary AL=0x00/offline interval reappeared in the recovery trace, but the 5 s recovery window allowed the test to continue until the full health criteria passed. This verifies the pinned KickCAT SIL scenario only; real-drive HIL, vendor behavior, DC timing and physical cable validation remain outstanding.

### Structured CoE PDO readback artifact

When `MCA_ECAT_COE_READBACK=1`, the veth/KickCAT discovery script now
preserves the ordered SDO reads in `coe-pdo-rows.tsv` and validates/serializes
them with `scripts/ethercat_sil/coe_snapshot.py` into
`coe-pdo-snapshot.json`. The versioned JSON records each ring position and
the pinned Vendor ID/Product code, ordered `0x1C12`/`0x1C13` assignment
subindices and PDO indices, the SDO-reported assignment/PDO entry
cardinalities, and each PDO's ordered raw 32-bit mapping words. Padding entries
are retained. The serializer compares observed row counts against the reported
SDO cardinalities, so truncated TSV data cannot silently become a smaller valid
snapshot.

The serializer fails closed on missing/non-contiguous ring positions,
assignment gaps or duplicate PDO selections, unknown assignment references,
mapping group/entry reordering, duplicate real OD objects, malformed padding,
and missing mapping groups. It does not sort or repair malformed input.
The raw `coe-sdo-raw.log`, text readback and packet captures remain available
as independent audit evidence.

The artifact deliberately does **not** invent SyncManager or direction fields:
the CoE assignment/mapping SDO reads do not provide that association. Those
fields must come from ESI expectations and/or IgH's independent
SM→PDO→Entry discovery. The current serializer records the identity checked
by the preceding `ethercat slaves -v` topology gate; it is not a third-party
device identity attestation. Set `MCA_ECAT_ESI_PDO_SNAPSHOT` to a previously generated
`--emit-pdo-snapshot` JSON file to enable fail-closed parity gates. The CoE
stage writes `esi-coe-parity.json` and fails on identity, assignment-order,
PDO-index, entry-order or raw mapping-word mismatch; it requires
`MCA_ECAT_COE_READBACK=1`. When cyclic SIL is also enabled, the C++ test emits
an ordered `MCA_IGH_PDO_SNAPSHOT` record from the actual IgH discovery profile.
The script extracts and validates `igh-pdo-snapshot.json`, then writes
`esi-igh-parity.json` and fails on identity, SM, direction, PDO, Entry-order or
raw mapping-word mismatch. IgH's SM-major sequence is compared with the ESI
expectation grouped by SM while preserving PDO/Entry order within each SM.


The same combined SIL run now performs a third, independent CoE-to-IgH
cross-check when both MCA_ECAT_COE_READBACK=1 and
MCA_ECAT_CYCLIC_SIL=1 are enabled. The coe_igh_parity.py comparator joins
records by ring position, Vendor ID/Product code and PDO index, then compares
each PDO's ordered raw 32-bit mapping entries (including padding). It verifies
that both sources expose the same slave set and PDO set; it never derives CoE
assignment subindices from IgH's SM-major discovery order. The report is
coe-igh-parity.json, and a mismatch fails the SIL script. This direct
cross-check supplements rather than replaces the two ESI comparisons:
CoE does not expose SM/direction, while IgH does not expose assignment
objects/subindices. The three reports therefore prove separate, explicitly
bounded relationships, not a physical-drive HIL result.


### Unified three-way parity gate

When both `MCA_ECAT_COE_READBACK=1` and
`MCA_ECAT_CYCLIC_SIL=1` are enabled, the harness now requires an explicit
`MCA_ECAT_ESI_PDO_SNAPSHOT` input. It fails closed if that file or either
ESI pairwise report is missing. After the CoE-to-IgH comparison, the harness
runs `three_way_pdo_parity.py` and writes
`three-way-pdo-parity.json`. The consolidated report requires all three
pairwise reports to have the expected schema/source kind, cover exactly the
configured slave count, and agree on contiguous ring positions. Any pairwise
failure fails the combined SIL run. CoE-only and cyclic-only modes remain
available independently and do not claim three-way parity.


### Two-slave baseline SIL pass at 10 ms (2026-11-06)

The rerun at `/tmp/mca-ethercat-sil-20261106-214826` passed the complete
two-slave KickCAT baseline after setting the cyclic SIL default to
`MCA_ECAT_CYCLE_TIME_US=10000` (10 ms). Startup transport and per-slave
health became ready after 212 cycles. Both ring positions independently
reached CiA 402 Operation Enabled (Statusword `0x0037`, CSP Mode Display
`0x08`) and remained stable for ten samples; each independent domain
reported complete WKC=3. The final state showed both slaves online and
operational, both drives returned to Switch On Disabled (Statusword
`0x0040`), link up, two responding slaves, and total WKC=6.

All three PDO comparisons passed in the same run:
ESI-to-CoE assignment and ordered raw mapping parity,
ESI-to-IgH SyncManager/direction/PDO/Entry parity, and
CoE-to-IgH PDO index and ordered raw mapping parity. The unified
three-way parity gate also passed and emitted
`three-way-pdo-parity.json`. The IgH discovery snapshot showed the same
four RxPDO and four TxPDO mapping entries on both ring positions.

This run validates the two-slave KickCAT baseline at a 10 ms software
simulation cycle. It does not establish 1 ms real-time performance, vendor
drive behavior, independent per-drive fault handling, distributed-clock
accuracy, motion trajectory performance, or physical-drive HIL. The
discovery-time PREOP snapshot is not runtime AL-state evidence; runtime
OP/health acceptance comes from the cyclic SIL result above.


### CoE-only ESI profile for KickCAT fault-isolation SIL

The 2026-11-06 ring-1 FRA=3 run exposed unsupported mailbox services in the
pinned KickCAT simulator fixture. Its source ESI advertises both EoE and
CoE SDO Information, but the simulator responds to EoE mailbox requests with
`MBXERR_UNSUPPORTEDPROTOCOL` and does not reliably answer SDO Information
description requests. The IgH master consequently starts EoE handlers for
both stations and emits repeated SDO-description timeout diagnostics while
the test is trying to read PDO mapping objects.

The multi-slave fault-isolation runner now creates a temporary, SIL-only ESI
beside the KickCAT JSON fixture. It removes the unsupported `<EoE />`
capability and changes only `CoE SdoInfo` to `false`. CoE itself remains
advertised, including SDO, PDO assignment and PDO configuration support.
The helper validates the transformed XML and compares all `RxPdo`,
`TxPdo` and `Sm` declarations before/after; any mapping or SyncManager
change fails preparation. The simulator JSON fixtures point to this temporary
ESI, while ESI-to-CoE and three-way PDO parity continue to use the original,
unmodified device ESI as the independent expectation. Temporary ESI and JSON
fixtures are removed by the runner's cleanup trap.

This is a simulator capability declaration, not a modification to the
vendor's production ESI and not a relaxation of PDO readback. All assignment
counts, PDO indices, entry counts, ordered raw mapping words (including
padding), WKC, AL-OP and fault-isolation checks remain mandatory. The next
wire-level run must show no EoE interfaces/unsupported-protocol mailbox
errors, successful SDO reads through ring 1 TxPDO entries 1–4, and the complete
FRA=3 two-direction acceptance. Until that run passes, ring-1 fault isolation
remains unverified.

The transport runner also supports `MCA_ECAT_EXPECT_NO_EOE=1`; in this mode
it fails before CoE reads if any `eoeNsN` interface/handler is still present.
The multi-slave fault-isolation wrapper enables this assertion and does not
rely on bringing interfaces down as a substitute for disabling the advertised
protocol capability.
