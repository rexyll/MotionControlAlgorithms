# EtherCAT CiA402 fault-injection SIL

## Scope and evidence boundary

The IgH + KickCAT base SIL has passed one-slave CoE PDO-map readback,
cyclic PDO/WKC/AL-OP and the baseline CiA402 enable/disable sequence.
KickCAT's pinned DS402 simulator did not originally model Fault Reaction
Active, latched Fault or the controlword bit-7 rising-edge reset. Therefore
fault recovery must not be tested against the unmodified simulator.

This fixture patch changes only the external KickCAT test dependency. It is
not part of MCA's production driver and does not represent a vendor drive's
fault semantics, diagnostic register behavior or safety response.

Pinned KickCAT revision:
`leducp/KickCAT@cb2e1a192ac11ffaa58f08945053b8129abf94cb`

Patch helper:
`scripts/ethercat_sil/apply_kickcat_ds402_fault_patch.py`

## Apply and build

The user's existing checkout is used by default:

```bash
export KICKCAT_SOURCE_DIR="$HOME/workspace/MotionControlAlgorithms/scripts/ethercat_sil/KickCAT"
bash scripts/ethercat_sil/prepare_kickcat_fault_injection.sh

cmake --build "$KICKCAT_SOURCE_DIR/build" --target network_simulator -j"$(nproc)"

cat > "$KICKCAT_SOURCE_DIR/simulation/slave_configs/ecat402-drive-fault.json" <<'JSON'
{
  "esi": "ecat402-drive.xml",
  "ds402_motor": true,
  "fault_after_cycles": 250,
  "fault_reaction_cycles": 3
}
JSON
```

The helper refuses a different KickCAT revision, checks all source insertion
anchors before writing either file, applies the fixture once, and runs
`git diff --check`. The CI job compiles both modified KickCAT translation units.
It does not fetch dependencies or modify the MCA production build.

## Simulator configuration

The existing `ecat402-drive.json` may be copied to a separate fault fixture
and extended with:

```json
{
  "esi": "ecat402-drive.xml",
  "ds402_motor": true,
  "fault_after_cycles": 250,
  "fault_reaction_cycles": 3
}
```

- `fault_after_cycles`: one-shot fault trigger measured in calls to the
  simulated DS402 motor cyclic step; zero disables injection.
- `fault_reaction_cycles`: number of consecutive PDO cycles reporting
  Fault Reaction Active (minimum effective value is one).
- After the reaction interval, statusword reports Fault and remains latched.
- A new rising edge of controlword bit 7 (`0x0080`) clears the simulated
  fault to Switch On Disabled. Holding bit 7 high does not create another edge.
- The simulated motor plant is de-energized during Fault Reaction Active and
  Fault.

The patch exposes the sequence through the same PDO-mapped controlword and
statusword as the normal simulator. It does not inject an arbitrary statusword
into the master's process image.

## Run the wire-level fault gate

After applying the patch and rebuilding `network_simulator`, create a separate
fixture JSON (do not overwrite the baseline fixture) with the two fault
parameters, for example:

```json
{
  "esi": "ecat402-drive.xml",
  "ds402_motor": true,
  "fault_after_cycles": 250,
  "fault_reaction_cycles": 3
}
```

Run the same IgH/veth harness with the fault gate enabled:

```bash
sudo -E env \
  KICKCAT_SIMULATOR="$KICKCAT_SOURCE_DIR/build/simulation/network_simulator" \
  KICKCAT_SLAVE_CONFIG="$KICKCAT_SOURCE_DIR/simulation/slave_configs/ecat402-drive-fault.json" \
  MCA_ECAT_COE_READBACK=1 \
  MCA_ECAT_DISABLE_EOE=1 \
  MCA_ECAT_CYCLIC_SIL=1 \
  MCA_ECAT_FAULT_SIL=1 \
  MCA_ECAT_CYCLIC_TIMEOUT_SEC=60 \
  bash scripts/ethercat_sil/run_kickcat_igh_veth_discovery.sh
```

The harness reads the expected reaction-cycle count from the JSON and passes it
to the C++ acceptance executable. It maintains Operation Enabled until the
configured injection occurs, then verifies the exact Fault Reaction Active
duration, latched Fault, one-cycle `0x0080` reset pulse and Switch On Disabled
feedback. It records `FAULT_TRACE` and `FAULT_RESET_TRACE` lines in
`cyclic-pdo-wkc.log`. The script rejects fault mode unless cyclic SIL is also
enabled and the fixture contains a positive `fault_after_cycles`.

## Required wire-level acceptance

Do not mark the fault gate passed until a run against the patched
`network_simulator` records all of the following from the MCA IgH cyclic
process image:

1. normal enable sequence reaches Operation Enabled;
2. the configured cycle produces Fault Reaction Active statusword
   (`0x000F`) for the configured number of cycles;
3. the drive then reports latched Fault (`0x0008`);
4. no ordinary enable command or motion setpoint is issued while faulted;
5. an explicit reset request sends exactly one cycle of controlword
   `0x0080`, then clears bit 7;
6. statusword feedback returns to Switch On Disabled (`0x0040`);
7. the sequencer does not automatically re-enable the drive;
8. WKC/domain completeness and AL OP remain healthy throughout the
   application-layer DS402 fault sequence.

The test report must include the simulator commit, patch SHA, JSON config,
statusword/controlword per cycle around the injected event, WKC and AL state.
A patch-application CI pass is not a compiled simulator, wire-level SIL or
physical HIL pass.

## Recorded local wire-level SIL pass

Observed on 2026-11-04 with the pinned KickCAT revision
`cb2e1a192ac11ffaa58f08945053b8129abf94cb` and the patched
`network_simulator`. User-provided artifact directory:
`/tmp/mca-ethercat-sil-20261104-203349`.

- CoE RxPDO/TxPDO assignment and PDO-entry SDO readback: PASS.
- One slave discovered; the cyclic harness reported all 8 scalar PDO bindings.
- Startup transport became ready after 81 cycles.
- The fault trace showed statusword `0x000F` at cycles 341, 342 and 343,
  followed by latched Fault `0x0008` at cycle 344 (three FRA cycles exactly).
- The reset sequencer emitted `controlword=0x0080` with
  `pulse_sent=1`. Subsequent reset traces used controlword `0x0000`;
  statusword remained `0x0008` at cycles 346 and 347 and changed to
  Switch On Disabled `0x0040` at cycle 348.
- Final cyclic result: `cycles=349`, `statusword=0x0040`,
  `disabled_feedback=1`, `WKC=3`, `domain_complete=1`,
  slave `online=1`, `operational=1`, `AL=0x08 (OP)`.
- The harness reported:
  `PASS: wire-level CiA402 Fault Reaction Active -> Fault -> explicit one-cycle Fault Reset -> Switch On Disabled; drive was not re-enabled.`

This is a successful wire-level SIL result for the deterministic patched
KickCAT model. It is not a physical drive/HIL result. In particular, the
simulator's mode-display echo does not prove a real drive accepted CSP or
responded to motion commands.

The runner's `slaves-verbose.log` is captured during initial discovery,
before the cyclic harness activates the slave. Its `PREOP` state is therefore
a discovery-time snapshot, not a contradictory post-test state. Use the
cyclic harness's runtime health and AL-state trace for the in-test state.

## Limitations

The fault model is deterministic and intentionally simple. It does not model
fault codes in object 0x603F, emergency messages, manufacturer-specific reset
conditions, STO, thermal/current protection, fault reaction option codes,
drive watchdogs, or real motor coast/braking behavior. Those remain future
device-profile/HIL work.
