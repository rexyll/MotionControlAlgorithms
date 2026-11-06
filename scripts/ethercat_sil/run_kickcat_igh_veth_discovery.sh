#!/usr/bin/env bash
set -Eeuo pipefail

# Opt-in, destructive-to-nothing-by-default IgH/KickCAT discovery smoke test.
# It creates only a temporary veth pair and loads IgH modules only when absent.
# Existing IgH module instances are deliberately rejected to avoid stealing a NIC.

readonly MASTER_IF="${MCA_ECAT_MASTER_IF:-mcaecat0}"
readonly SIM_IF="${MCA_ECAT_SIM_IF:-mcaecat1}"
readonly MASTER_MAC="${MCA_ECAT_MASTER_MAC:-02:4d:43:41:45:01}"
readonly SIM_MAC="${MCA_ECAT_SIM_MAC:-02:4d:43:41:45:02}"
readonly TIMEOUT_SEC="${MCA_ECAT_TIMEOUT_SEC:-20}"
readonly KICKCAT_SIMULATOR="${KICKCAT_SIMULATOR:-}"
readonly KICKCAT_SOURCE_DIR="${KICKCAT_SOURCE_DIR:-}"
readonly KICKCAT_SLAVE_CONFIG="${KICKCAT_SLAVE_CONFIG:-}"
readonly KICKCAT_SLAVE_CONFIGS="${KICKCAT_SLAVE_CONFIGS:-}"
readonly EXPECTED_SLAVES_OVERRIDE="${MCA_ECAT_EXPECTED_SLAVES:-}"
readonly ETHERCAT_CLI="${ETHERCAT_CLI:-ethercat}"
readonly COE_READBACK="${MCA_ECAT_COE_READBACK:-0}"
readonly ESI_PDO_SNAPSHOT="${MCA_ECAT_ESI_PDO_SNAPSHOT:-}"
readonly IGH_DEBUG_LEVEL="${MCA_ECAT_DEBUG_LEVEL:-0}"
readonly DISABLE_EOE="${MCA_ECAT_DISABLE_EOE:-0}"
readonly EXPECT_NO_EOE="${MCA_ECAT_EXPECT_NO_EOE:-0}"
readonly CYCLIC_SIL="${MCA_ECAT_CYCLIC_SIL:-0}"
readonly FAULT_SIL="${MCA_ECAT_FAULT_SIL:-0}"
readonly MULTI_FAULT_SIL="${MCA_ECAT_MULTI_FAULT_SIL:-0}"
readonly COMM_FAULT_SIL="${MCA_ECAT_COMM_FAULT_SIL:-0}"
readonly CYCLIC_TIMEOUT_SEC="${MCA_ECAT_CYCLIC_TIMEOUT_SEC:-20}"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
readonly ARTIFACT_DIR="${MCA_ECAT_ARTIFACT_DIR:-/tmp/mca-ethercat-sil-$(date +%Y%m%d-%H%M%S)}"

SIM_PID=""
TX_PID=""
RX_PID=""
COE_CAPTURE_PID=""
SIM_CONTROL_CLI=""
SIM_CONTROL_SHM=""
SIM_CONTROL_CREATED=0
LOADED_MASTER=0
LOADED_GENERIC=0
VETH_CREATED=0

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

capture_kernel_diagnostics() {
  local label="$1"
  {
    printf 'capture=%s\n' "$label"
    date --iso-8601=ns 2>&1 || date
    printf '\n--- IgH/EtherCAT kernel messages (last 500) ---\n'
    dmesg --time-format iso 2>&1 | tail -n 500 || true
  } >"$ARTIFACT_DIR/kernel-$label.log" 2>&1 || true
  {
    printf 'capture=%s\n' "$label"
    dmesg --time-format iso 2>&1 |
      grep -Ei 'EtherCAT|ec_master|ec_generic|SDO|mailbox|CoE|EoE' |
      tail -n 250 || true
  } >"$ARTIFACT_DIR/kernel-$label-ethercat.log" 2>&1 || true
} 

capture_runtime_host_diagnostics() {
  local label="$1" iface
  {
    printf 'capture=%s\n' "$label"
    date --iso-8601=ns 2>&1 || date
    printf '\n--- veth interface counters ---\n'
    for iface in "$MASTER_IF" "$SIM_IF"; do
      ip -s link show dev "$iface" 2>&1 || true
    done
    printf '\n--- KickCAT simulator process ---\n'
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
      ps -p "$SIM_PID" -o pid=,stat=,psr=,pcpu=,etime=,nlwp=,comm= 2>&1 || true
      if [[ -r "/proc/$SIM_PID/schedstat" ]]; then
        printf 'schedstat: '
        cat "/proc/$SIM_PID/schedstat" 2>&1 || true
      fi
      if [[ -r "/proc/$SIM_PID/status" ]]; then
        grep -E '^(State|Threads|voluntary_ctxt_switches|nonvoluntary_ctxt_switches):' \
          "/proc/$SIM_PID/status" 2>&1 || true
      fi
    else
      printf 'KickCAT simulator process is no longer running\n'
    fi
  } >"$ARTIFACT_DIR/runtime-host-$label.log" 2>&1 || true
}

# Read a typed CoE object through the IgH CLI. The CLI supports explicit
# --position and --type options; parse the first numeric token from its output.
read_sdo_value() {
  local index="$1" subindex="$2" type="$3" position="${4:-0}" output token value
  local command_status=0 started finished elapsed_ms started_wall finished_wall
  started_wall=$(date --iso-8601=ns)
  started=$(date +%s%N)
  if output=$(timeout "$TIMEOUT_SEC" "$ETHERCAT_CLI" upload --position "$position" \
      --type "$type" "$index" "$subindex" 2>&1); then
    command_status=0
  else
    command_status=$?
  fi
  finished=$(date +%s%N)
  finished_wall=$(date --iso-8601=ns)
  elapsed_ms=$(((finished - started) / 1000000))
  {
    printf 'READ position=%s %s:%s type=%s exit=%s elapsed_ms=%s start_epoch_ns=%s finish_epoch_ns=%s start_wall=%s finish_wall=%s\n' \
      "$position" "$index" "$subindex" "$type" "$command_status" "$elapsed_ms" "$started" "$finished" "$started_wall" "$finished_wall"
    printf '%s\n' "$output"
  } >>"$ARTIFACT_DIR/coe-sdo-raw.log"
  if (( command_status != 0 )); then
    printf 'SDO read failed: %s:%s (%s)\n%s\n' \
      "$index" "$subindex" "$type" "$output" >&2
    return 1
  fi
  token=$(printf '%s\n' "$output" | awk '
    NF {
      for (i = 1; i <= NF; ++i) {
        if ($i ~ /^0[xX][0-9a-fA-F]+$/ || $i ~ /^[0-9]+$/) {
          print $i
          exit
        }
      }
    }')
  [[ -n "$token" ]] || {
    printf 'Could not parse SDO value for %s:%s\n%s\n' \
      "$index" "$subindex" "$output" >&2
    return 1
  }
  if [[ "$token" == 0x* || "$token" == 0X* ]]; then
    value=$((token))
  else
    value=$((10#$token))
  fi
  printf '%s\n' "$value"
}

verify_coe_assignment() {
  local position="$1" assignment_index="$2" label="$3"
  local pdo_count pdo_pos pdo_index entry_count entry_pos mapped
  local object_index object_subindex bit_length
  pdo_count=$(read_sdo_value "$assignment_index" 0 uint8 "$position") || return 1
  (( pdo_count > 0 && pdo_count <= 16 )) || {
    printf '%s assignment count out of range: %s\n' "$label" "$pdo_count" >&2
    return 1
  }
  printf 'C\t%s\t%s\t%s\n' "$position" "$((assignment_index))" "$pdo_count" >>"$ARTIFACT_DIR/coe-pdo-rows.tsv"
  printf '%s assignment 0x%s: %s PDO(s)\n' "$label" "${assignment_index#0x}" "$pdo_count"
  for ((pdo_pos = 1; pdo_pos <= pdo_count; ++pdo_pos)); do
    pdo_index=$(read_sdo_value "$assignment_index" "$pdo_pos" uint16 "$position") || return 1
    (( pdo_index > 0 )) || {
      printf '%s assignment contains zero PDO index at subindex %s\n' "$label" "$pdo_pos" >&2
      return 1
    }
    printf 'A\t%s\t%s\t%s\t%s\n' "$position" "$((assignment_index))" "$pdo_pos" "$pdo_index" >>"$ARTIFACT_DIR/coe-pdo-rows.tsv"
    entry_count=$(read_sdo_value "$pdo_index" 0 uint8 "$position") || return 1
    (( entry_count > 0 && entry_count <= 64 )) || {
      printf 'PDO 0x%04x mapping entry count out of range: %s\n' "$pdo_index" "$entry_count" >&2
      return 1
    }
    printf 'P\t%s\t%s\t%s\t%s\t%s\n' "$position" "$((assignment_index))" "$pdo_pos" "$pdo_index" "$entry_count" >>"$ARTIFACT_DIR/coe-pdo-rows.tsv"
    printf '  PDO 0x%04x: %s mapped entries\n' "$pdo_index" "$entry_count"
    for ((entry_pos = 1; entry_pos <= entry_count; ++entry_pos)); do
      mapped=$(read_sdo_value "$pdo_index" "$entry_pos" uint32 "$position") || return 1
      object_index=$(((mapped >> 16) & 0xffff))
      object_subindex=$(((mapped >> 8) & 0xff))
      bit_length=$((mapped & 0xff))
      (( bit_length > 0 )) || {
        printf '  PDO 0x%04x entry %s has zero width %s\n' \
          "$pdo_index" "$entry_pos" "$bit_length" >&2
        return 1
      }
      printf 'M\t%s\t%s\t%s\t%s\t%s\t%s\n' "$position" "$((assignment_index))" "$pdo_pos" "$pdo_index" "$entry_pos" "$mapped" >>"$ARTIFACT_DIR/coe-pdo-rows.tsv"
      printf '    %04x:%02x width=%u\n' \
        "$object_index" "$object_subindex" "$bit_length"
    done
  done
}

run_coe_readback() {
  local position
  : >"$ARTIFACT_DIR/coe-pdo-rows.tsv"
  for ((position = 0; position < EXPECTED_SLAVES; ++position)); do
    printf 'S\t%s\t%s\t%s\n' "$position" "$((0x29c))" "$((0x03b11002))" >>"$ARTIFACT_DIR/coe-pdo-rows.tsv"
    printf 'Slave ring position %s CoE PDO mapping readback\n' "$position"
    verify_coe_assignment "$position" 0x1C12 "RxPDO" || return 1
    verify_coe_assignment "$position" 0x1C13 "TxPDO" || return 1
  done
}

start_coe_capture() {
  # Capture on the simulator endpoint. ec_generic binds the master endpoint
  # with its own packet socket and can prevent an additional AF_PACKET reader
  # there from seeing the EtherCAT stream (a 24-byte, header-only pcap).
  # A single reader on the unbound simulator endpoint observes both directions.
  tcpdump -U -i "$SIM_IF" -nn -s 0 -w \
    "$ARTIFACT_DIR/coe-master.pcap" \
    'ether proto 0x88a4' 2>"$ARTIFACT_DIR/coe-master-capture.log" &
  COE_CAPTURE_PID=$!
  sleep 0.5
  kill -0 "$COE_CAPTURE_PID" 2>/dev/null || {
    cat "$ARTIFACT_DIR/coe-master-capture.log" >&2 || true
    fail "EtherCAT master capture did not stay running"
  }
}

render_coe_capture() {
  local pcap="$1" text_log="$2"
  if [[ -s "$pcap" ]]; then
    tcpdump -nn -e -vvv -XX -r "$pcap" >"$text_log" 2>&1 || true
  else
    : >"$text_log"
  fi
}

stop_coe_capture() {
  if [[ -n "$COE_CAPTURE_PID" ]]; then
    kill -INT "$COE_CAPTURE_PID" 2>/dev/null || true
    wait "$COE_CAPTURE_PID" 2>/dev/null || true
    COE_CAPTURE_PID=""
  fi
  render_coe_capture "$ARTIFACT_DIR/coe-master.pcap" \
    "$ARTIFACT_DIR/coe-master.log"
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  stop_coe_capture
  if [[ -n "$TX_PID" ]]; then kill "$TX_PID" 2>/dev/null || true; wait "$TX_PID" 2>/dev/null || true; fi
  if [[ -n "$RX_PID" ]]; then kill "$RX_PID" 2>/dev/null || true; wait "$RX_PID" 2>/dev/null || true; fi
  if [[ -n "$SIM_PID" ]]; then kill -TERM "$SIM_PID" 2>/dev/null || true; wait "$SIM_PID" 2>/dev/null || true; fi
  if (( SIM_CONTROL_CREATED )) && [[ -x "$SIM_CONTROL_CLI" ]]; then
    "$SIM_CONTROL_CLI" unlink "$SIM_CONTROL_SHM" \
      >>"$ARTIFACT_DIR/sim-control-cleanup.log" 2>&1 || true
    SIM_CONTROL_CREATED=0
  fi
  if (( VETH_CREATED )); then ip link del "$MASTER_IF" 2>/dev/null || true; fi
  if (( LOADED_GENERIC )); then modprobe -r ec_generic 2>/dev/null || true; fi
  if (( LOADED_MASTER )); then modprobe -r ec_master 2>/dev/null || true; fi
  printf 'Artifacts: %s\n' "$ARTIFACT_DIR"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ -n "$ESI_PDO_SNAPSHOT" && "$COE_READBACK" != "1" ]]; then
  fail "MCA_ECAT_ESI_PDO_SNAPSHOT requires MCA_ECAT_COE_READBACK=1"
fi
if [[ "$COE_READBACK" == "1" && "$CYCLIC_SIL" == "1" ]]; then
  [[ -n "$ESI_PDO_SNAPSHOT" ]] || fail "combined CoE + cyclic SIL requires MCA_ECAT_ESI_PDO_SNAPSHOT for the complete three-way parity gate"
  [[ -f "$ESI_PDO_SNAPSHOT" ]] || fail "ESI PDO snapshot not found: $ESI_PDO_SNAPSHOT"
fi
[[ $EUID -eq 0 ]] || fail "run as root (sudo -E) because veth, IgH modules and AF_PACKET need privileges"
[[ -n "$KICKCAT_SIMULATOR" && -x "$KICKCAT_SIMULATOR" ]] || fail "set KICKCAT_SIMULATOR to the executable network_simulator"
SLAVE_CONFIGS=()
if [[ -n "$KICKCAT_SLAVE_CONFIGS" ]]; then
  read -r -a SLAVE_CONFIGS <<< "$KICKCAT_SLAVE_CONFIGS"
elif [[ -n "$KICKCAT_SLAVE_CONFIG" ]]; then
  SLAVE_CONFIGS=("$KICKCAT_SLAVE_CONFIG")
fi
(("${#SLAVE_CONFIGS[@]}" > 0)) || fail "set KICKCAT_SLAVE_CONFIG or KICKCAT_SLAVE_CONFIGS"
for slave_config in "${SLAVE_CONFIGS[@]}"; do
  [[ -f "$slave_config" ]] || fail "KickCAT slave config not found: $slave_config"
done
if [[ -n "$EXPECTED_SLAVES_OVERRIDE" ]]; then
  [[ "$EXPECTED_SLAVES_OVERRIDE" =~ ^[1-9][0-9]*$ ]] || fail "MCA_ECAT_EXPECTED_SLAVES must be a positive integer"
  EXPECTED_SLAVES=$((10#$EXPECTED_SLAVES_OVERRIDE))
  (( EXPECTED_SLAVES == ${#SLAVE_CONFIGS[@]} )) || fail "expected slave count must match the number of KickCAT config files"
else
  EXPECTED_SLAVES=${#SLAVE_CONFIGS[@]}
fi
command -v ip >/dev/null || fail "iproute2 (ip) is required"
command -v modprobe >/dev/null || fail "kmod (modprobe) is required"
command -v modinfo >/dev/null || fail "kmod (modinfo) is required"
command -v tcpdump >/dev/null || fail "tcpdump is required to prove bidirectional EtherCAT frames"
if [[ "$DISABLE_EOE" != "0" && "$DISABLE_EOE" != "1" ]]; then
  fail "MCA_ECAT_DISABLE_EOE must be 0 or 1"
fi
if [[ "$EXPECT_NO_EOE" != "0" && "$EXPECT_NO_EOE" != "1" ]]; then
  fail "MCA_ECAT_EXPECT_NO_EOE must be 0 or 1"
fi
if [[ "$FAULT_SIL" != "0" && "$FAULT_SIL" != "1" ]]; then
  fail "MCA_ECAT_FAULT_SIL must be 0 or 1"
fi
if [[ "$MULTI_FAULT_SIL" != "0" && "$MULTI_FAULT_SIL" != "1" ]]; then
  fail "MCA_ECAT_MULTI_FAULT_SIL must be 0 or 1"
fi
if [[ "$COMM_FAULT_SIL" != "0" && "$COMM_FAULT_SIL" != "1" ]]; then
  fail "MCA_ECAT_COMM_FAULT_SIL must be 0 or 1"
fi
if [[ "$COMM_FAULT_SIL" == "1" ]]; then
  [[ "$EXPECTED_SLAVES" == "2" ]] || fail "multi-slave communication fault SIL requires exactly two configured slaves"
  [[ "$CYCLIC_SIL" == "1" ]] || fail "multi-slave communication fault SIL requires MCA_ECAT_CYCLIC_SIL=1"
  [[ "$FAULT_SIL" == "0" && "$MULTI_FAULT_SIL" == "0" ]] || fail "link-break SIL cannot be combined with DS402 fault injection"
  [[ -n "$KICKCAT_SOURCE_DIR" && -f "$KICKCAT_SOURCE_DIR/lib/include/kickcat/SimulatorControl.h" ]] || fail "set KICKCAT_SOURCE_DIR to the pinned KickCAT checkout"
  [[ -f "$KICKCAT_SOURCE_DIR/build/lib/libkickcat.a" ]] || fail "build the pinned KickCAT libkickcat.a before communication fault SIL"
  simulator_help="$("$KICKCAT_SIMULATOR" --help 2>&1)" || fail "KICKCAT_SIMULATOR --help failed"
  [[ "$simulator_help" == *"--control"* ]] || fail "KICKCAT_SIMULATOR does not expose the --control shared-memory option"
fi
if [[ "$MULTI_FAULT_SIL" == "1" ]]; then
  [[ "$EXPECTED_SLAVES" == "2" ]] || fail "multi-slave fault SIL requires exactly two configured slaves"
  [[ "$CYCLIC_SIL" == "1" ]] || fail "multi-slave fault SIL requires MCA_ECAT_CYCLIC_SIL=1"
  [[ "$FAULT_SIL" == "0" ]] || fail "single-slave and multi-slave fault modes cannot be combined"
  [[ "${#SLAVE_CONFIGS[@]}" == "2" ]] || fail "multi-slave fault SIL requires two config files"
  [[ "${MCA_ECAT_MULTI_FAULT_RING_POSITION:-}" =~ ^[01]$ ]] || fail "set MCA_ECAT_MULTI_FAULT_RING_POSITION to 0 or 1"
  command -v python3 >/dev/null || fail "python3 is required to validate multi-slave fault fixtures"
  export MCA_ECAT_EXPECTED_FAULT_REACTION_CYCLES="$(
    python3 - "${SLAVE_CONFIGS[0]}" "${SLAVE_CONFIGS[1]}" "${MCA_ECAT_MULTI_FAULT_RING_POSITION}" <<'PY'
import json
import pathlib
import sys
configs = [json.loads(pathlib.Path(p).read_text(encoding="utf-8")) for p in sys.argv[1:3]]
target = int(sys.argv[3])
for slot, config in enumerate(configs):
    if not isinstance(config, dict) or not bool(config.get("ds402_motor", False)):
        raise SystemExit(f"slave {slot} must be a DS402 motor fixture")
    after = int(config.get("fault_after_cycles", 0))
    if slot == target:
        if after <= 0:
            raise SystemExit("selected slave must have positive fault_after_cycles")
        reaction = max(1, int(config.get("fault_reaction_cycles", 3)))
    elif after != 0:
        raise SystemExit("non-target slave must have fault_after_cycles=0")
print(reaction)
PY
  )" || fail "invalid per-slave multi-fault fixture JSON"
fi
if [[ "$FAULT_SIL" == "1" ]]; then
  [[ "$EXPECTED_SLAVES" == "1" ]] || fail "MCA_ECAT_FAULT_SIL still uses the single-slave cyclic harness"
  [[ "$CYCLIC_SIL" == "1" ]] || fail "MCA_ECAT_FAULT_SIL requires MCA_ECAT_CYCLIC_SIL=1"
  command -v python3 >/dev/null || fail "python3 is required to validate fault fixture JSON"
  export MCA_ECAT_EXPECTED_FAULT_REACTION_CYCLES="$(
    python3 -c 'import json,sys; c=json.load(open(sys.argv[1])); n=int(c.get("fault_reaction_cycles", 3)); assert int(c.get("fault_after_cycles", 0)) > 0; print(max(1,n))' "${SLAVE_CONFIGS[0]}"
  )" || fail "fault SIL requires positive fault_after_cycles and a valid fault_reaction_cycles"
fi
command -v timeout >/dev/null || fail "coreutils timeout is required"
command -v "$ETHERCAT_CLI" >/dev/null || fail "IgH ethercat CLI not found: $ETHERCAT_CLI"
[[ "$IGH_DEBUG_LEVEL" =~ ^[0-9]+$ ]] || fail "MCA_ECAT_DEBUG_LEVEL must be a non-negative integer"
modinfo ec_master >/dev/null 2>&1 || fail "IgH ec_master module is not installed for the running kernel"
modinfo ec_generic >/dev/null 2>&1 || fail "IgH ec_generic module is not installed for the running kernel"

[[ "$MASTER_IF" != "$SIM_IF" ]] || fail "master and simulator interfaces must differ"
(( ${#MASTER_IF} <= 15 && ${#SIM_IF} <= 15 )) || fail "Linux interface names must be at most 15 characters"
! ip link show "$MASTER_IF" >/dev/null 2>&1 || fail "interface already exists: $MASTER_IF"
! ip link show "$SIM_IF" >/dev/null 2>&1 || fail "interface already exists: $SIM_IF"
[[ ! -d /sys/module/ec_master ]] || fail "ec_master is already loaded; stop existing EtherCAT services and retry"
[[ ! -d /sys/module/ec_generic ]] || fail "ec_generic is already loaded; stop existing EtherCAT services and retry"

mkdir -p "$ARTIFACT_DIR"
chmod 700 "$ARTIFACT_DIR"
if [[ "$COMM_FAULT_SIL" == "1" ]]; then
  SIM_CONTROL_CLI="$ARTIFACT_DIR/mca-ethercat-control-shm"
  SIM_CONTROL_SHM="/mca_ethercat_${BASHPID}_$(date +%s)"
  if ! c++ -std=c++17 -Wall -Wextra -Wpedantic -Wconversion -Wshadow \
      -I"$KICKCAT_SOURCE_DIR/lib/include" \
      "$REPO_ROOT/tests/integration/ethercat_kickcat_sim_control_shm.cpp" \
      "$KICKCAT_SOURCE_DIR/build/lib/libkickcat.a" -pthread -lrt \
      -o "$SIM_CONTROL_CLI" \
      >"$ARTIFACT_DIR/sim-control-build.log" 2>&1; then
    cat "$ARTIFACT_DIR/sim-control-build.log" >&2
    fail "KickCAT simulator control shared-memory helper build failed"
  fi
  if ! "$SIM_CONTROL_CLI" init "$SIM_CONTROL_SHM" \
      >"$ARTIFACT_DIR/sim-control-init.log" 2>&1; then
    cat "$ARTIFACT_DIR/sim-control-init.log" >&2
    fail "KickCAT simulator control channel initialisation failed"
  fi
  SIM_CONTROL_CREATED=1
  export MCA_ECAT_SIM_CONTROL_SHM="$SIM_CONTROL_SHM"
fi
{
  uname -a
  printf '\n'
  "$KICKCAT_SIMULATOR" --help 2>&1 | head -40 || true
  printf '\n'
  printf 'master_if=%s\nsim_if=%s\nmaster_mac=%s\nsim_mac=%s\nexpected_slaves=%s\n' "$MASTER_IF" "$SIM_IF" "$MASTER_MAC" "$SIM_MAC" "$EXPECTED_SLAVES"
  printf 'slave_config=%s\n' "${SLAVE_CONFIGS[*]}"
  printf 'igh_debug_level=%s\ncoe_readback=%s\ndisable_eoe=%s\nexpect_no_eoe=%s\n' "$IGH_DEBUG_LEVEL" "$COE_READBACK" "$DISABLE_EOE" "$EXPECT_NO_EOE"
  printf 'comm_fault_sil=%s\nsim_control_shm=%s\nkickcat_source_dir=%s\n' "$COMM_FAULT_SIL" "$SIM_CONTROL_SHM" "$KICKCAT_SOURCE_DIR"
  printf 'cycle_time_us=%s\n' "${MCA_ECAT_CYCLE_TIME_US:-10000}"
  printf '\nIgH module parameters:\n'
  modinfo ec_master 2>&1 | grep -E '^(version|parm):' || true
} >"$ARTIFACT_DIR/environment.txt"

ip link add "$MASTER_IF" type veth peer name "$SIM_IF"
VETH_CREATED=1
ip link set dev "$MASTER_IF" address "$MASTER_MAC"
ip link set dev "$SIM_IF" address "$SIM_MAC"
ip link set dev "$MASTER_IF" up
ip link set dev "$SIM_IF" up

# IgH selects its master device by the main_devices MAC, not by interface name.
modprobe ec_master "main_devices=$MASTER_MAC" "debug_level=$IGH_DEBUG_LEVEL"
LOADED_MASTER=1
modprobe ec_generic
LOADED_GENERIC=1

# KickCAT uses its Linux PF_PACKET/SOCK_RAW backend on the peer veth endpoint.
SIM_ARGS=(-i "$SIM_IF")
if [[ "$COMM_FAULT_SIL" == "1" ]]; then
  SIM_ARGS+=(--control "$SIM_CONTROL_SHM")
fi
SIM_ARGS+=(-s "${SLAVE_CONFIGS[@]}")
"$KICKCAT_SIMULATOR" "${SIM_ARGS[@]}" \
  >"$ARTIFACT_DIR/kickcat-simulator.log" 2>&1 &
SIM_PID=$!
sleep 1
kill -0 "$SIM_PID" 2>/dev/null || {
  cat "$ARTIFACT_DIR/kickcat-simulator.log" >&2
  fail "KickCAT simulator exited during startup"
}

# Capture one master->simulator frame and one simulator->master response.
timeout "$((TIMEOUT_SEC + 5))" tcpdump -U -i "$MASTER_IF" -Q out -nn -e \
  'ether proto 0x88a4' -c 1 >"$ARTIFACT_DIR/master-tx.log" 2>&1 &
TX_PID=$!
timeout "$((TIMEOUT_SEC + 5))" tcpdump -U -i "$MASTER_IF" -Q in -nn -e \
  'ether proto 0x88a4' -c 1 >"$ARTIFACT_DIR/master-rx.log" 2>&1 &
RX_PID=$!

# IgH's idle master FSM scans automatically. Poll until the configured number
# of simulator slaves is discovered; do not accept a partial topology.
deadline=$((SECONDS + TIMEOUT_SEC))
found=0
stable_preop_samples=0
while (( SECONDS < deadline )); do
  # Use one CLI snapshot per poll. The previous count+verbose pair queried the
  # master twice while its idle FSM could still be scanning, and accepted
  # slaves in INIT. That allowed CoE uploads to race scan/mailbox traffic.
  "$ETHERCAT_CLI" slaves -v >"$ARTIFACT_DIR/slaves-verbose.log" 2>&1 || true
  count=$(awk '/^=== Master 0, Slave / { n++ } END { print n+0 }' "$ARTIFACT_DIR/slaves-verbose.log")
  preop_count=$(grep -c '^State: PREOP$' "$ARTIFACT_DIR/slaves-verbose.log" || true)
  vendor_count=$(grep -Eic 'Vendor Id:[[:space:]]*0x0*29c' "$ARTIFACT_DIR/slaves-verbose.log" || true)
  product_count=$(grep -Eic 'Product code:[[:space:]]*0x0*3b11002' "$ARTIFACT_DIR/slaves-verbose.log" || true)
  revision_count=$(grep -Eic 'Revision number:[[:space:]]*0x0*50005' "$ARTIFACT_DIR/slaves-verbose.log" || true)
  if [[ "$count" == "$EXPECTED_SLAVES" ]] &&
     [[ "$preop_count" == "$EXPECTED_SLAVES" ]] &&
     [[ "$vendor_count" == "$EXPECTED_SLAVES" ]] &&
     [[ "$product_count" == "$EXPECTED_SLAVES" ]] &&
     [[ "$revision_count" == "$EXPECTED_SLAVES" ]]; then
    stable_preop_samples=$((stable_preop_samples + 1))
    if (( stable_preop_samples >= 3 )); then
      found=1
      break
    fi
  else
    stable_preop_samples=0
  fi
  sleep 1
done

if (( ! found )); then
  "$ETHERCAT_CLI" master >"$ARTIFACT_DIR/master-status.log" 2>&1 || true
  cat "$ARTIFACT_DIR/master-status.log" >&2 || true
  cat "$ARTIFACT_DIR/slaves-verbose.log" >&2 || true
  cat "$ARTIFACT_DIR/kickcat-simulator.log" >&2 || true
  fail "IgH did not discover exactly $EXPECTED_SLAVES slaves with the pinned ESI identity and three consecutive all-PREOP snapshots"
fi

if ! wait "$TX_PID"; then
  cat "$ARTIFACT_DIR/master-tx.log" >&2
  fail "no outbound EtherCAT frame observed on $MASTER_IF"
fi
TX_PID=""
if ! wait "$RX_PID"; then
  cat "$ARTIFACT_DIR/master-rx.log" >&2
  fail "no inbound EtherCAT response observed on $MASTER_IF"
fi
RX_PID=""

if [[ "$COE_READBACK" == "1" ]]; then
  # Record EoE handler/interface state. EoE can generate mailbox traffic
  # independently of CoE and shares the slave mailbox Sync Managers.
  {
    ip -o link show
    "$ETHERCAT_CLI" eoe 2>&1 || true
  } >"$ARTIFACT_DIR/eoe-before-coe.log" 2>&1
  if [[ "$EXPECT_NO_EOE" == "1" ]] &&
     grep -Eq '(^|[[:space:]])eoe[0-9]+s[0-9]+([[:space:]]|$)' "$ARTIFACT_DIR/eoe-before-coe.log"; then
    cat "$ARTIFACT_DIR/eoe-before-coe.log" >&2
    fail "CoE-only SIL ESI was expected to suppress all IgH EoE handlers"
  fi
  if [[ "$DISABLE_EOE" == "1" ]]; then
    # Opt-in isolation experiment: close auto-created EoE interfaces.
    while IFS= read -r eoe_if; do
      [[ "$eoe_if" == eoe* ]] || continue
      ip link set dev "$eoe_if" down
    done < <(ip -o link show | awk -F': ' '{sub(/@.*/, "", $2); print $2}')
    {
      ip -o link show
      "$ETHERCAT_CLI" eoe 2>&1 || true
    } >"$ARTIFACT_DIR/eoe-after-disable.log" 2>&1
  fi
  "$ETHERCAT_CLI" master >"$ARTIFACT_DIR/master-before-coe.log" 2>&1 || true
  # Reuse the last validated all-PREOP snapshot; do not launch another scan
  # immediately before mailbox readback.
  cp "$ARTIFACT_DIR/slaves-verbose.log" "$ARTIFACT_DIR/slaves-before-coe.log"
  capture_kernel_diagnostics "before-coe"
  capture_runtime_host_diagnostics "before-coe"
  start_coe_capture
  if ! run_coe_readback >"$ARTIFACT_DIR/coe-pdo-readback.txt"; then
    stop_coe_capture
    capture_kernel_diagnostics "after-coe-failure"
    capture_runtime_host_diagnostics "after-coe-failure"
    {
      date --iso-8601=ns 2>&1 || date
      printf '\n--- ethercat master ---\n'
      "$ETHERCAT_CLI" master 2>&1 || true
      printf '\n--- ethercat slaves -v ---\n'
      "$ETHERCAT_CLI" slaves -v 2>&1 || true
      printf '\n--- ip -s link ---\n'
      ip -s link show 2>&1 || true
      printf '\n--- EoE interfaces ---\n'
      ip -o link show 2>&1 | grep -E 'eoe[0-9]+s[0-9]+' || true
    } >"$ARTIFACT_DIR/coe-failure-state.log" 2>&1
    cat "$ARTIFACT_DIR/coe-pdo-readback.txt" >&2 || true
    fail "CoE PDO assignment/mapping readback failed"
  fi
  stop_coe_capture
  command -v python3 >/dev/null || fail "python3 is required to serialize CoE PDO snapshots"
  if ! python3 "$SCRIPT_DIR/coe_snapshot.py" "$ARTIFACT_DIR/coe-pdo-rows.tsv" "$ARTIFACT_DIR/coe-pdo-snapshot.json" "$EXPECTED_SLAVES"; then
    fail "CoE PDO snapshot validation/serialization failed"
  fi
  if [[ -n "$ESI_PDO_SNAPSHOT" ]]; then
    [[ -f "$ESI_PDO_SNAPSHOT" ]] || fail "ESI PDO snapshot not found: $ESI_PDO_SNAPSHOT"
    if ! python3 "$SCRIPT_DIR/esi_coe_parity.py" "$ESI_PDO_SNAPSHOT" \
        "$ARTIFACT_DIR/coe-pdo-snapshot.json" "$ARTIFACT_DIR/esi-coe-parity.json"; then
      cat "$ARTIFACT_DIR/esi-coe-parity.json" >&2 2>/dev/null || true
      fail "ESI-to-CoE PDO assignment/mapping parity failed"
    fi
    printf 'PASS: ESI-to-CoE assignment and raw mapping parity; report: %s\n' \
      "$ARTIFACT_DIR/esi-coe-parity.json"
  fi
  {
    ip -o link show
    "$ETHERCAT_CLI" eoe 2>&1 || true
  } >"$ARTIFACT_DIR/eoe-after-coe.log" 2>&1
  printf '\nPASS: CoE SDO readback of RxPDO/TxPDO assignments and PDO entries; structured snapshot: %s\n' "$ARTIFACT_DIR/coe-pdo-snapshot.json"
fi

if [[ "$CYCLIC_SIL" == "1" ]]; then
  command -v python3 >/dev/null || fail "python3 is required to serialize IgH PDO snapshots"
  command -v cmake >/dev/null || fail "cmake is required for cyclic PDO SIL"
  command -v c++ >/dev/null || fail "a C++ compiler is required for cyclic PDO SIL"
  readonly CYCLIC_BUILD_DIR="$ARTIFACT_DIR/build-igh-cyclic"
  CYCLIC_TARGET=mca_ethercat_igh_kickcat_cyclic_sil
  if [[ "$EXPECTED_SLAVES" == "2" ]]; then
    CYCLIC_TARGET=mca_ethercat_igh_kickcat_multi_slave_cyclic_sil
  elif [[ "$EXPECTED_SLAVES" != "1" ]]; then
    fail "cyclic SIL supports one or two slaves; found $EXPECTED_SLAVES"
  fi
  CYCLIC_CMAKE_ARGS=(-DMCA_BUILD_TESTS=OFF -DMCA_BUILD_IGH_KICKCAT_SIL=ON)
  if [[ "$COMM_FAULT_SIL" == "1" ]]; then
    CYCLIC_CMAKE_ARGS+=(-DMCA_KICKCAT_SOURCE_DIR="$KICKCAT_SOURCE_DIR")
  fi
  if ! cmake -S "$REPO_ROOT" -B "$CYCLIC_BUILD_DIR" "${CYCLIC_CMAKE_ARGS[@]}" \
      >"$ARTIFACT_DIR/cyclic-cmake-configure.log" 2>&1; then
    cat "$ARTIFACT_DIR/cyclic-cmake-configure.log" >&2
    fail "real-IgH cyclic SIL target configuration failed (check ecrt.h, libethercat and Eigen3)"
  fi
  if ! cmake --build "$CYCLIC_BUILD_DIR" --target \
      "$CYCLIC_TARGET" --parallel 2 \
      >"$ARTIFACT_DIR/cyclic-cmake-build.log" 2>&1; then
    cat "$ARTIFACT_DIR/cyclic-cmake-build.log" >&2
    fail "real-IgH cyclic SIL target build failed"
  fi
  if ! timeout "$CYCLIC_TIMEOUT_SEC" \
      "$CYCLIC_BUILD_DIR/$CYCLIC_TARGET" \
      >"$ARTIFACT_DIR/cyclic-pdo-wkc.log" 2>&1; then
    cat "$ARTIFACT_DIR/cyclic-pdo-wkc.log" >&2
    # Preserve the final kernel and host evidence before cleanup unloads IgH
    # and removes the temporary veth pair.
    capture_kernel_diagnostics "after-cyclic-failure"
    capture_runtime_host_diagnostics "after-cyclic-failure"
    fail "cyclic PDO/WKC/OP SIL failed"
  fi
  cat "$ARTIFACT_DIR/cyclic-pdo-wkc.log"
  if ! python3 "$SCRIPT_DIR/igh_snapshot.py" "$ARTIFACT_DIR/cyclic-pdo-wkc.log" "$ARTIFACT_DIR/igh-pdo-snapshot.json"; then
    fail "IgH PDO discovery snapshot extraction/validation failed"
  fi
  if [[ -n "$ESI_PDO_SNAPSHOT" ]]; then
    if ! python3 "$SCRIPT_DIR/esi_igh_parity.py" "$ESI_PDO_SNAPSHOT" \
        "$ARTIFACT_DIR/igh-pdo-snapshot.json" "$ARTIFACT_DIR/esi-igh-parity.json"; then
      cat "$ARTIFACT_DIR/esi-igh-parity.json" >&2 2>/dev/null || true
      fail "ESI-to-IgH PDO mapping parity failed"
    fi
    printf 'PASS: ESI-to-IgH SM/PDO/Entry parity; report: %s\n' \
      "$ARTIFACT_DIR/esi-igh-parity.json"
  fi
fi


if [[ "$COE_READBACK" == "1" && "$CYCLIC_SIL" == "1" ]]; then
  [[ -n "$ESI_PDO_SNAPSHOT" ]] || fail "combined CoE + cyclic SIL requires MCA_ECAT_ESI_PDO_SNAPSHOT for the complete three-way parity gate"
  [[ -f "$ESI_PDO_SNAPSHOT" ]] || fail "ESI PDO snapshot not found: $ESI_PDO_SNAPSHOT"
  [[ -f "$ARTIFACT_DIR/esi-coe-parity.json" ]] || fail "ESI-to-CoE parity report is missing"
  [[ -f "$ARTIFACT_DIR/esi-igh-parity.json" ]] || fail "ESI-to-IgH parity report is missing"
  if ! python3 "$SCRIPT_DIR/coe_igh_parity.py" \
      "$ARTIFACT_DIR/coe-pdo-snapshot.json" \
      "$ARTIFACT_DIR/igh-pdo-snapshot.json" \
      "$ARTIFACT_DIR/coe-igh-parity.json"; then
    cat "$ARTIFACT_DIR/coe-igh-parity.json" >&2 2>/dev/null || true
    fail "CoE-to-IgH PDO mapping parity failed"
  fi
  printf 'PASS: CoE-to-IgH PDO index and ordered raw mapping parity; report: %s\n' \
    "$ARTIFACT_DIR/coe-igh-parity.json"
  if ! python3 "$SCRIPT_DIR/three_way_pdo_parity.py" \
      "$ARTIFACT_DIR/esi-coe-parity.json" \
      "$ARTIFACT_DIR/esi-igh-parity.json" \
      "$ARTIFACT_DIR/coe-igh-parity.json" \
      "$EXPECTED_SLAVES" "$ARTIFACT_DIR/three-way-pdo-parity.json"; then
    cat "$ARTIFACT_DIR/three-way-pdo-parity.json" >&2 2>/dev/null || true
    fail "unified ESI/CoE/IgH three-way PDO parity failed"
  fi
  printf 'PASS: unified ESI/CoE/IgH three-way PDO parity; report: %s\n' \
    "$ARTIFACT_DIR/three-way-pdo-parity.json"
fi


printf '\nDiscovery-time slave snapshot (captured before cyclic SIL; not runtime AL-state evidence):\n'
cat "$ARTIFACT_DIR/slaves-verbose.log"
printf '\nPASS: IgH ec_generic <-> veth <-> KickCAT raw-socket slave discovery and bidirectional EtherCAT frame exchange.\n'
if [[ "$CYCLIC_SIL" == "1" && "$COE_READBACK" == "1" ]]; then
  printf 'NOTE: CoE mapping readback and cyclic PDO/WKC/AL-OP passed; CiA402 motion, DC timing and real-slave HIL remain unvalidated.\n'
elif [[ "$CYCLIC_SIL" == "1" ]]; then
  printf 'NOTE: Cyclic PDO/WKC/AL-OP passed; independent CoE SDO readback, CiA402 motion, DC timing and real-slave HIL remain unvalidated.\n'
elif [[ "$COE_READBACK" == "1" ]]; then
  printf 'NOTE: CoE PDO objects were read independently; cyclic PDO/WKC, CiA402 motion, DC timing and HIL remain unvalidated.\n'
else
  printf 'NOTE: Transport/discovery only. Set MCA_ECAT_COE_READBACK=1 for CoE readback and MCA_ECAT_CYCLIC_SIL=1 for cyclic PDO/WKC/AL-OP validation.\n'
fi
