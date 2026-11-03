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
readonly KICKCAT_SLAVE_CONFIG="${KICKCAT_SLAVE_CONFIG:-}"
readonly ETHERCAT_CLI="${ETHERCAT_CLI:-ethercat}"
readonly COE_READBACK="${MCA_ECAT_COE_READBACK:-0}"
readonly ARTIFACT_DIR="${MCA_ECAT_ARTIFACT_DIR:-/tmp/mca-ethercat-sil-$(date +%Y%m%d-%H%M%S)}"

SIM_PID=""
TX_PID=""
RX_PID=""
LOADED_MASTER=0
LOADED_GENERIC=0
VETH_CREATED=0

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

# Read a typed CoE object through the IgH CLI. The CLI supports explicit
# --position and --type options; parse the first numeric token from its output.
read_sdo_value() {
  local index="$1" subindex="$2" type="$3" output token value
  output=$(timeout "$TIMEOUT_SEC" "$ETHERCAT_CLI" upload --position 0 \
    --type "$type" "$index" "$subindex" 2>&1) || {
      printf 'SDO read failed: %s:%s (%s)\n%s\n' \
        "$index" "$subindex" "$type" "$output" >&2
      return 1
    }
  {
    printf 'READ %s:%s type=%s\n' "$index" "$subindex" "$type"
    printf '%s\n' "$output"
  } >>"$ARTIFACT_DIR/coe-sdo-raw.log"
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
  local assignment_index="$1" label="$2"
  local pdo_count pdo_pos pdo_index entry_count entry_pos mapped
  local object_index object_subindex bit_length
  pdo_count=$(read_sdo_value "$assignment_index" 0 uint8) || return 1
  (( pdo_count > 0 && pdo_count <= 16 )) || {
    printf '%s assignment count out of range: %s\n' "$label" "$pdo_count" >&2
    return 1
  }
  printf '%s assignment 0x%s: %s PDO(s)\n' "$label" "${assignment_index#0x}" "$pdo_count"
  for ((pdo_pos = 1; pdo_pos <= pdo_count; ++pdo_pos)); do
    pdo_index=$(read_sdo_value "$assignment_index" "$pdo_pos" uint16) || return 1
    (( pdo_index > 0 )) || {
      printf '%s assignment contains zero PDO index at subindex %s\n' "$label" "$pdo_pos" >&2
      return 1
    }
    entry_count=$(read_sdo_value "$pdo_index" 0 uint8) || return 1
    (( entry_count > 0 && entry_count <= 64 )) || {
      printf 'PDO 0x%04x mapping entry count out of range: %s\n' "$pdo_index" "$entry_count" >&2
      return 1
    }
    printf '  PDO 0x%04x: %s mapped entries\n' "$pdo_index" "$entry_count"
    for ((entry_pos = 1; entry_pos <= entry_count; ++entry_pos)); do
      mapped=$(read_sdo_value "$pdo_index" "$entry_pos" uint32) || return 1
      object_index=$(((mapped >> 16) & 0xffff))
      object_subindex=$(((mapped >> 8) & 0xff))
      bit_length=$((mapped & 0xff))
      (( bit_length > 0 && bit_length <= 64 )) || {
        printf '  PDO 0x%04x entry %s has unsupported width %s\n' \
          "$pdo_index" "$entry_pos" "$bit_length" >&2
        return 1
      }
      printf '    %04x:%02x width=%u\n' \
        "$object_index" "$object_subindex" "$bit_length"
    done
  done
}

run_coe_readback() {
  printf 'Slave ring position 0 CoE PDO mapping readback\n'
  verify_coe_assignment 0x1C12 "RxPDO" || return 1
  verify_coe_assignment 0x1C13 "TxPDO" || return 1
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  if [[ -n "$TX_PID" ]]; then kill "$TX_PID" 2>/dev/null || true; wait "$TX_PID" 2>/dev/null || true; fi
  if [[ -n "$RX_PID" ]]; then kill "$RX_PID" 2>/dev/null || true; wait "$RX_PID" 2>/dev/null || true; fi
  if [[ -n "$SIM_PID" ]]; then kill -TERM "$SIM_PID" 2>/dev/null || true; wait "$SIM_PID" 2>/dev/null || true; fi
  if (( VETH_CREATED )); then ip link del "$MASTER_IF" 2>/dev/null || true; fi
  if (( LOADED_GENERIC )); then modprobe -r ec_generic 2>/dev/null || true; fi
  if (( LOADED_MASTER )); then modprobe -r ec_master 2>/dev/null || true; fi
  printf 'Artifacts: %s\n' "$ARTIFACT_DIR"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

[[ $EUID -eq 0 ]] || fail "run as root (sudo -E) because veth, IgH modules and AF_PACKET need privileges"
[[ -n "$KICKCAT_SIMULATOR" && -x "$KICKCAT_SIMULATOR" ]] || fail "set KICKCAT_SIMULATOR to the executable network_simulator"
[[ -n "$KICKCAT_SLAVE_CONFIG" && -f "$KICKCAT_SLAVE_CONFIG" ]] || fail "set KICKCAT_SLAVE_CONFIG to KickCAT's ecat402-drive.json"
command -v ip >/dev/null || fail "iproute2 (ip) is required"
command -v modprobe >/dev/null || fail "kmod (modprobe) is required"
command -v modinfo >/dev/null || fail "kmod (modinfo) is required"
command -v tcpdump >/dev/null || fail "tcpdump is required to prove bidirectional EtherCAT frames"
command -v timeout >/dev/null || fail "coreutils timeout is required"
command -v "$ETHERCAT_CLI" >/dev/null || fail "IgH ethercat CLI not found: $ETHERCAT_CLI"
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
{
  uname -a
  printf '\n'
  "$KICKCAT_SIMULATOR" --help 2>&1 | head -40 || true
  printf '\n'
  printf 'master_if=%s\nsim_if=%s\nmaster_mac=%s\nsim_mac=%s\n' "$MASTER_IF" "$SIM_IF" "$MASTER_MAC" "$SIM_MAC"
} >"$ARTIFACT_DIR/environment.txt"

ip link add "$MASTER_IF" type veth peer name "$SIM_IF"
VETH_CREATED=1
ip link set dev "$MASTER_IF" address "$MASTER_MAC"
ip link set dev "$SIM_IF" address "$SIM_MAC"
ip link set dev "$MASTER_IF" up
ip link set dev "$SIM_IF" up

# IgH selects its master device by the main_devices MAC, not by interface name.
modprobe ec_master "main_devices=$MASTER_MAC"
LOADED_MASTER=1
modprobe ec_generic
LOADED_GENERIC=1

# KickCAT uses its Linux PF_PACKET/SOCK_RAW backend on the peer veth endpoint.
"$KICKCAT_SIMULATOR" -i "$SIM_IF" -s "$KICKCAT_SLAVE_CONFIG" \
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

# IgH's idle master FSM scans automatically. Poll until the single simulator
# slave is discovered; do not treat CLI success with zero slaves as acceptance.
deadline=$((SECONDS + TIMEOUT_SEC))
found=0
while (( SECONDS < deadline )); do
  "$ETHERCAT_CLI" slaves -v >"$ARTIFACT_DIR/slaves-verbose.log" 2>&1 || true
  count=$("$ETHERCAT_CLI" slaves 2>/dev/null | awk '
    NF >= 4 && $1 ~ /^[0-9]+$/ && $2 ~ /^[0-9]+:[0-9]+$/ { n++ }
    END { print n+0 }')
  if [[ "$count" == "1" ]] &&
     grep -Eiq 'Vendor Id:[[:space:]]*0x0*29c' "$ARTIFACT_DIR/slaves-verbose.log" &&
     grep -Eiq 'Product code:[[:space:]]*0x0*3b11002' "$ARTIFACT_DIR/slaves-verbose.log" &&
     grep -Eiq 'Revision number:[[:space:]]*0x0*50005' "$ARTIFACT_DIR/slaves-verbose.log"; then
    found=1
    break
  fi
  sleep 1
done

if (( ! found )); then
  "$ETHERCAT_CLI" master >"$ARTIFACT_DIR/master-status.log" 2>&1 || true
  cat "$ARTIFACT_DIR/master-status.log" >&2 || true
  cat "$ARTIFACT_DIR/slaves-verbose.log" >&2 || true
  cat "$ARTIFACT_DIR/kickcat-simulator.log" >&2 || true
  fail "IgH did not discover exactly one slave with the pinned ESI identity"
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
  if ! run_coe_readback >"$ARTIFACT_DIR/coe-pdo-readback.txt"; then
    cat "$ARTIFACT_DIR/coe-pdo-readback.txt" >&2 || true
    fail "CoE PDO assignment/mapping readback failed"
  fi
  printf '\nPASS: CoE SDO readback of RxPDO/TxPDO assignments and PDO entries.\n'
fi

cat "$ARTIFACT_DIR/slaves-verbose.log"
printf '\nPASS: IgH ec_generic <-> veth <-> KickCAT raw-socket slave discovery and bidirectional EtherCAT frame exchange.\n'
if [[ "$COE_READBACK" == "1" ]]; then
  printf 'NOTE: CoE PDO objects were read independently; this still does not validate cyclic PDO/WKC, CiA402 motion, DC timing or HIL.\n'
else
  printf 'NOTE: Transport/discovery only. Set MCA_ECAT_COE_READBACK=1 to read CoE PDO assignment/mapping objects; cyclic PDO/WKC, CiA402 motion, DC timing and HIL remain unvalidated.\n'
fi
