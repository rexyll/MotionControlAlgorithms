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

cat "$ARTIFACT_DIR/slaves-verbose.log"
printf '\nPASS: IgH ec_generic <-> veth <-> KickCAT raw-socket slave discovery and bidirectional EtherCAT frame exchange.\n'
printf 'NOTE: This is transport/discovery SIL only; it does not validate PDO, WKC, CoE SDO, CiA402 motion, DC timing or HIL.\n'
