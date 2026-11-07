#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly RUNNER="$SCRIPT_DIR/run_kickcat_igh_veth_discovery.sh"
readonly KICKCAT_SOURCE_DIR="${KICKCAT_SOURCE_DIR:-$SCRIPT_DIR/KickCAT}"
readonly KICKCAT_SIMULATOR="${KICKCAT_SIMULATOR:-$KICKCAT_SOURCE_DIR/build/simulation/network_simulator}"
readonly BASE_CONFIG_INPUT="${KICKCAT_SLAVE_CONFIG:-$KICKCAT_SOURCE_DIR/simulation/slave_configs/ecat402-drive.json}"
readonly ARTIFACT_DIR="${MCA_ECAT_ARTIFACT_DIR:-/tmp/mca-ethercat-comm-fault-$(date +%Y%m%d-%H%M%S)}"

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || fail "run as root (sudo -E)"
[[ -x "$RUNNER" ]] || fail "veth discovery runner not found"
git -C "$KICKCAT_SOURCE_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1 || fail "KickCAT Git checkout not found: $KICKCAT_SOURCE_DIR"
[[ -x "$KICKCAT_SIMULATOR" ]] || fail "KickCAT network_simulator not found: $KICKCAT_SIMULATOR"
[[ -f "$BASE_CONFIG_INPUT" ]] || fail "base DS402 config not found: $BASE_CONFIG_INPUT"
[[ -f "$KICKCAT_SOURCE_DIR/lib/include/kickcat/SimulatorControl.h" ]] || fail "pinned KickCAT simulator-control API not found"
[[ -f "$KICKCAT_SOURCE_DIR/build/lib/libkickcat.a" ]] || fail "build KickCAT libkickcat.a first"
command -v python3 >/dev/null || fail "python3 is required to validate the DS402 fixture"

BASE_CONFIG="$(realpath -- "$BASE_CONFIG_INPUT")"
python3 - "$BASE_CONFIG" <<'PY'
import json
import pathlib
import sys

config = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if not isinstance(config, dict) or not bool(config.get("ds402_motor", False)):
    raise SystemExit("base fixture must be a DS402 motor configuration")
if int(config.get("fault_after_cycles", 0)) != 0:
    raise SystemExit("communication-fault SIL requires fault_after_cycles=0")
PY

# The combined CoE + cyclic gate requires an independent PDO expectation
# generated from the original ESI (not a protocol-trimmed simulator copy).
if [[ -z "${MCA_ECAT_ESI_PDO_SNAPSHOT:-}" ]]; then
  ESI_SOURCE="$(python3 - "$BASE_CONFIG" <<'PY'
import json
import pathlib
import sys

config_path = pathlib.Path(sys.argv[1]).resolve(strict=True)
config = json.loads(config_path.read_text(encoding="utf-8"))
esi = config.get("esi")
if not isinstance(esi, str) or not esi:
    raise SystemExit("KickCAT base JSON must contain a non-empty 'esi' path")
esi_path = pathlib.Path(esi)
if not esi_path.is_absolute():
    esi_path = config_path.parent / esi_path
print(esi_path.resolve(strict=True))
PY
  )" || fail "could not resolve ESI referenced by KickCAT JSON"
  ESI_RESOLVER="$SCRIPT_DIR/../ethercat_esi/esi_to_profile.py"
  [[ -f "$ESI_RESOLVER" ]] || fail "ESI resolver not found: $ESI_RESOLVER"
  mkdir -p "$ARTIFACT_DIR"
  export MCA_ECAT_ESI_PDO_SNAPSHOT="$ARTIFACT_DIR/esi-pdo-snapshot.json"
  python3 "$ESI_RESOLVER" "$ESI_SOURCE" \
    --product-code 0x03b11002 \
    --rx-pdo 0x1600 --tx-pdo 0x1a00 \
    --treat-slot-dependent-indices-as-literal \
    --emit-pdo-snapshot \
    -o "$MCA_ECAT_ESI_PDO_SNAPSHOT" ||
    fail "could not generate ESI PDO snapshot from the original ESI"
else
  [[ -f "$MCA_ECAT_ESI_PDO_SNAPSHOT" ]] ||
    fail "MCA_ECAT_ESI_PDO_SNAPSHOT does not exist: $MCA_ECAT_ESI_PDO_SNAPSHOT"
fi

export KICKCAT_SOURCE_DIR
export KICKCAT_SIMULATOR
export KICKCAT_SLAVE_CONFIGS="$BASE_CONFIG $BASE_CONFIG"
export MCA_ECAT_ARTIFACT_DIR="$ARTIFACT_DIR"
export MCA_ECAT_EXPECTED_SLAVES=2
export MCA_ECAT_CYCLIC_SIL=1
export MCA_ECAT_COMM_FAULT_SIL=1
export MCA_ECAT_FAULT_SIL=0
export MCA_ECAT_MULTI_FAULT_SIL=0
export MCA_ECAT_COE_READBACK="${MCA_ECAT_COE_READBACK:-1}"
export MCA_ECAT_DISABLE_EOE="${MCA_ECAT_DISABLE_EOE:-1}"
export MCA_ECAT_CYCLE_TIME_US="${MCA_ECAT_CYCLE_TIME_US:-1000}"
export MCA_ECAT_CYCLIC_TIMEOUT_SEC="${MCA_ECAT_CYCLIC_TIMEOUT_SEC:-60}"

exec bash "$RUNNER"
