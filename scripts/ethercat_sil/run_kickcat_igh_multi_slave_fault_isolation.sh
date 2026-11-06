#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly RUNNER="$SCRIPT_DIR/run_kickcat_igh_veth_discovery.sh"
BASE_CONFIG="${KICKCAT_SLAVE_CONFIG:-}"
readonly ARTIFACT_ROOT="${MCA_ECAT_MULTI_FAULT_ARTIFACT_ROOT:-/tmp/mca-ethercat-multi-fault-$(date +%Y%m%d-%H%M%S)}"
readonly FAULT_AFTER_CYCLES="${MCA_ECAT_MULTI_FAULT_AFTER_CYCLES:-40}"
readonly REACTION_MATRIX="${MCA_ECAT_MULTI_FAULT_REACTION_MATRIX:-1,2,3,5}"
FIXTURES=()

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
cleanup() { local f; for f in "${FIXTURES[@]}"; do rm -f -- "$f"; done; }
trap cleanup EXIT

[[ $EUID -eq 0 ]] || fail "run as root (sudo -E)"
[[ -x "$RUNNER" ]] || fail "veth discovery runner not found"
[[ -n "${KICKCAT_SIMULATOR:-}" && -x "$KICKCAT_SIMULATOR" ]] || fail "set KICKCAT_SIMULATOR to the patched simulator"
[[ -n "$BASE_CONFIG" && -f "$BASE_CONFIG" ]] || fail "set KICKCAT_SLAVE_CONFIG to the known-good DS402 JSON"
[[ "$FAULT_AFTER_CYCLES" =~ ^[0-9]+$ ]] && (( FAULT_AFTER_CYCLES >= 20 && FAULT_AFTER_CYCLES <= 100000 )) || fail "fault-after cycles must be in [20, 100000]"
IFS=',' read -r -a reaction_cycles <<< "$REACTION_MATRIX"
(( ${#reaction_cycles[@]} > 0 )) || fail "reaction matrix must not be empty"
for reaction in "${reaction_cycles[@]}"; do
  [[ "$reaction" =~ ^[0-9]+$ ]] && (( reaction >= 1 && reaction <= 1000 )) || fail "each reaction duration must be in [1, 1000]"
done
command -v python3 >/dev/null || fail "python3 is required"
command -v realpath >/dev/null || fail "realpath is required"

BASE_CONFIG="$(realpath -- "$BASE_CONFIG")"
CONFIG_DIR="$(dirname -- "$BASE_CONFIG")"
mkdir -p "$ARTIFACT_ROOT"

# Resolve the ESI referenced by the KickCAT JSON config. Keep the original
# ESI as the independent PDO expectation, but use a SIL-only clone for the
# simulator: it disables EoE (not implemented by this simulator) and CoE
# SDO Information (which the simulator does not answer), while retaining CoE
# SDO/PDO assignment/configuration and byte-for-byte PDO/SM declarations.
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
ESI_PREPARER="$SCRIPT_DIR/prepare_kickcat_coe_sil_esi.py"
[[ -f "$ESI_PREPARER" ]] || fail "CoE SIL ESI preparer not found: $ESI_PREPARER"
ESI_SIL_FIXTURE="$(mktemp --tmpdir="$CONFIG_DIR" ".ecat402-drive-coe-sil.XXXXXX.xml")"
FIXTURES+=("$ESI_SIL_FIXTURE")
python3 "$ESI_PREPARER" "$ESI_SOURCE" "$ESI_SIL_FIXTURE" ||
  fail "could not prepare CoE-only KickCAT SIL ESI"

# Generate/reuse the fail-closed three-way parity expectation from the
# unmodified source ESI, never from the protocol-trimmed SIL clone.
if [[ -z "${MCA_ECAT_ESI_PDO_SNAPSHOT:-}" ]]; then
  ESI_RESOLVER="$SCRIPT_DIR/../ethercat_esi/esi_to_profile.py"
  [[ -f "$ESI_RESOLVER" ]] || fail "ESI resolver not found: $ESI_RESOLVER"
  export MCA_ECAT_ESI_PDO_SNAPSHOT="$ARTIFACT_ROOT/esi-pdo-snapshot.json"
  python3 "$ESI_RESOLVER" "$ESI_SOURCE" \
    --product-code 0x03b11002 \
    --rx-pdo 0x1600 --tx-pdo 0x1a00 \
    --treat-slot-dependent-indices-as-literal \
    --emit-pdo-snapshot \
    -o "$MCA_ECAT_ESI_PDO_SNAPSHOT"
else
  [[ -f "$MCA_ECAT_ESI_PDO_SNAPSHOT" ]] || fail "MCA_ECAT_ESI_PDO_SNAPSHOT does not exist: $MCA_ECAT_ESI_PDO_SNAPSHOT"
  export MCA_ECAT_ESI_PDO_SNAPSHOT
fi

for reaction in "${reaction_cycles[@]}"; do
for target in 0 1; do
  configs=()
  for slot in 0 1; do
    fixture="$(mktemp --tmpdir="$CONFIG_DIR" ".ecat402-drive-multifault-${reaction}-${target}-${slot}.XXXXXX")"
    FIXTURES+=("$fixture")
    python3 - "$BASE_CONFIG" "$fixture" "$slot" "$target" "$FAULT_AFTER_CYCLES" "$reaction" "$ESI_SIL_FIXTURE" <<'PY'
import json
import pathlib
import sys
source = pathlib.Path(sys.argv[1]).resolve(strict=True)
target_path = pathlib.Path(sys.argv[2]).resolve()
slot, selected = int(sys.argv[3]), int(sys.argv[4])
after, reaction = int(sys.argv[5]), int(sys.argv[6])
esi_fixture = pathlib.Path(sys.argv[7]).resolve(strict=True)
if target_path.parent != source.parent or esi_fixture.parent != source.parent:
    raise SystemExit("fixtures must remain beside base JSON to preserve relative ESI resources")
config = json.loads(source.read_text(encoding="utf-8"))
if not isinstance(config, dict) or not bool(config.get("ds402_motor", False)):
    raise SystemExit("base fixture must be a DS402 motor JSON object")
config["esi"] = esi_fixture.name
config["fault_after_cycles"] = after if slot == selected else 0
config["fault_reaction_cycles"] = reaction
target_path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
PY
    configs+=("$fixture")
  done

  printf '\n=== Independent fault isolation: FRA=%s cycles, inject ring position %s ===\n' "$reaction" "$target"
  export KICKCAT_SLAVE_CONFIGS="${configs[0]} ${configs[1]}"
  export MCA_ECAT_EXPECTED_SLAVES=2
  export MCA_ECAT_MULTI_FAULT_SIL=1
  export MCA_ECAT_MULTI_FAULT_RING_POSITION="$target"
  export MCA_ECAT_MULTI_FAULT_REACTION_CYCLES="$reaction"
  export MCA_ECAT_CYCLIC_SIL=1
  export MCA_ECAT_COE_READBACK="${MCA_ECAT_COE_READBACK:-1}"
  export MCA_ECAT_DISABLE_EOE="${MCA_ECAT_DISABLE_EOE:-1}"
  export MCA_ECAT_CYCLIC_TIMEOUT_SEC="${MCA_ECAT_CYCLIC_TIMEOUT_SEC:-60}"
  export MCA_ECAT_ARTIFACT_DIR="$ARTIFACT_ROOT/fra-$reaction/ring-$target"
  bash "$RUNNER"
  printf 'PASS: independent fault isolation FRA=%s target ring=%s\n' "$reaction" "$target"
done
done

printf '\nPASS: dual-slave fault-isolation matrix completed (FRA durations: %s; targets: ring 0 and 1).\n' "$REACTION_MATRIX"
printf 'Artifacts: %s\n' "$ARTIFACT_ROOT"
