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

for reaction in "${reaction_cycles[@]}"; do
for target in 0 1; do
  configs=()
  for slot in 0 1; do
    fixture="$(mktemp --tmpdir="$CONFIG_DIR" ".ecat402-drive-multifault-${reaction}-${target}-${slot}.XXXXXX")"
    FIXTURES+=("$fixture")
    python3 - "$BASE_CONFIG" "$fixture" "$slot" "$target" "$FAULT_AFTER_CYCLES" "$reaction" <<'PY'
import json
import pathlib
import sys
source = pathlib.Path(sys.argv[1]).resolve(strict=True)
target_path = pathlib.Path(sys.argv[2]).resolve()
slot, selected = int(sys.argv[3]), int(sys.argv[4])
after, reaction = int(sys.argv[5]), int(sys.argv[6])
if target_path.parent != source.parent:
    raise SystemExit("fixture must remain beside base JSON to preserve relative ESI resources")
config = json.loads(source.read_text(encoding="utf-8"))
if not isinstance(config, dict) or not bool(config.get("ds402_motor", False)):
    raise SystemExit("base fixture must be a DS402 motor JSON object")
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
