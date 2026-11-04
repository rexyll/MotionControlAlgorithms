#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly RUNNER="$SCRIPT_DIR/run_kickcat_igh_veth_discovery.sh"
readonly BASE_CONFIG="${KICKCAT_SLAVE_CONFIG:-}"
readonly MATRIX="${MCA_ECAT_FAULT_REACTION_MATRIX:-1 2 3 5}"
readonly ARTIFACT_ROOT="${MCA_ECAT_FAULT_MATRIX_ARTIFACT_ROOT:-/tmp/mca-ethercat-fault-matrix-$(date +%Y%m%d-%H%M%S)}"
readonly WORK_DIR="$(mktemp -d)"

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

cleanup() {
  rm -rf "$WORK_DIR"
}
trap cleanup EXIT

[[ $EUID -eq 0 ]] || fail "run as root (sudo -E) because each case creates a veth pair and loads IgH modules"
[[ -x "$RUNNER" ]] || fail "single-run SIL harness not found: $RUNNER"
[[ -n "${KICKCAT_SIMULATOR:-}" && -x "$KICKCAT_SIMULATOR" ]] || fail "set KICKCAT_SIMULATOR to the patched network_simulator"
[[ -n "$BASE_CONFIG" && -f "$BASE_CONFIG" ]] || fail "set KICKCAT_SLAVE_CONFIG to the base fault fixture JSON"
command -v python3 >/dev/null || fail "python3 is required to derive duration fixtures"

[[ -n "${MATRIX//[[:space:]]/}" ]] || fail "MCA_ECAT_FAULT_REACTION_MATRIX must contain at least one duration"
read -r -a DURATION_VALUES <<< "$MATRIX"
(("${#DURATION_VALUES[@]}" > 0)) || fail "no fault-reaction durations were parsed"

mkdir -p "$ARTIFACT_ROOT"
printf 'Fault Reaction Active duration matrix: %s\n' "$MATRIX"
printf 'Artifacts root: %s\n' "$ARTIFACT_ROOT"

for requested_cycles in "${DURATION_VALUES[@]}"; do
  [[ "$requested_cycles" =~ ^[0-9]+$ ]] || fail "invalid duration value: $requested_cycles"
  cycles=$((10#$requested_cycles))
  (( cycles >= 1 && cycles <= 1000 )) || fail "duration must be in [1, 1000] cycles: $requested_cycles"

  fixture="$WORK_DIR/ecat402-drive-fault-${cycles}.json"
  python3 - "$BASE_CONFIG" "$fixture" "$cycles" <<'PY'
import json
import pathlib
import sys

source = pathlib.Path(sys.argv[1])
target = pathlib.Path(sys.argv[2])
duration = int(sys.argv[3])
config = json.loads(source.read_text(encoding="utf-8"))
if not isinstance(config, dict):
    raise SystemExit("fault fixture must be a JSON object")
if not bool(config.get("ds402_motor", False)):
    raise SystemExit("fault fixture must set ds402_motor=true")
if int(config.get("fault_after_cycles", 0)) <= 0:
    raise SystemExit("fault fixture must set a positive fault_after_cycles")
config["fault_reaction_cycles"] = duration
target.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
PY

  export KICKCAT_SLAVE_CONFIG="$fixture"
  export MCA_ECAT_ARTIFACT_DIR="$ARTIFACT_ROOT/reaction-${cycles}"
  export MCA_ECAT_COE_READBACK="${MCA_ECAT_COE_READBACK:-1}"
  export MCA_ECAT_DISABLE_EOE="${MCA_ECAT_DISABLE_EOE:-1}"
  export MCA_ECAT_CYCLIC_SIL=1
  export MCA_ECAT_FAULT_SIL=1
  export MCA_ECAT_CYCLIC_TIMEOUT_SEC="${MCA_ECAT_CYCLIC_TIMEOUT_SEC:-60}"

  mkdir -p "$MCA_ECAT_ARTIFACT_DIR"
  printf '\n=== Running fault-reaction duration: %s cycle(s) ===\n' "$cycles"
  bash "$RUNNER"
  printf 'PASS: Fault Reaction Active duration %s cycle(s)\n' "$cycles"
done

printf '\nPASS: all Fault Reaction Active duration matrix cases: %s\n' "$MATRIX"
printf 'Artifacts: %s\n' "$ARTIFACT_ROOT"
