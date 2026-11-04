#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly RUNNER="$SCRIPT_DIR/run_kickcat_igh_veth_discovery.sh"
BASE_CONFIG="${KICKCAT_SLAVE_CONFIG:-}"
readonly MATRIX="${MCA_ECAT_FAULT_REACTION_MATRIX:-1 2 3 5}"
readonly ARTIFACT_ROOT="${MCA_ECAT_FAULT_MATRIX_ARTIFACT_ROOT:-/tmp/mca-ethercat-fault-matrix-$(date +%Y%m%d-%H%M%S)}"
FIXTURE_FILES=()

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

cleanup() {
  local fixture
  for fixture in "${FIXTURE_FILES[@]}"; do
    rm -f -- "$fixture"
  done
}
trap cleanup EXIT

[[ $EUID -eq 0 ]] || fail "run as root (sudo -E) because each case creates a veth pair and loads IgH modules"
[[ -x "$RUNNER" ]] || fail "single-run SIL harness not found: $RUNNER"
[[ -n "${KICKCAT_SIMULATOR:-}" && -x "$KICKCAT_SIMULATOR" ]] || fail "set KICKCAT_SIMULATOR to the patched network_simulator"
[[ -n "$BASE_CONFIG" && -f "$BASE_CONFIG" ]] || fail "set KICKCAT_SLAVE_CONFIG to the base fault fixture JSON"
command -v python3 >/dev/null || fail "python3 is required to derive duration fixtures"
command -v realpath >/dev/null || fail "realpath is required to normalize the base fixture path"

# KickCAT resolves ESI/EEPROM/CoE/FoE resources relative to the directory of
# the JSON path passed on the command line. Derived fixtures must therefore
# live beside the base JSON, not in /tmp or a nested temporary directory.
BASE_CONFIG="$(realpath -- "$BASE_CONFIG")"
readonly BASE_CONFIG
CONFIG_DIR="$(dirname -- "$BASE_CONFIG")"

[[ -n "${MATRIX//[[:space:]]/}" ]] || fail "MCA_ECAT_FAULT_REACTION_MATRIX must contain at least one duration"
read -r -a DURATION_VALUES <<< "$MATRIX"
((${#DURATION_VALUES[@]} > 0)) || fail "no fault-reaction durations were parsed"

mkdir -p "$ARTIFACT_ROOT"
printf 'Fault Reaction Active duration matrix: %s\n' "$MATRIX"
printf 'Artifacts root: %s\n' "$ARTIFACT_ROOT"

for requested_cycles in "${DURATION_VALUES[@]}"; do
  [[ "$requested_cycles" =~ ^[0-9]+$ ]] || fail "invalid duration value: $requested_cycles"
  cycles=$((10#$requested_cycles))
  (( cycles >= 1 && cycles <= 1000 )) || fail "duration must be in [1, 1000] cycles: $requested_cycles"

  fixture="$(mktemp --tmpdir="$CONFIG_DIR" ".ecat402-drive-fault-${cycles}.XXXXXX")"
  FIXTURE_FILES+=("$fixture")
  python3 - "$BASE_CONFIG" "$fixture" "$cycles" <<'PY'
import json
import pathlib
import sys

source = pathlib.Path(sys.argv[1]).resolve(strict=True)
target = pathlib.Path(sys.argv[2]).resolve()
duration = int(sys.argv[3])
if target.parent != source.parent:
    raise SystemExit(
        "derived fixture must be beside base config so KickCAT resolves "
        "relative resources from the same directory"
    )
config = json.loads(source.read_text(encoding="utf-8"))
if not isinstance(config, dict):
    raise SystemExit("fault fixture must be a JSON object")
if not bool(config.get("ds402_motor", False)):
    raise SystemExit("fault fixture must set ds402_motor=true")
if int(config.get("fault_after_cycles", 0)) <= 0:
    raise SystemExit("fault fixture must set a positive fault_after_cycles")

# Validate the resource files/directories KickCAT resolves against config_dir.
# Preserve their original relative strings: this file has the same parent
# directory as the base JSON, so all relative paths retain their meaning.
for key in ("esi", "eeprom", "coe_xml"):
    if key in config:
        resource = pathlib.Path(config[key])
        resolved = resource if resource.is_absolute() else source.parent / resource
        if not resolved.is_file():
            raise SystemExit(f"configured {key} file not found: {resolved}")
if "foe_dir" in config:
    resource = pathlib.Path(config["foe_dir"])
    resolved = resource if resource.is_absolute() else source.parent / resource
    if not resolved.is_dir():
        raise SystemExit(f"configured foe_dir not found: {resolved}")

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
