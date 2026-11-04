#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
readonly APPLY_SCRIPT="$REPO_ROOT/scripts/ethercat_sil/apply_kickcat_ds402_fault_patch.py"
readonly PINNED_REVISION="cb2e1a192ac11ffaa58f08945053b8129abf94cb"
readonly KICKCAT_SOURCE_DIR="${KICKCAT_SOURCE_DIR:-$REPO_ROOT/scripts/ethercat_sil/KickCAT}"

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

[[ -d "$KICKCAT_SOURCE_DIR/.git" ]] || fail "KickCAT checkout not found: $KICKCAT_SOURCE_DIR"
[[ -f "$APPLY_SCRIPT" ]] || fail "fault-injection patch helper not found: $APPLY_SCRIPT"
actual_revision="$(git -C "$KICKCAT_SOURCE_DIR" rev-parse HEAD)"
[[ "$actual_revision" == "$PINNED_REVISION" ]] || fail \
  "KickCAT revision mismatch: expected $PINNED_REVISION, got $actual_revision"

header="$KICKCAT_SOURCE_DIR/lib/simulation/include/kickcat/simulation/devices/Ds402Motor.h"
device_app="$KICKCAT_SOURCE_DIR/lib/simulation/src/DeviceApp.cc"
[[ -f "$header" && -f "$device_app" ]] || fail "unexpected KickCAT source layout"

python3 "$APPLY_SCRIPT" "$KICKCAT_SOURCE_DIR" || fail "KickCAT DS402 fault fixture application failed"
git -C "$KICKCAT_SOURCE_DIR" diff --check || fail "KickCAT fixture contains whitespace errors"
grep -q 'fault_after_cycles' "$header" || fail "fault injection parameter missing after patch"
grep -q 'fault_reaction_cycles' "$device_app" || fail "JSON configuration binding missing after patch"
printf 'PASS: KickCAT DS402 fault-injection fixture ready at %s\n' "$actual_revision"
printf 'Build network_simulator in the KickCAT checkout before running MCA fault SIL.\n'
