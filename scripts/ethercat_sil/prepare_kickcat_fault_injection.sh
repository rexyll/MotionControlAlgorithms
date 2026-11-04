#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
readonly PATCH_FILE="$REPO_ROOT/patches/kickcat-ds402-fault-injection.patch"
readonly PINNED_REVISION="cb2e1a192ac11ffaa58f08945053b8129abf94cb"
readonly KICKCAT_SOURCE_DIR="${KICKCAT_SOURCE_DIR:-$REPO_ROOT/scripts/ethercat_sil/KickCAT}"

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

[[ -d "$KICKCAT_SOURCE_DIR/.git" ]] || fail "KickCAT checkout not found: $KICKCAT_SOURCE_DIR"
[[ -f "$PATCH_FILE" ]] || fail "fault-injection patch not found: $PATCH_FILE"
actual_revision="$(git -C "$KICKCAT_SOURCE_DIR" rev-parse HEAD)"
[[ "$actual_revision" == "$PINNED_REVISION" ]] || fail \
  "KickCAT revision mismatch: expected $PINNED_REVISION, got $actual_revision"

header="$KICKCAT_SOURCE_DIR/lib/simulation/include/kickcat/simulation/devices/Ds402Motor.h"
device_app="$KICKCAT_SOURCE_DIR/lib/simulation/src/DeviceApp.cc"
[[ -f "$header" && -f "$device_app" ]] || fail "unexpected KickCAT source layout"

if grep -q 'fault_after_cycles' "$header" && grep -q 'fault_after_cycles' "$device_app"; then
  printf 'KickCAT DS402 fault-injection patch already applied at %s\n' "$actual_revision"
  exit 0
fi

git -C "$KICKCAT_SOURCE_DIR" apply --check "$PATCH_FILE" || fail "patch does not apply cleanly; checkout may be modified"
git -C "$KICKCAT_SOURCE_DIR" apply "$PATCH_FILE"
git -C "$KICKCAT_SOURCE_DIR" diff --check
grep -q 'fault_after_cycles' "$header" || fail "fault injection parameter missing after patch"
grep -q 'fault_reaction_cycles' "$device_app" || fail "JSON configuration binding missing after patch"
printf 'PASS: KickCAT DS402 fault-injection patch applied to %s\n' "$actual_revision"
printf 'Build network_simulator in the KickCAT checkout before running MCA fault SIL.\n'
