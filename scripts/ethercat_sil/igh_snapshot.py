#!/usr/bin/env python3
"""Extract and validate the ordered IgH PDO discovery record from SIL output."""
from __future__ import annotations

import json
import sys
from pathlib import Path


class IghSnapshotError(ValueError):
    pass


PREFIX = "MCA_IGH_PDO_SNAPSHOT="


def parse_igh_log(text: str) -> dict:
    records = [line[len(PREFIX):] for line in text.splitlines() if line.startswith(PREFIX)]
    if len(records) != 1:
        raise IghSnapshotError(f"expected exactly one IgH snapshot record, found {len(records)}")
    try:
        raw = json.loads(records[0])
    except json.JSONDecodeError as exc:
        raise IghSnapshotError(f"invalid IgH snapshot JSON: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1 or raw.get("source_kind") != "igh_discovery":
        raise IghSnapshotError("unsupported IgH snapshot schema/provenance")
    entries = raw.get("entries")
    if not isinstance(entries, list) or not entries:
        raise IghSnapshotError("IgH snapshot has no mapping entries")
    slaves: dict[int, dict] = {}
    seen_groups: dict[int, set[tuple[int, int]]] = {}
    sm_directions: dict[tuple[int, int], str] = {}
    previous_key = None
    previous_entry_position = 0
    for position, item in enumerate(entries, 1):
        if not isinstance(item, dict):
            raise IghSnapshotError(f"entry {position}: record must be an object")
        required = ("ring_position", "vendor_id", "product_code", "sync_manager",
                    "direction", "pdo_index", "entry_position", "raw_mapping_entry")
        if any(key not in item for key in required):
            raise IghSnapshotError(f"entry {position}: missing required field")
        ring, vendor, product = item["ring_position"], item["vendor_id"], item["product_code"]
        sm, direction, pdo = item["sync_manager"], item["direction"], item["pdo_index"]
        entry_pos, raw_word = item["entry_position"], item["raw_mapping_entry"]
        if any(not isinstance(value, int) for value in (ring, vendor, product, sm, pdo, entry_pos, raw_word)):
            raise IghSnapshotError(f"entry {position}: numeric field has wrong type")
        if ring < 0 or not 0 <= vendor <= 0xFFFFFFFF or not 0 <= product <= 0xFFFFFFFF:
            raise IghSnapshotError(f"entry {position}: slave identity out of range")
        if not 0 <= sm <= 255 or not 0x1000 <= pdo <= 0xFFFF:
            raise IghSnapshotError(f"entry {position}: SM/PDO index out of range")
        if direction not in ("input", "output") or not 1 <= entry_pos <= 64 or not 0 <= raw_word <= 0xFFFFFFFF:
            raise IghSnapshotError(f"entry {position}: direction/entry position/mapping word invalid")
        width = raw_word & 0xFF
        od_index = (raw_word >> 16) & 0xFFFF
        subindex = (raw_word >> 8) & 0xFF
        if width == 0 or (od_index == 0 and subindex != 0):
            raise IghSnapshotError(f"entry {position}: malformed mapping entry/padding")
        if ring not in slaves:
            slaves[ring] = {
                "ring_position": ring, "vendor_id": vendor, "product_code": product,
                "mappings": [],
            }
            seen_groups[ring] = set()
        elif slaves[ring]["vendor_id"] != vendor or slaves[ring]["product_code"] != product:
            raise IghSnapshotError(f"ring {ring}: inconsistent slave identity")
        sm_key = (ring, sm)
        prior_direction = sm_directions.get(sm_key)
        if prior_direction is not None and prior_direction != direction:
            raise IghSnapshotError(f"ring {ring}: SyncManager direction changed")
        sm_directions[sm_key] = direction
        key = (ring, sm, pdo)
        if previous_key is None or key != previous_key:
            if previous_key is not None:
                seen_groups[previous_key[0]].add((previous_key[1], previous_key[2]))
            if (sm, pdo) in seen_groups[ring]:
                raise IghSnapshotError(f"ring {ring}: SM/PDO group is not contiguous")
            if previous_key is not None and ring < previous_key[0]:
                raise IghSnapshotError("slave ring order regressed")
            if previous_key is not None and ring == previous_key[0] and sm < previous_key[1]:
                raise IghSnapshotError(f"ring {ring}: SyncManager order regressed")
            previous_entry_position = 0
        previous_entry_position += 1
        if entry_pos != previous_entry_position:
            raise IghSnapshotError(f"ring {ring}: entry position gap in PDO 0x{pdo:04X}")
        slaves[ring]["mappings"].append(item)
        previous_key = key
    if sorted(slaves) != list(range(len(slaves))):
        raise IghSnapshotError("slave ring positions must be contiguous from zero")
    return {
        "schema_version": 1, "source_kind": "igh_discovery",
        "slaves": [slaves[ring] for ring in sorted(slaves)],
        "runtime_discovery_performed": True,
    }


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: igh_snapshot.py CYCLIC_LOG OUTPUT.json", file=sys.stderr)
        return 2
    try:
        result = parse_igh_log(Path(sys.argv[1]).read_text(encoding="utf-8"))
        Path(sys.argv[2]).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    except (OSError, IghSnapshotError) as exc:
        print(f"IgH PDO snapshot error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
