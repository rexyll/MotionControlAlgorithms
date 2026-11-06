#!/usr/bin/env python3
"""Convert ordered CoE SDO rows into a validated, versioned PDO snapshot."""
from __future__ import annotations

import json
import sys
from pathlib import Path


class SnapshotError(ValueError):
    pass


def parse_rows(text: str) -> dict:
    slaves: dict[int, dict] = {}
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        if not raw_line.strip():
            continue
        fields = raw_line.split("\t")
        kind = fields[0]
        try:
            nums = [int(value, 0) for value in fields[1:]]
        except ValueError as exc:
            raise SnapshotError(f"line {line_number}: non-integer field") from exc
        if kind == "S" and len(nums) == 3:
            ring, vendor, product = nums
            if ring in slaves or ring < 0 or not 0 <= vendor <= 0xFFFFFFFF or not 0 <= product <= 0xFFFFFFFF:
                raise SnapshotError(f"line {line_number}: invalid or duplicate slave identity")
            slaves[ring] = {
                "ring_position": ring, "vendor_id": vendor, "product_code": product,
                "assignments": [], "mappings": [],
            }
        elif kind == "A" and len(nums) == 4:
            ring, assignment_object, position, pdo_index = nums
            if ring not in slaves or assignment_object not in (0x1C12, 0x1C13):
                raise SnapshotError(f"line {line_number}: assignment has no slave or invalid object")
            if not 1 <= position <= 16 or not 0x1000 <= pdo_index <= 0xFFFF:
                raise SnapshotError(f"line {line_number}: assignment field out of range")
            slaves[ring]["assignments"].append({
                "assignment_object": assignment_object,
                "assignment_position": position, "pdo_index": pdo_index,
            })
        elif kind == "M" and len(nums) == 6:
            ring, assignment_object, assignment_position, pdo_index, entry_position, raw_word = nums
            if ring not in slaves or assignment_object not in (0x1C12, 0x1C13):
                raise SnapshotError(f"line {line_number}: mapping has no slave or invalid assignment object")
            if not 1 <= assignment_position <= 16 or not 1 <= entry_position <= 64:
                raise SnapshotError(f"line {line_number}: mapping position out of range")
            if not 0 <= raw_word <= 0xFFFFFFFF:
                raise SnapshotError(f"line {line_number}: mapping word out of range")
            index, subindex, width = (raw_word >> 16) & 0xFFFF, (raw_word >> 8) & 0xFF, raw_word & 0xFF
            if width == 0 or (index == 0 and subindex != 0):
                raise SnapshotError(f"line {line_number}: malformed mapping entry/padding")
            slaves[ring]["mappings"].append({
                "assignment_object": assignment_object,
                "assignment_position": assignment_position, "pdo_index": pdo_index,
                "entry_position": entry_position, "raw_mapping_entry": raw_word,
            })
        else:
            raise SnapshotError(f"line {line_number}: malformed {kind!r} row")
    if not slaves:
        raise SnapshotError("snapshot contains no slave identity rows")
    result_slaves = []
    for ring in sorted(slaves):
        slave = slaves[ring]
        assignments = slave["assignments"]
        if not assignments:
            raise SnapshotError(f"ring {ring}: no assignments")
        expected = {0x1C12: 1, 0x1C13: 1}
        seen_pairs = set()
        for item in assignments:
            obj, pos = item["assignment_object"], item["assignment_position"]
            if pos != expected[obj]:
                raise SnapshotError(f"ring {ring}: assignment order/gap at 0x{obj:04X}:{pos}")
            expected[obj] += 1
            pair = (obj, pos)
            if pair in seen_pairs:
                raise SnapshotError(f"ring {ring}: duplicate assignment position")
            seen_pairs.add(pair)
        assignment_keys = {
            (item["assignment_object"], item["assignment_position"], item["pdo_index"])
            for item in assignments
        }
        seen_entries: dict[tuple[int, int, int], int] = {}
        mapped_objects = set()
        for item in slave["mappings"]:
            key = (item["assignment_object"], item["assignment_position"], item["pdo_index"])
            if key not in assignment_keys:
                raise SnapshotError(f"ring {ring}: mapping references unknown assignment/PDO")
            expected_entry = seen_entries.get(key, 0) + 1
            if item["entry_position"] != expected_entry:
                raise SnapshotError(f"ring {ring}: mapping entry order/gap in PDO 0x{item['pdo_index']:04X}")
            seen_entries[key] = expected_entry
            raw = item["raw_mapping_entry"]
            od_index, subindex = (raw >> 16) & 0xFFFF, (raw >> 8) & 0xFF
            if od_index:
                od_key = (od_index, subindex)
                if od_key in mapped_objects:
                    raise SnapshotError(f"ring {ring}: duplicate mapped object 0x{od_index:04X}:{subindex:02X}")
                mapped_objects.add(od_key)
        expected_order = [
            (item["assignment_object"], item["assignment_position"], item["pdo_index"])
            for item in assignments
        ]
        actual_order = list(dict.fromkeys(
            (item["assignment_object"], item["assignment_position"], item["pdo_index"])
            for item in slave["mappings"]
        ))
        if actual_order != expected_order:
            raise SnapshotError(f"ring {ring}: mapping PDO order/cardinality differs from assignments")
        slave["assignments"] = assignments
        result_slaves.append(slave)
    return {
        "schema_version": 1, "source_kind": "coe_sdo_readback",
        "slaves": result_slaves, "runtime_readback_performed": True,
    }


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: coe_snapshot.py INPUT.tsv OUTPUT.json", file=sys.stderr)
        return 2
    try:
        snapshot = parse_rows(Path(sys.argv[1]).read_text(encoding="utf-8"))
        Path(sys.argv[2]).write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
    except (OSError, SnapshotError) as exc:
        print(f"CoE snapshot error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
