#!/usr/bin/env python3
"""Convert ordered CoE SDO rows into a validated, versioned PDO snapshot."""
from __future__ import annotations

import json
import sys
from pathlib import Path


class SnapshotError(ValueError):
    pass


def parse_rows(text: str, expected_slave_count: int | None = None) -> dict:
    if expected_slave_count is not None and expected_slave_count < 1:
        raise SnapshotError("expected_slave_count must be positive")
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
                "assignments": [], "mappings": [], "pdo_entry_counts": {}, "assignment_counts": {},
            }
        elif kind == "C" and len(nums) == 3:
            ring, assignment_object, pdo_count = nums
            if ring not in slaves or assignment_object not in (0x1C12, 0x1C13):
                raise SnapshotError(f"line {line_number}: assignment count has no slave or invalid object")
            if not 1 <= pdo_count <= 16:
                raise SnapshotError(f"line {line_number}: assignment PDO count out of range")
            counts = slaves[ring].setdefault("assignment_counts", {})
            if assignment_object in counts:
                raise SnapshotError(f"line {line_number}: duplicate assignment count record")
            counts[assignment_object] = pdo_count
        elif kind == "A" and len(nums) == 4:
            ring, assignment_object, position, pdo_index = nums
            if ring not in slaves or assignment_object not in (0x1C12, 0x1C13):
                raise SnapshotError(f"line {line_number}: assignment has no slave or invalid object")
            if not 1 <= position <= 16 or not 0x1000 <= pdo_index <= 0xFFFF:
                raise SnapshotError(f"line {line_number}: assignment field out of range")
            lower, upper = (0x1600, 0x17FF) if assignment_object == 0x1C12 else (0x1A00, 0x1BFF)
            if not lower <= pdo_index <= upper:
                raise SnapshotError(f"line {line_number}: PDO index outside assignment direction range")
            slaves[ring]["assignments"].append({
                "assignment_object": assignment_object,
                "assignment_position": position, "pdo_index": pdo_index,
            })
        elif kind == "P" and len(nums) == 5:
            ring, assignment_object, assignment_position, pdo_index, entry_count = nums
            if ring not in slaves or assignment_object not in (0x1C12, 0x1C13):
                raise SnapshotError(f"line {line_number}: PDO count has no slave or invalid assignment object")
            if not 1 <= assignment_position <= 16 or not 0x1000 <= pdo_index <= 0xFFFF or not 1 <= entry_count <= 64:
                raise SnapshotError(f"line {line_number}: PDO entry count record out of range")
            key = (assignment_object, assignment_position, pdo_index)
            counts = slaves[ring].setdefault("pdo_entry_counts", {})
            if key in counts:
                raise SnapshotError(f"line {line_number}: duplicate PDO entry count record")
            counts[key] = entry_count
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
    if expected_slave_count is not None and len(slaves) != expected_slave_count:
        raise SnapshotError(f"expected {expected_slave_count} slave identities, found {len(slaves)}")
    result_slaves = []
    if sorted(slaves) != list(range(len(slaves))):
        raise SnapshotError("slave ring positions must be contiguous from zero")
    for ring in sorted(slaves):
        slave = slaves[ring]
        assignments = slave["assignments"]
        if not assignments:
            raise SnapshotError(f"ring {ring}: no assignments")
        expected = {0x1C12: 1, 0x1C13: 1}
        assignment_counts = slave["assignment_counts"]
        if set(assignment_counts) != {0x1C12, 0x1C13}:
            raise SnapshotError(f"ring {ring}: missing or extra assignment count records")
        seen_pairs = set()
        seen_pdos = {0x1C12: set(), 0x1C13: set()}
        for item in assignments:
            obj, pos = item["assignment_object"], item["assignment_position"]
            if pos != expected[obj]:
                raise SnapshotError(f"ring {ring}: assignment order/gap at 0x{obj:04X}:{pos}")
            expected[obj] += 1
            pair = (obj, pos)
            if pair in seen_pairs:
                raise SnapshotError(f"ring {ring}: duplicate assignment position")
            seen_pairs.add(pair)
            if item["pdo_index"] in seen_pdos[obj]:
                raise SnapshotError(f"ring {ring}: duplicate PDO index in assignment 0x{obj:04X}")
            seen_pdos[obj].add(item["pdo_index"])
        for obj, count in assignment_counts.items():
            if expected[obj] - 1 != count:
                raise SnapshotError(f"ring {ring}: assignment cardinality mismatch for 0x{obj:04X}")
        assignment_keys = {
            (item["assignment_object"], item["assignment_position"], item["pdo_index"])
            for item in assignments
        }
        pdo_entry_counts = slave["pdo_entry_counts"]
        if set(pdo_entry_counts) != assignment_keys:
            raise SnapshotError(f"ring {ring}: missing or extra PDO entry count records")
        seen_entries: dict[tuple[int, int, int], int] = {}
        mapped_objects = set()
        closed_mapping_groups = set()
        previous_mapping_key = None
        for item in slave["mappings"]:
            key = (item["assignment_object"], item["assignment_position"], item["pdo_index"])
            if key not in assignment_keys:
                raise SnapshotError(f"ring {ring}: mapping references unknown assignment/PDO")
            if key not in pdo_entry_counts:
                raise SnapshotError(f"ring {ring}: mapping has no PDO entry count record")
            if previous_mapping_key is not None and key != previous_mapping_key:
                closed_mapping_groups.add(previous_mapping_key)
            if key in closed_mapping_groups:
                raise SnapshotError(f"ring {ring}: mapping PDO entries are not contiguous")
            previous_mapping_key = key
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
        for key, expected_count in pdo_entry_counts.items():
            if seen_entries.get(key, 0) != expected_count:
                raise SnapshotError(f"ring {ring}: PDO mapping cardinality mismatch for 0x{key[2]:04X}")
        slave.pop("pdo_entry_counts", None)
        slave.pop("assignment_counts", None)
        slave["assignments"] = assignments
        result_slaves.append(slave)
    return {
        "schema_version": 1, "source_kind": "coe_sdo_readback",
        "slaves": result_slaves, "runtime_readback_performed": True,
    }


def main() -> int:
    if len(sys.argv) not in (3, 4):
        print("usage: coe_snapshot.py INPUT.tsv OUTPUT.json [EXPECTED_SLAVES]", file=sys.stderr)
        return 2
    try:
        try:
            expected = int(sys.argv[3], 10) if len(sys.argv) == 4 else None
        except ValueError as exc:
            raise SnapshotError("EXPECTED_SLAVES must be a positive decimal integer") from exc
        snapshot = parse_rows(Path(sys.argv[1]).read_text(encoding="utf-8"), expected)
        Path(sys.argv[2]).write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
    except (OSError, SnapshotError) as exc:
        print(f"CoE snapshot error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
