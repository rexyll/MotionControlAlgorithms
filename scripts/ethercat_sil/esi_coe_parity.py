#!/usr/bin/env python3
"""Fail-closed comparison of explicit ESI PDO expectations against CoE SDO readback."""
from __future__ import annotations

import json
import sys
from pathlib import Path


class ParityError(ValueError):
    pass


def _load(path: str) -> dict:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ParityError(f"cannot load JSON snapshot {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ParityError(f"snapshot root must be an object: {path}")
    return value


def compare_snapshots(esi: dict, coe: dict) -> dict:
    if esi.get("schema_version") != 1 or esi.get("source_kind") != "esi_expected":
        raise ParityError("input ESI snapshot must be schema_version=1/source_kind=esi_expected")
    if esi.get("runtime_readback_performed") is not False:
        raise ParityError("ESI expectation must not claim runtime readback")
    if coe.get("schema_version") != 1 or coe.get("source_kind") != "coe_sdo_readback":
        raise ParityError("input CoE snapshot must be schema_version=1/source_kind=coe_sdo_readback")
    if coe.get("runtime_readback_performed") is not True:
        raise ParityError("CoE snapshot must explicitly identify runtime readback")
    identity = esi.get("identity")
    if not isinstance(identity, dict):
        raise ParityError("ESI snapshot has no identity object")
    expected_assignments = esi.get("assignments")
    expected_mappings = esi.get("mappings")
    slaves = coe.get("slaves")
    if not isinstance(expected_assignments, list) or not expected_assignments:
        raise ParityError("ESI snapshot has no assignment records")
    if not isinstance(expected_mappings, list) or not expected_mappings:
        raise ParityError("ESI snapshot has no mapping records")
    if not isinstance(slaves, list) or not slaves:
        raise ParityError("CoE snapshot has no slave records")
    if any(not isinstance(item, dict) for item in expected_assignments + expected_mappings):
        raise ParityError("ESI assignment/mapping records must be objects")
    if any(not isinstance(item, dict) for item in slaves):
        raise ParityError("CoE slave records must be objects")
    ring_positions = [item.get("ring_position") for item in slaves]
    if any(not isinstance(ring, int) or ring < 0 for ring in ring_positions):
        raise ParityError("CoE slave has invalid ring_position")
    if sorted(ring_positions) != list(range(len(slaves))) or len(set(ring_positions)) != len(ring_positions):
        raise ParityError("CoE slave ring positions must be unique and contiguous from zero")

    def assignment_tuple(item: dict) -> tuple:
        return (item.get("assignment_object"), item.get("assignment_position"), item.get("pdo_index"))

    def mapping_tuple(item: dict) -> tuple:
        return (
            item.get("assignment_object"), item.get("assignment_position"),
            item.get("pdo_index"), item.get("entry_position"), item.get("raw_mapping_entry"),
        )

    expected_assignment_order = [assignment_tuple(item) for item in expected_assignments]
    expected_mapping_order = [mapping_tuple(item) for item in expected_mappings]
    if any(None in record for record in expected_assignment_order):
        raise ParityError("ESI assignment record is incomplete")
    if any(None in record for record in expected_mapping_order):
        raise ParityError("ESI mapping record is incomplete")

    results = []
    for slave in slaves:
        ring = slave["ring_position"]
        observed_identity = {
            "vendor_id": slave.get("vendor_id"),
            "product_code": slave.get("product_code"),
        }
        if observed_identity != identity:
            results.append({
                "ring_position": ring, "match": False,
                "mismatches": [{"layer": "identity", "expected": identity, "observed": observed_identity}],
            })
            continue
        assignments = slave.get("assignments")
        mappings = slave.get("mappings")
        if not isinstance(assignments, list) or not isinstance(mappings, list):
            raise ParityError(f"ring {ring}: CoE assignments/mappings must be arrays")
        if any(not isinstance(item, dict) for item in assignments + mappings):
            raise ParityError(f"ring {ring}: CoE assignment/mapping records must be objects")
        observed_assignment_order = [assignment_tuple(item) for item in assignments]
        observed_mapping_order = [mapping_tuple(item) for item in mappings]
        mismatches = []
        if observed_assignment_order != expected_assignment_order:
            mismatches.append({
                "layer": "assignment", "expected": expected_assignment_order,
                "observed": observed_assignment_order,
            })
        if observed_mapping_order != expected_mapping_order:
            mismatches.append({
                "layer": "mapping", "expected": expected_mapping_order,
                "observed": observed_mapping_order,
            })
        results.append({"ring_position": ring, "match": not mismatches, "mismatches": mismatches})
    return {
        "schema_version": 1, "source_kind": "esi_coe_parity",
        "match": all(item["match"] for item in results),
        "slaves": results,
        "limitations": [
            "CoE SDO readback does not report SyncManager association or PDO direction.",
            "IgH runtime SM/PDO/Entry parity is not part of this comparison.",
            "ESI expectations are valid only for the explicitly selected non-slot PDO configuration.",
        ],
    }


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: esi_coe_parity.py ESI_EXPECTED.json COE_READBACK.json REPORT.json", file=sys.stderr)
        return 2
    try:
        report = compare_snapshots(_load(sys.argv[1]), _load(sys.argv[2]))
        Path(sys.argv[3]).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    except ParityError as exc:
        print(f"ESI/CoE parity error: {exc}", file=sys.stderr)
        return 2
    return 0 if report["match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
