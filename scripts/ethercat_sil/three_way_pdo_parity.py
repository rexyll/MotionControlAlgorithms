#!/usr/bin/env python3
"""Require and consolidate all three pairwise ESI/CoE/IgH PDO parity reports."""
from __future__ import annotations
import json
import sys
from pathlib import Path

class ThreeWayParityError(ValueError):
    pass

EXPECTED_KINDS = {
    "esi_coe": "esi_coe_parity",
    "esi_igh": "esi_igh_parity",
    "coe_igh": "coe_igh_parity",
}

def consolidate_reports(esi_coe, esi_igh, coe_igh, expected_slaves):
    if type(expected_slaves) is not int or expected_slaves < 1:
        raise ThreeWayParityError("expected_slaves must be a positive integer")
    reports = {"esi_coe": esi_coe, "esi_igh": esi_igh, "coe_igh": coe_igh}
    checks, ring_sets = [], []
    for name, kind in EXPECTED_KINDS.items():
        report = reports[name]
        if not isinstance(report, dict) or report.get("schema_version") != 1:
            raise ThreeWayParityError(f"{name} report has unsupported schema")
        if report.get("source_kind") != kind:
            raise ThreeWayParityError(f"{name} report source_kind must be {kind}")
        slaves = report.get("slaves")
        if not isinstance(slaves, list) or len(slaves) != expected_slaves:
            raise ThreeWayParityError(f"{name} report must contain exactly {expected_slaves} slave results")
        rings = []
        for item in slaves:
            if not isinstance(item, dict) or type(item.get("ring_position")) is not int:
                raise ThreeWayParityError(f"{name} report has malformed slave result")
            if type(item.get("match")) is not bool:
                raise ThreeWayParityError(f"{name} slave result has no boolean match")
            rings.append(item["ring_position"])
        if sorted(rings) != list(range(expected_slaves)) or len(set(rings)) != expected_slaves:
            raise ThreeWayParityError(f"{name} report ring positions are not unique/contiguous")
        ring_sets.append(tuple(sorted(rings)))
        checks.append({
            "comparison": name,
            "match": report.get("match") is True and all(item["match"] for item in slaves),
            "ring_positions": sorted(rings),
            "mismatch_count": sum(not item["match"] for item in slaves),
        })
    if len(set(ring_sets)) != 1:
        raise ThreeWayParityError("pairwise reports cover different slave ring positions")
    return {
        "schema_version": 1,
        "source_kind": "three_way_pdo_parity",
        "match": all(item["match"] for item in checks),
        "expected_slave_count": expected_slaves,
        "checks": checks,
        "evidence": {
            "esi_coe": "esi-coe-parity.json",
            "esi_igh": "esi-igh-parity.json",
            "coe_igh": "coe-igh-parity.json",
        },
        "scope": [
            "ESI-to-CoE validates assignment order and raw mapping entries.",
            "ESI-to-IgH validates SyncManager, direction, PDO and Entry order.",
            "CoE-to-IgH independently cross-checks PDO sets and ordered raw mapping entries.",
            "This is SIL/runtime configuration parity, not physical-drive HIL or motion acceptance.",
        ],
    }

def main():
    if len(sys.argv) != 6:
        print("usage: three_way_pdo_parity.py ESI_COE.json ESI_IGH.json COE_IGH.json EXPECTED_SLAVES OUTPUT.json", file=sys.stderr)
        return 2
    try:
        expected = int(sys.argv[4], 10)
        loaded = []
        for path in sys.argv[1:4]:
            value = json.loads(Path(path).read_text(encoding="utf-8"))
            if not isinstance(value, dict):
                raise ThreeWayParityError(f"report root must be an object: {path}")
            loaded.append(value)
        result = consolidate_reports(*loaded, expected)
        Path(sys.argv[5]).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError, json.JSONDecodeError, ThreeWayParityError) as exc:
        print(f"Three-way PDO parity error: {exc}", file=sys.stderr)
        return 2
    return 0 if result["match"] else 1

if __name__ == "__main__":
    raise SystemExit(main())
