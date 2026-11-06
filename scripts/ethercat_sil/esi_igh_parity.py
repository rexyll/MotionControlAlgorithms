#!/usr/bin/env python3
"""Compare explicit ESI PDO expectations with IgH's SM/PDO/Entry discovery."""
from __future__ import annotations

import json
import sys
from pathlib import Path


class IghParityError(ValueError):
    pass


def compare_snapshots(esi: dict, igh: dict) -> dict:
    if esi.get("schema_version") != 1 or esi.get("source_kind") != "esi_expected":
        raise IghParityError("ESI input must be schema_version=1/source_kind=esi_expected")
    if esi.get("runtime_readback_performed") is not False:
        raise IghParityError("ESI expectation must not claim runtime readback")
    if igh.get("schema_version") != 1 or igh.get("source_kind") != "igh_discovery":
        raise IghParityError("IgH input must be schema_version=1/source_kind=igh_discovery")
    if igh.get("runtime_discovery_performed") is not True:
        raise IghParityError("IgH snapshot must identify runtime discovery")
    identity = esi.get("identity")
    expected = esi.get("mappings")
    slaves = igh.get("slaves")
    if not isinstance(identity, dict) or not isinstance(expected, list) or not expected:
        raise IghParityError("ESI snapshot has no identity or mapping entries")
    if not isinstance(slaves, list) or not slaves:
        raise IghParityError("IgH snapshot has no slave records")
    if any(not isinstance(item, dict) for item in expected + slaves):
        raise IghParityError("ESI/IgH records must be objects")

    expected_entries = []
    for item in expected:
        if not all(key in item for key in ("sync_manager", "direction", "pdo_index",
                                           "entry_position", "raw_mapping_entry")):
            raise IghParityError("ESI mapping record is incomplete")
        direction = {"rx": "output", "tx": "input"}.get(item["direction"])
        if direction is None:
            raise IghParityError("ESI mapping has unsupported direction")
        numeric = (item["sync_manager"], item["pdo_index"],
                   item["entry_position"], item["raw_mapping_entry"])
        if any(not isinstance(value, int) for value in numeric):
            raise IghParityError("ESI mapping contains a non-integer field")
        if not 0 <= numeric[0] <= 255 or not 0x1000 <= numeric[1] <= 0xFFFF:
            raise IghParityError("ESI mapping SM/PDO index is out of range")
        if not 1 <= numeric[2] <= 64 or not 0 <= numeric[3] <= 0xFFFFFFFF:
            raise IghParityError("ESI mapping entry position/raw word is out of range")
        expected_entries.append((
            numeric[0], direction, numeric[1], numeric[2], numeric[3],
        ))
    # IgH exposes SM-major order. Preserve ESI's original PDO/entry order within
    # each SM while grouping the ESI expectation into that independently exposed order.
    expected_entries = sorted(expected_entries, key=lambda entry: entry[0])

    ring_positions = [slave.get("ring_position") for slave in slaves]
    if any(not isinstance(ring, int) or ring < 0 for ring in ring_positions):
        raise IghParityError("IgH slave has invalid ring_position")
    if sorted(ring_positions) != list(range(len(slaves))) or len(set(ring_positions)) != len(ring_positions):
        raise IghParityError("IgH slave ring positions must be unique and contiguous from zero")

    results = []
    for slave in slaves:
        ring = slave["ring_position"]
        observed_identity = {
            "vendor_id": slave.get("vendor_id"),
            "product_code": slave.get("product_code"),
        }
        mismatches = []
        if observed_identity != identity:
            mismatches.append({"layer": "identity", "expected": identity, "observed": observed_identity})
        mappings = slave.get("mappings")
        if not isinstance(mappings, list) or any(not isinstance(item, dict) for item in mappings):
            raise IghParityError(f"ring {ring}: IgH mappings must be an array of objects")
        observed_entries = [
            (item.get("sync_manager"), item.get("direction"), item.get("pdo_index"),
             item.get("entry_position"), item.get("raw_mapping_entry"))
            for item in mappings
        ]
        if observed_entries != expected_entries:
            mismatches.append({
                "layer": "mapping", "expected": expected_entries, "observed": observed_entries,
            })
        results.append({"ring_position": ring, "match": not mismatches, "mismatches": mismatches})
    return {
        "schema_version": 1, "source_kind": "esi_igh_parity",
        "match": all(item["match"] for item in results),
        "slaves": results,
        "limitations": [
            "IgH discovery reports SM/PDO/Entry order but not CoE assignment objects or assignment subindices.",
            "ESI expectations are valid only for the explicitly selected non-slot PDO configuration.",
            "CoE assignment and raw mapping parity is checked separately by the ESI/CoE gate.",
        ],
    }


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: esi_igh_parity.py ESI_EXPECTED.json IGH_DISCOVERY.json REPORT.json", file=sys.stderr)
        return 2
    try:
        esi = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
        igh = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
        if not isinstance(esi, dict) or not isinstance(igh, dict):
            raise IghParityError("snapshot roots must be objects")
        report = compare_snapshots(esi, igh)
        Path(sys.argv[3]).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    except (OSError, json.JSONDecodeError, IghParityError) as exc:
        print(f"ESI/IgH parity error: {exc}", file=sys.stderr)
        return 2
    return 0 if report["match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
