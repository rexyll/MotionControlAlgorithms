#!/usr/bin/env python3
"""Compare CoE assignment/mapping readback with IgH SM/PDO/Entry discovery."""
from __future__ import annotations
import json
import sys
from pathlib import Path


class CoeIghParityError(ValueError):
    pass


def _slaves(snapshot: dict, source: str) -> dict[int, dict]:
    kind = "coe_sdo_readback" if source == "CoE" else "igh_discovery"
    provenance = "runtime_readback_performed" if source == "CoE" else "runtime_discovery_performed"
    if not isinstance(snapshot, dict) or snapshot.get("schema_version") != 1:
        raise CoeIghParityError(f"{source} snapshot schema_version must be 1")
    if snapshot.get("source_kind") != kind or snapshot.get(provenance) is not True:
        raise CoeIghParityError(f"{source} snapshot provenance is invalid")
    records = snapshot.get("slaves")
    if not isinstance(records, list) or not records:
        raise CoeIghParityError(f"{source} snapshot has no slave records")
    result = {}
    for slave in records:
        if not isinstance(slave, dict):
            raise CoeIghParityError(f"{source} slave record must be an object")
        ring, vendor, product = (slave.get("ring_position"), slave.get("vendor_id"),
                                 slave.get("product_code"))
        if type(ring) is not int or ring < 0 or type(vendor) is not int or not 0 <= vendor <= 0xFFFFFFFF or type(product) is not int or not 0 <= product <= 0xFFFFFFFF:
            raise CoeIghParityError(f"{source} slave identity/ring position is invalid")
        if ring in result:
            raise CoeIghParityError(f"{source} has duplicate ring_position {ring}")
        result[ring] = slave
    if sorted(result) != list(range(len(result))):
        raise CoeIghParityError(f"{source} ring positions must be contiguous from zero")
    return result


def _coe_pdos(slave: dict, ring: int) -> dict[int, list[int]]:
    assignments, mappings = slave.get("assignments"), slave.get("mappings")
    if not isinstance(assignments, list) or not isinstance(mappings, list):
        raise CoeIghParityError(f"ring {ring}: CoE assignments/mappings must be arrays")
    assigned = {}
    for item in assignments:
        if not isinstance(item, dict):
            raise CoeIghParityError(f"ring {ring}: malformed CoE assignment")
        obj, pos, pdo = (item.get("assignment_object"), item.get("assignment_position"),
                         item.get("pdo_index"))
        if any(type(v) is not int for v in (obj, pos, pdo)) or obj not in (0x1C12, 0x1C13) or not 1 <= pos <= 16 or not 0x1000 <= pdo <= 0xFFFF:
            raise CoeIghParityError(f"ring {ring}: invalid CoE assignment")
        if pdo in assigned:
            raise CoeIghParityError(f"ring {ring}: duplicate assigned PDO 0x{pdo:04X}")
        assigned[pdo] = (obj, pos)
    result = {}
    for item in mappings:
        if not isinstance(item, dict):
            raise CoeIghParityError(f"ring {ring}: malformed CoE mapping")
        obj, pos, pdo, entry, raw = (item.get("assignment_object"), item.get("assignment_position"),
                                     item.get("pdo_index"), item.get("entry_position"),
                                     item.get("raw_mapping_entry"))
        if any(type(v) is not int for v in (obj, pos, pdo, entry, raw)) or assigned.get(pdo) != (obj, pos):
            raise CoeIghParityError(f"ring {ring}: CoE mapping references an invalid assignment/PDO")
        if not 1 <= entry <= 64 or not 0 <= raw <= 0xFFFFFFFF or (raw & 0xFF) == 0 or (((raw >> 16) & 0xFFFF) == 0 and ((raw >> 8) & 0xFF) != 0):
            raise CoeIghParityError(f"ring {ring}: invalid CoE mapping word/entry position")
        entries = result.setdefault(pdo, [])
        if entry != len(entries) + 1:
            raise CoeIghParityError(f"ring {ring}: CoE entry order/gap in PDO 0x{pdo:04X}")
        entries.append(raw)
    if set(result) != set(assigned):
        raise CoeIghParityError(f"ring {ring}: CoE mapping PDO set differs from assignments")
    return result


def _igh_pdos(slave: dict, ring: int) -> dict[int, list[int]]:
    mappings = slave.get("mappings")
    if not isinstance(mappings, list) or not mappings:
        raise CoeIghParityError(f"ring {ring}: IgH mappings must be a non-empty array")
    result, closed = {}, set()
    previous = None
    for item in mappings:
        if not isinstance(item, dict):
            raise CoeIghParityError(f"ring {ring}: malformed IgH mapping")
        sm, direction, pdo, entry, raw = (item.get("sync_manager"), item.get("direction"),
                                          item.get("pdo_index"), item.get("entry_position"),
                                          item.get("raw_mapping_entry"))
        if any(type(v) is not int for v in (sm, pdo, entry, raw)) or not 0 <= sm <= 255 or direction not in ("input", "output") or not 0x1000 <= pdo <= 0xFFFF or not 1 <= entry <= 64 or not 0 <= raw <= 0xFFFFFFFF or (raw & 0xFF) == 0 or (((raw >> 16) & 0xFFFF) == 0 and ((raw >> 8) & 0xFF) != 0):
            raise CoeIghParityError(f"ring {ring}: invalid IgH SM/direction/PDO/entry")
        key = (sm, pdo)
        if key != previous:
            if pdo in result or pdo in closed:
                raise CoeIghParityError(f"ring {ring}: IgH PDO group is duplicated or non-contiguous")
            if previous is not None:
                closed.add(previous[1])
            previous = key
        entries = result.setdefault(pdo, [])
        if entry != len(entries) + 1:
            raise CoeIghParityError(f"ring {ring}: IgH entry order/gap in PDO 0x{pdo:04X}")
        entries.append(raw)
    return result


def compare_snapshots(coe: dict, igh: dict) -> dict:
    coe_slaves, igh_slaves = _slaves(coe, "CoE"), _slaves(igh, "IgH")
    if set(coe_slaves) != set(igh_slaves):
        raise CoeIghParityError("CoE and IgH slave ring-position sets differ")
    results = []
    for ring in sorted(coe_slaves):
        c, h = coe_slaves[ring], igh_slaves[ring]
        ci = (c["vendor_id"], c["product_code"])
        hi = (h["vendor_id"], h["product_code"])
        if ci != hi:
            results.append({"ring_position": ring, "match": False,
                            "mismatches": [{"layer": "identity", "coe": ci, "igh": hi}]})
            continue
        cp, hp = _coe_pdos(c, ring), _igh_pdos(h, ring)
        mismatches = []
        if set(cp) != set(hp):
            mismatches.append({"layer": "pdo_set", "coe_pdo_indices": list(cp),
                               "igh_pdo_indices": list(hp)})
        for pdo in cp:
            if pdo in hp and cp[pdo] != hp[pdo]:
                mismatches.append({"layer": "mapping_entries", "pdo_index": pdo,
                                   "coe_raw_entries": cp[pdo], "igh_raw_entries": hp[pdo]})
        results.append({"ring_position": ring, "match": not mismatches, "mismatches": mismatches})
    return {
        "schema_version": 1, "source_kind": "coe_igh_parity",
        "match": all(item["match"] for item in results), "slaves": results,
        "limitations": [
            "CoE provides assignment order and raw mapping entries, but not SyncManager or direction.",
            "IgH provides SyncManager, direction and PDO/Entry order, but not CoE assignment objects or subindices.",
            "The join uses ring position, slave identity and PDO index; assignment order is never inferred from IgH.",
        ],
    }


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: coe_igh_parity.py COE_READBACK.json IGH_DISCOVERY.json REPORT.json", file=sys.stderr)
        return 2
    try:
        coe = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
        igh = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
        report = compare_snapshots(coe, igh)
        Path(sys.argv[3]).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    except (OSError, json.JSONDecodeError, CoeIghParityError) as exc:
        print(f"CoE/IgH parity error: {exc}", file=sys.stderr)
        return 2
    return 0 if report["match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
