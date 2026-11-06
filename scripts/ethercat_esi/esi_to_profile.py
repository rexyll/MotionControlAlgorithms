#!/usr/bin/env python3
"""Convert EtherCAT ESI XML into a normalized, offline JSON inventory.

This tool is intentionally outside the real-time C++ library. It uses only the
Python standard library and does not download, execute, or modify vendor files.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

MAX_ESI_BYTES = 32 * 1024 * 1024


class EsiError(ValueError):
    """Raised when an ESI document cannot be normalized safely."""


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _children(element: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in list(element) if _local_name(child.tag) == name]


def _one(element: ET.Element, name: str, *, required: bool = False) -> ET.Element | None:
    matches = _children(element, name)
    if len(matches) > 1:
        raise EsiError(f"expected at most one <{name}> below <{_local_name(element.tag)}>")
    if not matches:
        if required:
            raise EsiError(f"missing <{name}> below <{_local_name(element.tag)}>")
        return None
    return matches[0]


def _text(element: ET.Element | None, field: str, *, required: bool = False) -> str | None:
    value = (element.text or "").strip() if element is not None else ""
    if not value:
        if required:
            raise EsiError(f"missing text for {field}")
        return None
    return value


def _number(value: str | None, field: str, *, required: bool = True) -> int | None:
    if value is None or not value.strip():
        if required:
            raise EsiError(f"missing numeric value for {field}")
        return None
    token = value.strip()
    try:
        if token.lower().startswith("#x"):
            return int(token[2:], 16)
        if token.lower().startswith("0x"):
            return int(token[2:], 16)
        if re.fullmatch(r"[0-9]+", token):
            return int(token, 10)
    except ValueError as exc:
        raise EsiError(f"invalid {field}: {value!r}") from exc
    raise EsiError(f"invalid {field}: {value!r}")



def _optional_boolean(value: str | None, field: str) -> bool | None:
    """Parse the XML Schema xs:boolean lexical forms without guessing."""
    if value is None:
        return None
    token = value.strip()
    if token in ("true", "1"):
        return True
    if token in ("false", "0"):
        return False
    raise EsiError(f"invalid boolean value for {field}: {value!r}")


def _require_range(value: int, field: str, maximum: int) -> int:
    if not 0 <= value <= maximum:
        raise EsiError(f"{field} value {value} is outside 0..{maximum}")
    return value


def _required_child_number(parent: ET.Element, child_name: str, field: str) -> int:
    child = _one(parent, child_name, required=True)
    assert child is not None
    value = _text(child, field, required=True)
    result = _number(value, field)
    assert result is not None
    return result


def _parse_sync_managers(device: ET.Element) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for index, sm in enumerate(_children(device, "Sm")):
        result.append({
            "index": index,
            "name": (sm.text or "").strip() or None,
            "start_address": _number(sm.attrib.get("StartAddress"), "Sm.StartAddress", required=False),
            "control_byte": _number(sm.attrib.get("ControlByte"), "Sm.ControlByte", required=False),
            "enable": _number(sm.attrib.get("Enable"), "Sm.Enable", required=False),
            "min_size": _number(sm.attrib.get("MinSize"), "Sm.MinSize", required=False),
            "max_size": _number(sm.attrib.get("MaxSize"), "Sm.MaxSize", required=False),
            "default_size": _number(sm.attrib.get("DefaultSize"), "Sm.DefaultSize", required=False),
            "virtual": _optional_boolean(sm.attrib.get("Virtual"), "Sm.Virtual"),
            "op_only": _optional_boolean(sm.attrib.get("OpOnly"), "Sm.OpOnly"),
        })
    return result


def _parse_pdo(pdo: ET.Element, direction: str) -> dict[str, Any]:
    index_element = _one(pdo, "Index", required=True)
    assert index_element is not None
    index_text = _text(index_element, f"{direction}PDO.Index", required=True)
    index_value = _number(index_text, f"{direction}PDO.Index")
    assert index_value is not None
    pdo_index = _require_range(index_value, f"{direction}PDO.Index", 0xFFFF)
    index_depend_on_slot = _optional_boolean(
        index_element.attrib.get("DependOnSlot"),
        f"{direction}PDO.Index.DependOnSlot",
    ) or False
    index_depend_on_slot_group = _optional_boolean(
        index_element.attrib.get("DependOnSlotGroup"),
        f"{direction}PDO.Index.DependOnSlotGroup",
    ) or False
    entries: list[dict[str, Any]] = []
    for entry in _children(pdo, "Entry"):
        index = _require_range(_required_child_number(entry, "Index", f"{direction}PDO.Entry.Index"), f"{direction}PDO.Entry.Index", 0xFFFF)
        sub_index = _require_range(_required_child_number(entry, "SubIndex", f"{direction}PDO.Entry.SubIndex"), f"{direction}PDO.Entry.SubIndex", 0xFF)
        bit_length = _required_child_number(entry, "BitLen", f"{direction}PDO.Entry.BitLen")
        if not 1 <= bit_length <= 64:
            raise EsiError(f"{direction}PDO entry 0x{index:04X}:{sub_index:02X} has unsupported bit length {bit_length}")
        entries.append({
            "index": index,
            "sub_index": sub_index,
            "bit_length": bit_length,
            "name": _text(_one(entry, "Name"), f"{direction}PDO.Entry.Name"),
            "data_type": _text(_one(entry, "DataType"), f"{direction}PDO.Entry.DataType"),
            "is_padding": index == 0,
        })
    virtual = _optional_boolean(pdo.attrib.get("Virtual"), f"{direction}PDO.Virtual") or False
    if not entries and not virtual:
        raise EsiError(f"{direction}PDO 0x{pdo_index:04X} contains no entries")

    excluded_pdos: list[dict[str, Any]] = []
    seen_excluded_pdos: set[int] = set()
    for exclude in _children(pdo, "Exclude"):
        value = _text(exclude, f"{direction}PDO.Exclude", required=True)
        excluded_index = _number(value, f"{direction}PDO.Exclude")
        assert excluded_index is not None
        _require_range(excluded_index, f"{direction}PDO.Exclude", 0xFFFF)
        if excluded_index in seen_excluded_pdos:
            raise EsiError(
                f"{direction}PDO 0x{pdo_index:04X} has duplicate Exclude "
                f"reference 0x{excluded_index:04X}"
            )
        seen_excluded_pdos.add(excluded_index)
        excluded_pdos.append({
            "index": excluded_index,
            "depend_on_slot": _optional_boolean(
                exclude.attrib.get("DependOnSlot"),
                f"{direction}PDO.Exclude.DependOnSlot",
            ),
            "depend_on_slot_group": _optional_boolean(
                exclude.attrib.get("DependOnSlotGroup"),
                f"{direction}PDO.Exclude.DependOnSlotGroup",
            ),
        })

    excluded_sync_managers: list[int] = []
    for excluded_sm in _children(pdo, "ExcludedSm"):
        value = _text(excluded_sm, f"{direction}PDO.ExcludedSm", required=True)
        excluded_sm_index = _number(value, f"{direction}PDO.ExcludedSm")
        assert excluded_sm_index is not None
        _require_range(excluded_sm_index, f"{direction}PDO.ExcludedSm", 0xFF)
        if excluded_sm_index in excluded_sync_managers:
            raise EsiError(
                f"{direction}PDO 0x{pdo_index:04X} has duplicate ExcludedSm "
                f"reference {excluded_sm_index}"
            )
        excluded_sync_managers.append(excluded_sm_index)

    sm_value = _number(pdo.attrib.get("Sm"), f"{direction}PDO.Sm", required=False)
    sm_index = (
        None if sm_value is None
        else _require_range(sm_value, f"{direction}PDO.Sm", 0xFF)
    )
    mandatory = _optional_boolean(
        pdo.attrib.get("Mandatory"), f"{direction}PDO.Mandatory"
    ) or False
    if mandatory and sm_index is None:
        raise EsiError(
            f"{direction}PDO 0x{pdo_index:04X} is mandatory but has no "
            "default SyncManager assignment"
        )
    if sm_index is not None and sm_index in excluded_sync_managers:
        raise EsiError(
            f"{direction}PDO 0x{pdo_index:04X} declares SyncManager "
            f"{sm_index} as both its default and excluded assignment"
        )
    return {
        "direction": direction,
        "index": pdo_index,
        "index_depend_on_slot": index_depend_on_slot,
        "index_depend_on_slot_group": index_depend_on_slot_group,
        "name": _text(_one(pdo, "Name"), f"{direction}PDO.Name"),
        "sync_manager": sm_index,
        "fixed": _optional_boolean(pdo.attrib.get("Fixed"), f"{direction}PDO.Fixed") or False,
        "mandatory": mandatory,
        "virtual": virtual,
        "excluded_pdos": excluded_pdos,
        "excluded_sync_managers": excluded_sync_managers,
        "entries": entries,
    }


def _parse_device(device: ET.Element, vendor_id: int) -> dict[str, Any]:
    type_element = _one(device, "Type", required=True)
    assert type_element is not None
    product_code = _require_range(_number(type_element.attrib.get("ProductCode"), "Device.Type.ProductCode"), "Device.Type.ProductCode", 0xFFFFFFFF)
    revision = _require_range(_number(type_element.attrib.get("RevisionNo"), "Device.Type.RevisionNo"), "Device.Type.RevisionNo", 0xFFFFFFFF)
    assert product_code is not None and revision is not None
    device_name = _text(_one(device, "Name"), "Device.Name") or _text(type_element, "Device.Type") or ""
    if not device_name:
        raise EsiError(f"device 0x{product_code:08X} has no name")

    pdos: list[dict[str, Any]] = []
    for child in list(device):
        tag = _local_name(child.tag)
        if tag == "RxPdo":
            pdos.append(_parse_pdo(child, "rx"))
        elif tag == "TxPdo":
            pdos.append(_parse_pdo(child, "tx"))

    if not pdos:
        raise EsiError(f"device {device_name!r} has no RxPdo/TxPdo declarations")
    seen_static_pdo_indices: dict[int, str] = {}
    for pdo in pdos:
        is_dynamic = (
            pdo["index_depend_on_slot"] or pdo["index_depend_on_slot_group"]
        )
        if is_dynamic:
            continue
        previous_direction = seen_static_pdo_indices.get(pdo["index"])
        if previous_direction is not None:
            raise EsiError(
                f"duplicate PDO index 0x{pdo['index']:04X} declared by "
                f"{previous_direction.upper()}PDO and {pdo['direction'].upper()}PDO"
            )
        seen_static_pdo_indices[pdo["index"]] = pdo["direction"]
    sync_managers = _parse_sync_managers(device)
    for pdo in pdos:
        sync_manager = pdo["sync_manager"]
        if sync_managers and sync_manager is not None and sync_manager >= len(sync_managers):
            raise EsiError(
                f"{pdo['direction']}PDO 0x{pdo['index']:04X} references "
                f"SyncManager {sync_manager}, but the device declares "
                f"only {len(sync_managers)} SyncManager entries"
            )
        if sync_managers:
            for excluded_sm in pdo["excluded_sync_managers"]:
                if excluded_sm >= len(sync_managers):
                    raise EsiError(
                        f"{pdo['direction']}PDO 0x{pdo['index']:04X} excludes "
                        f"SyncManager {excluded_sm}, but the device declares "
                        f"only {len(sync_managers)} SyncManager entries"
                    )
    return {
        "identity": {
            "vendor_id": vendor_id,
            "product_code": product_code,
            "revision": revision,
        },
        "name": device_name,
        "type_name": _text(type_element, "Device.Type"),
        "sync_managers": sync_managers,
        "pdos": pdos,
    }



CIA402_OBJECTS = {
    "controlword": (0x6040, 0, 16, "rx", True),
    "target_position": (0x607A, 0, 32, "rx", False),
    "target_velocity": (0x60FF, 0, 32, "rx", False),
    "mode_of_operation": (0x6060, 0, 8, "rx", True),
    "statusword": (0x6041, 0, 16, "tx", True),
    "actual_position": (0x6064, 0, 32, "tx", False),
    "actual_velocity": (0x606C, 0, 32, "tx", False),
    "mode_display": (0x6061, 0, 8, "tx", True),
    "target_torque": (0x6071, 0, 16, "rx", False),
    "actual_torque": (0x6077, 0, 16, "tx", False),
}


def resolve_cia402_candidate(
    device: dict[str, Any], rx_pdo_indices: list[int], tx_pdo_indices: list[int]
) -> dict[str, Any]:
    """Resolve CiA 402 objects only from explicitly selected PDO declarations.

    ESI may describe alternative PDOs. The caller must select the PDO indices
    that represent the intended mapping; declarations alone are not active
    assignment readback or proof of runtime capability.
    """
    if not rx_pdo_indices or not tx_pdo_indices:
        raise EsiError("explicit RxPDO and TxPDO selections are both required")
    if any(
        pdo["index_depend_on_slot"] or pdo["index_depend_on_slot_group"]
        for pdo in device["pdos"]
    ):
        raise EsiError(
            "device declares slot-dependent PDO indices; the static candidate "
            "resolver requires slot-specific index resolution"
        )
    selected: list[dict[str, Any]] = []
    for direction, indices in (("rx", rx_pdo_indices), ("tx", tx_pdo_indices)):
        if len(indices) != len(set(indices)):
            raise EsiError(f"duplicate {direction.upper()}PDO selection")
        for pdo_index in indices:
            matches = [pdo for pdo in device["pdos"]
                       if pdo["direction"] == direction and pdo["index"] == pdo_index]
            if len(matches) != 1:
                raise EsiError(
                    f"selected {direction.upper()}PDO 0x{pdo_index:04X} "
                    f"has {len(matches)} matching declarations"
                )
            if matches[0]["sync_manager"] is None:
                raise EsiError(
                    f"selected {direction.upper()}PDO 0x{pdo_index:04X} "
                    "has no SyncManager association"
                )
            selected.append(matches[0])

    selected_pdo_indices = {pdo["index"] for pdo in selected}
    for pdo in selected:
        for exclusion in pdo["excluded_pdos"]:
            if exclusion["depend_on_slot"] or exclusion["depend_on_slot_group"]:
                raise EsiError(
                    f"selected {pdo['direction'].upper()}PDO 0x{pdo['index']:04X} "
                    "has a slot-dependent Exclude reference that cannot be "
                    "resolved by this static candidate resolver"
                )
            excluded_index = exclusion["index"]
            if excluded_index in selected_pdo_indices:
                raise EsiError(
                    f"selected PDO 0x{pdo['index']:04X} excludes selected "
                    f"PDO 0x{excluded_index:04X}"
                )

    found: dict[tuple[int, int], dict[str, Any]] = {}
    bit_offsets = {"rx": 0, "tx": 0}
    ordered_entries: list[dict[str, Any]] = []
    for pdo in selected:
        direction = pdo["direction"]
        for entry_number, entry in enumerate(pdo["entries"]):
            key = (entry["index"], entry["sub_index"])
            if entry["index"] != 0 and key in found:
                raise EsiError(
                    f"duplicate selected object 0x{key[0]:04X}:{key[1]:02X}"
                )
            normalized = {
                "index": entry["index"],
                "sub_index": entry["sub_index"],
                "bit_length": entry["bit_length"],
                "direction": direction,
                "pdo_index": pdo["index"],
                "sync_manager": pdo["sync_manager"],
                "entry_number": entry_number,
                "bit_offset": bit_offsets[direction],
                "is_padding": entry["is_padding"],
            }
            ordered_entries.append(normalized)
            if entry["index"] != 0:
                found[key] = normalized
            bit_offsets[direction] += entry["bit_length"]

    resolved: dict[str, Any] = {}
    for name, (index, subindex, width, direction, required) in CIA402_OBJECTS.items():
        match = found.get((index, subindex))
        if match is None:
            if required:
                raise EsiError(f"selected mapping is missing required CiA 402 object 0x{index:04X}:{subindex:02X}")
            continue
        if match["direction"] != direction or match["bit_length"] != width:
            raise EsiError(
                f"CiA 402 object {name} 0x{index:04X}:{subindex:02X} "
                f"must be {direction.upper()}PDO/{width}-bit"
            )
        resolved[name] = match

    def has_pair(command: str, feedback: str) -> bool:
        return command in resolved and feedback in resolved

    return {
        "profile": "CiA402",
        "identity": device["identity"],
        "selected_rx_pdo_indices": rx_pdo_indices,
        "selected_tx_pdo_indices": tx_pdo_indices,
        "bindings": resolved,
        "ordered_entries": ordered_entries,
        "supports": {
            "profile_position": has_pair("target_position", "actual_position"),
            "cyclic_synchronous_position": has_pair("target_position", "actual_position"),
            "profile_velocity": has_pair("target_velocity", "actual_velocity"),
            "cyclic_synchronous_velocity": has_pair("target_velocity", "actual_velocity"),
            "profile_torque": has_pair("target_torque", "actual_torque"),
            "cyclic_synchronous_torque": has_pair("target_torque", "actual_torque"),
            "homing": False,
        },
        "is_runtime_readback": False,
    }



def render_cia402_cpp_fixture(
    candidate: dict[str, Any], *, array_name: str = "kEsiCia402PdoEntries",
    include_common_declarations: bool = True,
) -> str:
    """Render a deterministic C++ fixture from the normalized selected PDO map."""
    identity = candidate["identity"]
    if include_common_declarations:
        lines = [
            "#pragma once", "", "#include <array>", "#include <cstdint>", "",
            "namespace mca::test_fixtures {",
            "struct EsiCia402PdoEntry {",
            "  std::uint16_t index;", "  std::uint8_t subindex;",
            "  std::uint8_t bit_length;",
            "  std::uint8_t direction;  // 0 = RxPDO/output, 1 = TxPDO/input",
            "  std::uint8_t sync_manager;", "  std::uint16_t pdo_index;",
            "  std::uint32_t bit_offset;", "};",
            f"inline constexpr std::uint32_t kVendorId = 0x{identity['vendor_id']:08X}U;",
            f"inline constexpr std::uint32_t kProductCode = 0x{identity['product_code']:08X}U;",
        ]
    else:
        lines = [
            "#pragma once", "", '#include "cia402_bindings_fixture.hpp"', "",
            "namespace mca::test_fixtures {",
        ]
    lines.append(
        "inline constexpr std::array<EsiCia402PdoEntry, "
        f"{len(candidate['ordered_entries'])}U> {array_name}{{{{"
    )
    for entry in candidate["ordered_entries"]:
        direction = 0 if entry["direction"] == "rx" else 1
        lines.append(
            "    {"
            f"0x{entry['index']:04X}U, {entry['sub_index']}U, "
            f"{entry['bit_length']}U, {direction}U, "
            f"{entry['sync_manager']}U, 0x{entry['pdo_index']:04X}U, "
            f"{entry['bit_offset']}U"
            "},"
        )
    lines.extend(["}};", "}  // namespace mca::test_fixtures", ""])
    return "\n".join(lines)

def parse_esi(path: Path) -> dict[str, Any]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise EsiError(f"cannot read ESI file {path}: {exc}") from exc
    if not raw:
        raise EsiError("ESI file is empty")
    if len(raw) > MAX_ESI_BYTES:
        raise EsiError(f"ESI file exceeds the {MAX_ESI_BYTES}-byte safety limit")
    # ElementTree does not resolve external entities, but reject DTD/entity
    # declarations as a simple defense against expansion-heavy untrusted input.
    if re.search(br"<!\s*(DOCTYPE|ENTITY)\b", raw, flags=re.IGNORECASE):
        raise EsiError("DOCTYPE and ENTITY declarations are not accepted")
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise EsiError(f"invalid ESI XML: {exc}") from exc
    if _local_name(root.tag) != "EtherCATInfo":
        raise EsiError("root element must be <EtherCATInfo>")

    vendor = _one(root, "Vendor", required=True)
    assert vendor is not None
    vendor_id = _require_range(_required_child_number(vendor, "Id", "Vendor.Id"), "Vendor.Id", 0xFFFFFFFF)
    vendor_name = _text(_one(vendor, "Name"), "Vendor.Name")
    descriptions = _one(root, "Descriptions", required=True)
    assert descriptions is not None
    devices_container = _one(descriptions, "Devices", required=True)
    assert devices_container is not None
    devices = [_parse_device(node, vendor_id) for node in _children(devices_container, "Device")]
    if not devices:
        raise EsiError("ESI contains no <Device> declarations")
    return {
        "schema_version": 1,
        "source": {"file_name": path.name, "sha256": hashlib.sha256(raw).hexdigest()},
        "vendor": {"id": vendor_id, "name": vendor_name},
        "devices": devices,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("esi", type=Path, help="input EtherCAT ESI XML file")
    parser.add_argument("-o", "--output", type=Path, help="write JSON to this path (default: stdout)")
    parser.add_argument("--product-code", help="include only devices matching this decimal, 0x..., or #x... product code")
    parser.add_argument("--rx-pdo", action="append", default=[], help="select an RxPDO index (repeatable; decimal, 0x..., or #x...) for CiA 402 candidate resolution")
    parser.add_argument("--tx-pdo", action="append", default=[], help="select a TxPDO index (repeatable; decimal, 0x..., or #x...) for CiA 402 candidate resolution")
    parser.add_argument("--emit-cpp-fixture", action="store_true", help="emit a C++ golden fixture instead of normalized JSON; requires explicit PDO selections")
    args = parser.parse_args(argv)
    try:
        document = parse_esi(args.esi)
        if args.product_code:
            wanted = _number(args.product_code, "--product-code")
            document["devices"] = [
                item for item in document["devices"]
                if item["identity"]["product_code"] == wanted
            ]
            if not document["devices"]:
                raise EsiError(f"no device has product code 0x{wanted:08X}")
        if args.rx_pdo or args.tx_pdo:
            if len(document["devices"]) != 1:
                raise EsiError("CiA 402 candidate resolution requires exactly one selected device; use --product-code")
            rx_indices = [_number(value, "--rx-pdo") for value in args.rx_pdo]
            tx_indices = [_number(value, "--tx-pdo") for value in args.tx_pdo]
            document["cia402_candidate"] = resolve_cia402_candidate(
                document["devices"][0], rx_indices, tx_indices
            )
        if args.emit_cpp_fixture:
            if "cia402_candidate" not in document:
                raise EsiError("--emit-cpp-fixture requires explicit --rx-pdo and --tx-pdo selections")
            rendered = render_cia402_cpp_fixture(document["cia402_candidate"])
        else:
            rendered = json.dumps(document, indent=2, sort_keys=True) + "\n"
        if args.output:
            args.output.write_text(rendered, encoding="utf-8")
        else:
            sys.stdout.write(rendered)
    except (EsiError, OSError) as exc:
        print(f"esi_to_profile: error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
