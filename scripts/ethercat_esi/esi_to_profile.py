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
        })
    return result


def _parse_pdo(pdo: ET.Element, direction: str) -> dict[str, Any]:
    pdo_index = _required_child_number(pdo, "Index", f"{direction}PDO.Index")
    entries: list[dict[str, Any]] = []
    for entry in _children(pdo, "Entry"):
        index = _required_child_number(entry, "Index", f"{direction}PDO.Entry.Index")
        sub_index = _required_child_number(entry, "SubIndex", f"{direction}PDO.Entry.SubIndex")
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
    if not entries:
        raise EsiError(f"{direction}PDO 0x{pdo_index:04X} contains no entries")
    sm_index = _number(pdo.attrib.get("Sm"), f"{direction}PDO.Sm", required=False)
    return {
        "direction": direction,
        "index": pdo_index,
        "name": _text(_one(pdo, "Name"), f"{direction}PDO.Name"),
        "sync_manager": sm_index,
        "fixed": pdo.attrib.get("Fixed", "").strip().lower() == "true",
        "mandatory": pdo.attrib.get("Mandatory", "").strip().lower() == "true",
        "entries": entries,
    }


def _parse_device(device: ET.Element, vendor_id: int) -> dict[str, Any]:
    type_element = _one(device, "Type", required=True)
    assert type_element is not None
    product_code = _number(type_element.attrib.get("ProductCode"), "Device.Type.ProductCode")
    revision = _number(type_element.attrib.get("RevisionNo"), "Device.Type.RevisionNo")
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
    return {
        "identity": {
            "vendor_id": vendor_id,
            "product_code": product_code,
            "revision": revision,
        },
        "name": device_name,
        "type_name": _text(type_element, "Device.Type"),
        "sync_managers": _parse_sync_managers(device),
        "pdos": pdos,
    }


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
    vendor_id = _required_child_number(vendor, "Id", "Vendor.Id")
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
