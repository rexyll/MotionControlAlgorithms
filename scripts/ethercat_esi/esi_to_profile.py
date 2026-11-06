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


def _localized_names(
    parent: ET.Element,
    field: str,
    *,
    required: bool = False,
) -> list[dict[str, Any]]:
    """Read all localized NameType values; the schema default LcId is 1033."""
    elements = _children(parent, "Name")
    if required and not elements:
        raise EsiError(f"missing <Name> for {field}")
    result: list[dict[str, Any]] = []
    for element in elements:
        value = _text(element, field, required=True)
        raw_lcid = element.attrib.get("LcId")
        try:
            language_id = 1033 if raw_lcid is None else int(raw_lcid.strip(), 10)
        except ValueError as exc:
            raise EsiError(f"invalid LcId for {field}: {raw_lcid!r}") from exc
        result.append({"language_id": language_id, "text": value})
    return result


def _primary_name(names: list[dict[str, Any]]) -> str | None:
    """Prefer the ESI default language (1033), then preserve document order."""
    for item in names:
        if item["language_id"] == 1033:
            return item["text"]
    return names[0]["text"] if names else None


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



def _xs_integer(value: str | None, field: str, *, required: bool = False) -> int | None:
    """Parse an XML Schema xs:int value (decimal, signed 32-bit)."""
    if value is None or not value.strip():
        if required:
            raise EsiError(f"missing integer value for {field}")
        return None
    token = value.strip()
    if not re.fullmatch(r"[+-]?[0-9]+", token):
        raise EsiError(f"invalid {field}: {value!r}")
    result = int(token, 10)
    if not -(2**31) <= result <= 2**31 - 1:
        raise EsiError(f"{field} value {result} is outside the xs:int range")
    return result


def _validate_static_pdo_indices(
    pdos: list[dict[str, Any]],
    context: str,
) -> None:
    seen: dict[int, str] = {}
    for pdo in pdos:
        is_dynamic = (
            pdo["index_depend_on_slot"] or pdo["index_depend_on_slot_group"]
        )
        if is_dynamic:
            continue
        previous_direction = seen.get(pdo["index"])
        if previous_direction is not None:
            raise EsiError(
                f"{context} duplicate PDO index 0x{pdo['index']:04X} declared by "
                f"{previous_direction.upper()}PDO and {pdo['direction'].upper()}PDO"
            )
        seen[pdo["index"]] = pdo["direction"]


def _parse_module(module: ET.Element) -> dict[str, Any]:
    type_element = _one(module, "Type", required=True)
    assert type_element is not None
    module_ident_value = _number(
        type_element.attrib.get("ModuleIdent"), "Module.Type.ModuleIdent"
    )
    if module_ident_value is None:
        raise EsiError("missing ModuleIdent on Module.Type")
    module_ident = _require_range(
        module_ident_value, "Module.Type.ModuleIdent", 0xFFFFFFFF
    )
    if module_ident == 0:
        raise EsiError("ModuleIdent 0x00000000 is reserved for an empty slot")

    names = _localized_names(module, "Module.Name", required=True)
    pdos: list[dict[str, Any]] = []
    for child in list(module):
        tag = _local_name(child.tag)
        if tag == "RxPdo":
            pdos.append(_parse_pdo(child, "rx"))
        elif tag == "TxPdo":
            pdos.append(_parse_pdo(child, "tx"))
    _validate_static_pdo_indices(pdos, f"Module 0x{module_ident:08X}")

    safety_mappings: list[dict[str, Any]] = []
    for mapping in _children(module, "SafetyParaMapping"):
        mapping_index = _required_child_number(
            mapping, "Index", "Module.SafetyParaMapping.Index"
        )
        mapping_names = _localized_names(
            mapping, "Module.SafetyParaMapping.Name"
        )
        safety_mappings.append({
            "index": _require_range(
                mapping_index, "Module.SafetyParaMapping.Index", 0xFFFF
            ),
            "name": _primary_name(mapping_names),
            "localized_names": mapping_names,
            "fixed": _optional_boolean(
                mapping.attrib.get("Fixed"), "Module.SafetyParaMapping.Fixed"
            ),
            "mandatory": _optional_boolean(
                mapping.attrib.get("Mandatory"), "Module.SafetyParaMapping.Mandatory"
            ),
        })

    crc32 = _number(module.attrib.get("Crc32"), "Module.Crc32", required=False)
    if crc32 is not None:
        crc32 = _require_range(crc32, "Module.Crc32", 0xFFFFFFFF)
    return {
        "module_ident": module_ident,
        "type_name": _text(type_element, "Module.Type"),
        "name": _primary_name(names),
        "localized_names": names,
        "module_class": type_element.attrib.get("ModuleClass"),
        "module_pdo_group": _xs_integer(
            type_element.attrib.get("ModulePdoGroup"), "Module.Type.ModulePdoGroup"
        ),
        "sra_parameter_supported": _optional_boolean(
            type_element.attrib.get("SRA_ParameterSupported"),
            "Module.Type.SRA_ParameterSupported",
        ),
        "crc32": crc32,
        "pdos": pdos,
        "safety_parameter_mappings": safety_mappings,
        "has_mailbox": _one(module, "Mailbox") is not None,
    }


def _parse_slots(device: ET.Element) -> dict[str, Any] | None:
    slots_element = _one(device, "Slots")
    if slots_element is None:
        return None

    parsed_slots: list[dict[str, Any]] = []
    for ordinal, slot in enumerate(_children(slots_element, "Slot")):
        min_instances = _xs_integer(
            slot.attrib.get("MinInstances"), f"Slots.Slot[{ordinal}].MinInstances"
        )
        max_instances = _xs_integer(
            slot.attrib.get("MaxInstances"), f"Slots.Slot[{ordinal}].MaxInstances"
        )
        if min_instances is not None and min_instances < 0:
            raise EsiError(f"Slots.Slot[{ordinal}].MinInstances cannot be negative")
        if max_instances is not None and max_instances < 0:
            raise EsiError(f"Slots.Slot[{ordinal}].MaxInstances cannot be negative")
        if (
            min_instances is not None
            and max_instances is not None
            and min_instances > max_instances
        ):
            raise EsiError(
                f"Slots.Slot[{ordinal}] MinInstances exceeds MaxInstances"
            )

        names = _localized_names(slot, f"Slots.Slot[{ordinal}].Name")
        module_options: list[dict[str, Any]] = []
        seen_idents: set[int] = set()
        default_count = 0
        for module_element in _children(slot, "ModuleIdent"):
            ident_text = _text(
                module_element, f"Slots.Slot[{ordinal}].ModuleIdent", required=True
            )
            ident_value = _number(
                ident_text, f"Slots.Slot[{ordinal}].ModuleIdent"
            )
            assert ident_value is not None
            ident = _require_range(
                ident_value, f"Slots.Slot[{ordinal}].ModuleIdent", 0xFFFFFFFF
            )
            if ident in seen_idents:
                raise EsiError(
                    f"Slots.Slot[{ordinal}] repeats ModuleIdent 0x{ident:08X}"
                )
            seen_idents.add(ident)
            is_default = _optional_boolean(
                module_element.attrib.get("Default"),
                f"Slots.Slot[{ordinal}].ModuleIdent.Default",
            ) or False
            if is_default:
                default_count += 1
            module_options.append({
                "module_ident": ident,
                "default": is_default,
                "is_empty_slot": ident == 0,
            })
        if default_count > 1:
            raise EsiError(
                f"Slots.Slot[{ordinal}] declares more than one default ModuleIdent"
            )
        parsed_slots.append({
            "ordinal": ordinal,
            "module_list_subindex": ordinal + 1,
            "name": _primary_name(names),
            "localized_names": names,
            "min_instances": min_instances,
            "max_instances": max_instances,
            "module_options": module_options,
            "default_module_ident": next(
                (item["module_ident"] for item in module_options if item["default"]),
                None,
            ),
        })

    slot_group_data: list[dict[str, Any]] = []
    seen_groups: set[int] = set()
    for group in _children(slots_element, "SlotGroupData"):
        group_value = _number(
            group.attrib.get("SlotGroup"), "Slots.SlotGroupData.SlotGroup"
        )
        if group_value is None:
            raise EsiError("missing Slots.SlotGroupData.SlotGroup")
        group_ident = _require_range(
            group_value, "Slots.SlotGroupData.SlotGroup", 0xFFFFFFFF
        )
        if group_ident in seen_groups:
            raise EsiError(
                f"Slots contains duplicate SlotGroupData {group_ident}"
            )
        seen_groups.add(group_ident)
        names = _localized_names(group, "Slots.SlotGroupData.Name")
        slot_group_data.append({
            "slot_group": group_ident,
            "name": _primary_name(names),
            "localized_names": names,
        })

    module_pdo_groups: list[dict[str, Any]] = []
    for group in _children(slots_element, "ModulePdoGroup"):
        rx_pdo = _number(
            group.attrib.get("RxPdo"), "Slots.ModulePdoGroup.RxPdo", required=False
        )
        tx_pdo = _number(
            group.attrib.get("TxPdo"), "Slots.ModulePdoGroup.TxPdo", required=False
        )
        module_pdo_groups.append({
            "name": _text(group, "Slots.ModulePdoGroup"),
            "alignment": _xs_integer(
                group.attrib.get("Alignment"), "Slots.ModulePdoGroup.Alignment"
            ),
            "rx_pdo": (
                None if rx_pdo is None
                else _require_range(rx_pdo, "Slots.ModulePdoGroup.RxPdo", 0xFFFF)
            ),
            "tx_pdo": (
                None if tx_pdo is None
                else _require_range(tx_pdo, "Slots.ModulePdoGroup.TxPdo", 0xFFFF)
            ),
        })

    slot_selections: list[dict[str, Any]] = []
    for selection in _children(slots_element, "SlotSelections"):
        names = _localized_names(
            selection, "Slots.SlotSelections.Name", required=True
        )
        module_idents: list[int] = []
        for module_element in _children(selection, "ModuleIdent"):
            value = _text(
                module_element, "Slots.SlotSelections.ModuleIdent", required=True
            )
            ident_value = _number(
                value, "Slots.SlotSelections.ModuleIdent"
            )
            assert ident_value is not None
            ident = _require_range(
                ident_value, "Slots.SlotSelections.ModuleIdent", 0xFFFFFFFF
            )
            if ident in module_idents:
                raise EsiError(
                    f"Slots.SlotSelections repeats ModuleIdent 0x{ident:08X}"
                )
            module_idents.append(ident)
        if not module_idents:
            raise EsiError("Slots.SlotSelections must list at least one ModuleIdent")
        slot_selections.append({
            "name": _primary_name(names),
            "localized_names": names,
            "module_idents": module_idents,
        })

    transition = slots_element.attrib.get("DownloadModuleListTransition", "SP")
    if transition not in {"IP", "SP", "SO"}:
        raise EsiError(
            f"invalid Slots.DownloadModuleListTransition: {transition!r}"
        )

    def optional_hex_attribute(name: str) -> int | None:
        value = _number(slots_element.attrib.get(name), f"Slots.{name}", required=False)
        if value is None:
            return None
        return _require_range(value, f"Slots.{name}", 0xFFFFFFFF)

    return {
        "download_module_ident_list": _optional_boolean(
            slots_element.attrib.get("DownloadModuleIdentList"),
            "Slots.DownloadModuleIdentList",
        ),
        "download_module_address_list": _optional_boolean(
            slots_element.attrib.get("DownloadModuleAddressList"),
            "Slots.DownloadModuleAddressList",
        ),
        "download_module_list_transition": transition,
        "max_slot_count": optional_hex_attribute("MaxSlotCount"),
        "max_slot_group_count": optional_hex_attribute("MaxSlotGroupCount"),
        "slot_pdo_increment": optional_hex_attribute("SlotPdoIncrement"),
        "slot_group_pdo_increment": optional_hex_attribute("SlotGroupPdoIncrement"),
        "slot_index_increment": optional_hex_attribute("SlotIndexIncrement"),
        "slot_group_index_increment": optional_hex_attribute("SlotGroupIndexIncrement"),
        "identify_module_by": slots_element.attrib.get("IdentifyModuleBy"),
        "slots": parsed_slots,
        "slot_group_data": slot_group_data,
        "module_pdo_groups": module_pdo_groups,
        "slot_selections": slot_selections,
    }


def _parse_pdo(pdo: ET.Element, direction: str) -> dict[str, Any]:
    index_element = _one(pdo, "Index", required=True)
    assert index_element is not None
    index_text = _text(index_element, f"{direction}PDO.Index", required=True)
    index_value = _number(index_text, f"{direction}PDO.Index")
    assert index_value is not None
    pdo_index = _require_range(index_value, f"{direction}PDO.Index", 0xFFFF)
    allowed_range = (0x1600, 0x17FF) if direction == "rx" else (0x1A00, 0x1BFF)
    if not allowed_range[0] <= pdo_index <= allowed_range[1]:
        raise EsiError(
            f"{direction}PDO index 0x{pdo_index:04X} is outside the allowed "
            f"range 0x{allowed_range[0]:04X}..0x{allowed_range[1]:04X}"
        )
    pdo_index_depend_on_slot = _optional_boolean(
        index_element.attrib.get("DependOnSlot"),
        f"{direction}PDO.Index.DependOnSlot",
    ) or False
    pdo_index_depend_on_slot_group = _optional_boolean(
        index_element.attrib.get("DependOnSlotGroup"),
        f"{direction}PDO.Index.DependOnSlotGroup",
    ) or False
    pdo_names = _localized_names(pdo, f"{direction}PDO.Name", required=True)
    entries: list[dict[str, Any]] = []
    for entry in _children(pdo, "Entry"):
        index_element = _one(entry, "Index", required=True)
        assert index_element is not None
        index_text = _text(index_element, f"{direction}PDO.Entry.Index", required=True)
        index_value = _number(index_text, f"{direction}PDO.Entry.Index")
        assert index_value is not None
        index = _require_range(index_value, f"{direction}PDO.Entry.Index", 0xFFFF)
        index_depend_on_slot = _optional_boolean(
            index_element.attrib.get("DependOnSlot"),
            f"{direction}PDO.Entry.Index.DependOnSlot",
        ) or False
        index_depend_on_slot_group = _optional_boolean(
            index_element.attrib.get("DependOnSlotGroup"),
            f"{direction}PDO.Entry.Index.DependOnSlotGroup",
        ) or False
        sub_index_element = _one(entry, "SubIndex")
        if sub_index_element is None:
            if index != 0:
                raise EsiError(
                    f"{direction}PDO entry 0x{index:04X} is missing required SubIndex"
                )
            sub_index = 0
        else:
            sub_index_text = _text(
                sub_index_element, f"{direction}PDO.Entry.SubIndex", required=True
            )
            sub_index_value = _number(
                sub_index_text, f"{direction}PDO.Entry.SubIndex"
            )
            assert sub_index_value is not None
            sub_index = _require_range(
                sub_index_value, f"{direction}PDO.Entry.SubIndex", 0xFF
            )
        bit_length = _required_child_number(entry, "BitLen", f"{direction}PDO.Entry.BitLen")
        if not 1 <= bit_length <= 64:
            raise EsiError(f"{direction}PDO entry 0x{index:04X}:{sub_index:02X} has unsupported bit length {bit_length}")
        name_values = _localized_names(
            entry, f"{direction}PDO.Entry.Name", required=index != 0
        )
        data_type_element = _one(entry, "DataType")
        data_type_value = _text(
            data_type_element, f"{direction}PDO.Entry.DataType", required=index != 0
        )
        entries.append({
            "index": index,
            "index_depend_on_slot": index_depend_on_slot,
            "index_depend_on_slot_group": index_depend_on_slot_group,
            "sub_index": sub_index,
            "bit_length": bit_length,
            "name": _primary_name(name_values),
            "localized_names": name_values,
            "data_type": data_type_value,
            "data_scale": (
                data_type_element.attrib.get("DScale")
                if data_type_element is not None else None
            ),
            "swap_data": (
                data_type_element.attrib.get("SwapData")
                if data_type_element is not None else None
            ),
            "fixed": _optional_boolean(
                entry.attrib.get("Fixed"), f"{direction}PDO.Entry.Fixed"
            ),
            "safety_connection_number": _number(
                entry.attrib.get("SafetyConnNumber"),
                f"{direction}PDO.Entry.SafetyConnNumber",
                required=False,
            ),
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
        if not allowed_range[0] <= excluded_index <= allowed_range[1]:
            raise EsiError(
                f"{direction}PDO 0x{pdo_index:04X} excludes PDO index "
                f"0x{excluded_index:04X} outside the allowed "
                f"{direction.upper()}PDO range "
                f"0x{allowed_range[0]:04X}..0x{allowed_range[1]:04X}"
            )
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
        "name": _primary_name(pdo_names),
        "localized_names": pdo_names,
        "index_depend_on_slot": pdo_index_depend_on_slot,
        "index_depend_on_slot_group": pdo_index_depend_on_slot_group,
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
    device_names = _localized_names(device, "Device.Name")
    device_name = _primary_name(device_names) or _text(type_element, "Device.Type") or ""
    if not device_name:
        raise EsiError(f"device 0x{product_code:08X} has no name")

    pdos: list[dict[str, Any]] = []
    for child in list(device):
        tag = _local_name(child.tag)
        if tag == "RxPdo":
            pdos.append(_parse_pdo(child, "rx"))
        elif tag == "TxPdo":
            pdos.append(_parse_pdo(child, "tx"))

    slots = _parse_slots(device)
    if not pdos and slots is None:
        raise EsiError(
            f"device {device_name!r} has no PDO declarations and no Slots description"
        )
    _validate_static_pdo_indices(pdos, f"Device {device_name!r}")
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
        "localized_names": device_names,
        "type_name": _text(type_element, "Device.Type"),
        "module_pdo_group": _xs_integer(
            type_element.attrib.get("ModulePdoGroup"), "Device.Type.ModulePdoGroup"
        ),
        "sync_managers": sync_managers,
        "pdos": pdos,
        "slots": slots,
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
    if not device["pdos"] and device.get("slots") is not None:
        raise EsiError(
            "module-only device requires slot-resolved module PDO mappings; "
            "static CiA 402 candidate resolution cannot infer installed modules"
        )
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

    selected_pdo_keys = {
        (pdo["direction"], pdo["index"]) for pdo in selected
    }
    for pdo in device["pdos"]:
        if pdo["mandatory"] and (pdo["direction"], pdo["index"]) not in selected_pdo_keys:
            raise EsiError(
                f"mandatory {pdo['direction'].upper()}PDO 0x{pdo['index']:04X} "
                "is missing from the selected assignment"
            )

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
            if entry["index_depend_on_slot"] or entry["index_depend_on_slot_group"]:
                raise EsiError(
                    f"selected {pdo['direction'].upper()}PDO 0x{pdo['index']:04X} "
                    "contains a slot-dependent object-dictionary entry index "
                    "that cannot be resolved by the static candidate resolver"
                )
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
    vendor_names = _localized_names(vendor, "Vendor.Name", required=True)
    vendor_name = _primary_name(vendor_names)
    info_references: list[str] = []
    for reference in _children(root, "InfoReference"):
        info_references.append(
            _text(reference, "EtherCATInfo.InfoReference", required=True) or ""
        )
    modules_element = _one(root, "Modules")
    module_catalog = (
        [] if modules_element is None
        else [_parse_module(node) for node in _children(modules_element, "Module")]
    )
    seen_module_idents: set[int] = set()
    for module in module_catalog:
        ident = module["module_ident"]
        if ident in seen_module_idents:
            raise EsiError(f"duplicate ModuleIdent 0x{ident:08X} in root Modules catalog")
        seen_module_idents.add(ident)
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
        "vendor": {
            "id": vendor_id,
            "name": vendor_name,
            "localized_names": vendor_names,
        },
        "info_references": info_references,
        "modules": module_catalog,
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
