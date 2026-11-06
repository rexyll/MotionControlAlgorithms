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



def _optional_hex_attribute(
    element: ET.Element,
    attribute: str,
    field: str,
) -> int | None:
    value = _number(element.attrib.get(attribute), field, required=False)
    if value is None:
        return None
    return _require_range(value, field, 0xFFFFFFFF)


def _required_hex_attribute(
    element: ET.Element,
    attribute: str,
    field: str,
) -> int:
    value = _number(element.attrib.get(attribute), field, required=True)
    assert value is not None
    return _require_range(value, field, 0xFFFFFFFF)


def _parse_image_choice(
    parent: ET.Element,
    field: str,
) -> dict[str, str] | None:
    choices = {
        "Image16x14": "image_16x14",
        "ImageFile16x14": "image_file_16x14",
        "ImageData16x14": "image_data_16x14",
    }
    selected = [(element_name, key, _children(parent, element_name))
                for element_name, key in choices.items()]
    present = [(element_name, key, matches[0]) for element_name, key, matches in selected if matches]
    if len(present) > 1 or any(len(matches) > 1 for _, _, matches in selected):
        raise EsiError(f"{field} must contain at most one image representation")
    if not present:
        return None
    element_name, key, element = present[0]
    value = _text(element, f"{field}.{element_name}", required=True)
    assert value is not None
    return {key: value}


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


def _optional_nonnegative_xs_integer(value: str | None, field: str) -> int | None:
    result = _xs_integer(value, field)
    if result is not None and result < 0:
        raise EsiError(f"{field} value {result} must be non-negative")
    return result


def _parse_module_mailbox(module: ET.Element) -> dict[str, Any] | None:
    mailbox = _one(module, "Mailbox")
    if mailbox is None:
        return None

    unsupported_children = [
        _local_name(child.tag)
        for child in list(mailbox)
        if _local_name(child.tag) != "CoE"
    ]
    if unsupported_children:
        raise EsiError(
            "Module.Mailbox supports only the CoE child in ModuleType; "
            f"unsupported child <{unsupported_children[0]}>"
        )
    coe = _one(mailbox, "CoE")
    init_commands: list[dict[str, Any]] = []
    if coe is not None:
        allowed_transitions = {"IP", "PS", "SO", "SP", "OP", "OS"}
        for ordinal, command in enumerate(_children(coe, "InitCmd")):
            field = f"Module.Mailbox.CoE.InitCmd[{ordinal}]"
            transitions = [
                _text(element, f"{field}.Transition", required=True)
                for element in _children(command, "Transition")
            ]
            if not transitions:
                raise EsiError(f"{field} must specify at least one Transition")
            invalid = [item for item in transitions if item not in allowed_transitions]
            if invalid:
                raise EsiError(
                    f"{field} contains invalid Transition {invalid[0]!r}"
                )

            index_element = _one(command, "Index", required=True)
            assert index_element is not None
            index_text = _text(index_element, f"{field}.Index", required=True)
            index_value = _number(index_text, f"{field}.Index")
            assert index_value is not None
            index = _require_range(index_value, f"{field}.Index", 0xFFFF)
            sub_index = _required_child_number(command, "SubIndex", f"{field}.SubIndex")
            sub_index = _require_range(sub_index, f"{field}.SubIndex", 0xFF)

            data_element = _one(command, "Data", required=True)
            assert data_element is not None
            raw_data = re.sub(r"\s+", "", data_element.text or "")
            if len(raw_data) % 2 != 0 or not re.fullmatch(r"[0-9A-Fa-f]*", raw_data):
                raise EsiError(f"{field}.Data is not valid xs:hexBinary data")
            init_commands.append({
                "fixed": _optional_boolean(
                    command.attrib.get("Fixed"), f"{field}.Fixed"
                ),
                "complete_access": _optional_boolean(
                    command.attrib.get("CompleteAccess"), f"{field}.CompleteAccess"
                ),
                "overwritten_by_module": _optional_boolean(
                    command.attrib.get("OverwrittenByModule"),
                    f"{field}.OverwrittenByModule",
                ),
                "transitions": transitions,
                "index": index,
                "index_depend_on_slot": _optional_boolean(
                    index_element.attrib.get("DependOnSlot"),
                    f"{field}.Index.DependOnSlot",
                ) or False,
                "index_depend_on_slot_group": _optional_boolean(
                    index_element.attrib.get("DependOnSlotGroup"),
                    f"{field}.Index.DependOnSlotGroup",
                ) or False,
                "sub_index": sub_index,
                "data_hex": raw_data.upper(),
                "adapt_automatically": _optional_boolean(
                    data_element.attrib.get("AdaptAutomatically"),
                    f"{field}.Data.AdaptAutomatically",
                ),
                "comment": _text(_one(command, "Comment"), f"{field}.Comment"),
            })

    return {
        "attributes": {
            "sdo_info": _optional_boolean(
                mailbox.attrib.get("SdoInfo"), "Module.Mailbox.SdoInfo"
            ),
            "pdo_assign": _optional_boolean(
                mailbox.attrib.get("PdoAssign"), "Module.Mailbox.PdoAssign"
            ),
            "pdo_config": _optional_boolean(
                mailbox.attrib.get("PdoConfig"), "Module.Mailbox.PdoConfig"
            ),
            "pdo_upload": _optional_boolean(
                mailbox.attrib.get("PdoUpload"), "Module.Mailbox.PdoUpload"
            ),
            "complete_access": _optional_boolean(
                mailbox.attrib.get("CompleteAccess"),
                "Module.Mailbox.CompleteAccess",
            ),
            "segmented_sdo": _optional_boolean(
                mailbox.attrib.get("SegmentedSdo"), "Module.Mailbox.SegmentedSdo"
            ),
            "module_od": _optional_boolean(
                mailbox.attrib.get("ModuleOD"), "Module.Mailbox.ModuleOD"
            ),
            "eds_file": mailbox.attrib.get("EdsFile"),
            "ds402_channels": _xs_integer(
                mailbox.attrib.get("DS402Channels"), "Module.Mailbox.DS402Channels"
            ),
            "diag_history": _optional_boolean(
                mailbox.attrib.get("DiagHistory"), "Module.Mailbox.DiagHistory"
            ),
        },
        "has_coe": coe is not None,
        "protocols": {"coe": coe is not None},
        "coe_init_commands": init_commands,
    }


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

    safety_mappings = [
        _parse_safety_parameter_mapping(mapping)
        for mapping in _children(module, "SafetyParaMapping")
    ]

    mailbox = _parse_module_mailbox(module)
    crc32 = _number(module.attrib.get("Crc32"), "Module.Crc32", required=False)
    if crc32 is not None:
        crc32 = _require_range(crc32, "Module.Crc32", 0xFFFFFFFF)
    return {
        "module_ident": module_ident,
        "type_name": _text(type_element, "Module.Type"),
        "name": _primary_name(names),
        "localized_names": names,
        "module_class": type_element.attrib.get("ModuleClass"),
        "module_pdo_group": _optional_nonnegative_xs_integer(
            type_element.attrib.get("ModulePdoGroup"), "Module.Type.ModulePdoGroup"
        ),
        "sra_parameter_supported": _optional_boolean(
            type_element.attrib.get("SRA_ParameterSupported"),
            "Module.Type.SRA_ParameterSupported",
        ),
        "crc32": crc32,
        "pdos": pdos,
        "safety_parameter_mappings": safety_mappings,
        "is_safety_relevant": bool(safety_mappings),
        "has_mailbox": mailbox is not None,
        "mailbox": mailbox,
    }


def _parse_slots(
    device: ET.Element,
    default_vendor_id: int,
) -> dict[str, Any] | None:
    slots_element = _one(device, "Slots")
    if slots_element is None:
        return None

    root_increments = {
        "slot_pdo_increment": _optional_hex_attribute(
            slots_element, "SlotPdoIncrement", "Slots.SlotPdoIncrement"
        ),
        "slot_group_pdo_increment": _optional_hex_attribute(
            slots_element, "SlotGroupPdoIncrement", "Slots.SlotGroupPdoIncrement"
        ),
        "slot_index_increment": _optional_hex_attribute(
            slots_element, "SlotIndexIncrement", "Slots.SlotIndexIncrement"
        ),
        "slot_group_index_increment": _optional_hex_attribute(
            slots_element, "SlotGroupIndexIncrement", "Slots.SlotGroupIndexIncrement"
        ),
    }

    parsed_slots: list[dict[str, Any]] = []
    for ordinal, slot in enumerate(_children(slots_element, "Slot")):
        slot_field = f"Slots.Slot[{ordinal}]"
        min_instances = _required_hex_attribute(
            slot, "MinInstances", f"{slot_field}.MinInstances"
        )
        max_instances = _required_hex_attribute(
            slot, "MaxInstances", f"{slot_field}.MaxInstances"
        )
        if min_instances > max_instances:
            raise EsiError(f"{slot_field} MinInstances exceeds MaxInstances")

        names = _localized_names(slot, f"{slot_field}.Name")
        module_options: list[dict[str, Any]] = []
        seen_idents: set[int] = set()
        default_count = 0
        module_ident_elements = _children(slot, "ModuleIdent")
        module_class_elements = _children(slot, "ModuleClass")
        if module_ident_elements and module_class_elements:
            raise EsiError(
                f"{slot_field} cannot mix ModuleIdent and ModuleClass choices"
            )
        if not module_ident_elements and not module_class_elements:
            raise EsiError(
                f"{slot_field} must define at least one ModuleIdent or ModuleClass option"
            )

        for module_element in module_ident_elements:
            ident_text = _text(
                module_element, f"{slot_field}.ModuleIdent", required=True
            )
            ident_value = _number(ident_text, f"{slot_field}.ModuleIdent")
            assert ident_value is not None
            ident = _require_range(
                ident_value, f"{slot_field}.ModuleIdent", 0xFFFFFFFF
            )
            if ident in seen_idents:
                raise EsiError(
                    f"{slot_field} repeats ModuleIdent 0x{ident:08X}"
                )
            seen_idents.add(ident)

            default_value = _optional_hex_attribute(
                module_element, "Default", f"{slot_field}.ModuleIdent.Default"
            )
            is_default = default_value is not None and default_value != 0
            if is_default:
                default_count += 1
            module_options.append({
                "module_ident": ident,
                "default": is_default,
                "default_value": default_value,
                "is_empty_slot": ident == 0,
            })
        if default_count > 1:
            raise EsiError(
                f"{slot_field} declares more than one default ModuleIdent"
            )

        module_class_options: list[dict[str, Any]] = []
        for class_ordinal, module_class in enumerate(module_class_elements):
            class_field = f"{slot_field}.ModuleClass[{class_ordinal}]"
            class_name = _text(
                _one(module_class, "Class", required=True),
                f"{class_field}.Class",
                required=True,
            )
            vendor_id_element = _one(module_class, "VendorId")
            vendor_id_text = _text(
                vendor_id_element, f"{class_field}.VendorId"
            )
            vendor_id_value = _number(
                vendor_id_text, f"{class_field}.VendorId", required=False
            )
            vendor_id_explicit = (
                None if vendor_id_value is None
                else _require_range(
                    vendor_id_value, f"{class_field}.VendorId", 0xFFFFFFFF
                )
            )
            vendor_id = (
                default_vendor_id
                if vendor_id_explicit is None else vendor_id_explicit
            )
            class_names = _localized_names(
                module_class, f"{class_field}.Name"
            )
            image = _parse_image_choice(module_class, class_field)
            module_class_options.append({
                "class": class_name,
                "vendor_id": vendor_id,
                "vendor_id_explicit": vendor_id_explicit,
                "name": _primary_name(class_names),
                "localized_names": class_names,
                "image": image,
            })

        tree_view = slot.attrib.get("TreeView")
        if tree_view is not None and tree_view not in {"SLOTGROUP", "SLOT", "PDO"}:
            raise EsiError(f"{slot_field} has invalid TreeView {tree_view!r}")

        local_increments = {
            "slot_pdo_increment": _optional_hex_attribute(
                slot, "SlotPdoIncrement", f"{slot_field}.SlotPdoIncrement"
            ),
            "slot_group_pdo_increment": _optional_hex_attribute(
                slot, "SlotGroupPdoIncrement",
                f"{slot_field}.SlotGroupPdoIncrement"
            ),
            "slot_index_increment": _optional_hex_attribute(
                slot, "SlotIndexIncrement", f"{slot_field}.SlotIndexIncrement"
            ),
            "slot_group_index_increment": _optional_hex_attribute(
                slot, "SlotGroupIndexIncrement",
                f"{slot_field}.SlotGroupIndexIncrement"
            ),
        }
        effective_increments = {
            key: value if value is not None else root_increments[key]
            for key, value in local_increments.items()
        }

        parsed_slots.append({
            "ordinal": ordinal,
            "module_list_subindex": ordinal + 1,
            "name": _primary_name(names),
            "localized_names": names,
            "min_instances": min_instances,
            "max_instances": max_instances,
            "slot_group": _optional_hex_attribute(
                slot, "SlotGroup", f"{slot_field}.SlotGroup"
            ),
            "tree_view": tree_view,
            "module_options": module_options,
            "module_class_options": module_class_options,
            "default_module_ident": next(
                (item["module_ident"] for item in module_options if item["default"]),
                None,
            ),
            "increments": local_increments,
            "effective_increments": effective_increments,
            "image": _parse_image_choice(slot, slot_field),
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
            "image": _parse_image_choice(group, "Slots.SlotGroupData"),
        })

    module_pdo_groups: list[dict[str, Any]] = []
    seen_alignment_rx_pdos: dict[int, int] = {}
    seen_alignment_tx_pdos: dict[int, int] = {}
    for group_ordinal, group in enumerate(_children(slots_element, "ModulePdoGroup")):
        alignment = _xs_integer(
            group.attrib.get("Alignment"), "Slots.ModulePdoGroup.Alignment"
        )
        if alignment is not None and alignment < 0:
            raise EsiError(
                f"Slots.ModulePdoGroup[{group_ordinal}] Alignment must be non-negative"
            )
        rx_pdo = _number(
            group.attrib.get("RxPdo"), "Slots.ModulePdoGroup.RxPdo", required=False
        )
        tx_pdo = _number(
            group.attrib.get("TxPdo"), "Slots.ModulePdoGroup.TxPdo", required=False
        )
        if rx_pdo is not None and not 0x1600 <= rx_pdo <= 0x17FF:
            raise EsiError(
                f"Slots.ModulePdoGroup RxPdo 0x{rx_pdo:04X} is outside "
                "0x1600..0x17FF"
            )
        if tx_pdo is not None and not 0x1A00 <= tx_pdo <= 0x1BFF:
            raise EsiError(
                f"Slots.ModulePdoGroup TxPdo 0x{tx_pdo:04X} is outside "
                "0x1A00..0x1BFF"
            )
        if rx_pdo is not None:
            previous = seen_alignment_rx_pdos.get(rx_pdo)
            if previous is not None:
                raise EsiError(
                    f"Slots.ModulePdoGroup RxPdo 0x{rx_pdo:04X} is duplicated "
                    f"by groups {previous} and {group_ordinal}; alignment PDO "
                    "indices must be unique"
                )
            seen_alignment_rx_pdos[rx_pdo] = group_ordinal
        if tx_pdo is not None:
            previous = seen_alignment_tx_pdos.get(tx_pdo)
            if previous is not None:
                raise EsiError(
                    f"Slots.ModulePdoGroup TxPdo 0x{tx_pdo:04X} is duplicated "
                    f"by groups {previous} and {group_ordinal}; alignment PDO "
                    "indices must be unique"
                )
            seen_alignment_tx_pdos[tx_pdo] = group_ordinal
        module_pdo_groups.append({
            "index": group_ordinal,
            "name": _text(group, "Slots.ModulePdoGroup"),
            "alignment": alignment,
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
    identify_module_by = slots_element.attrib.get("IdentifyModuleBy")
    if identify_module_by is not None and identify_module_by not in {
        "ModuleIdent", "IdentityObjekt"
    }:
        raise EsiError(
            f"invalid Slots.IdentifyModuleBy: {identify_module_by!r}"
        )

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
        "max_slot_count": _optional_hex_attribute(
            slots_element, "MaxSlotCount", "Slots.MaxSlotCount"
        ),
        "max_slot_group_count": _optional_hex_attribute(
            slots_element, "MaxSlotGroupCount", "Slots.MaxSlotGroupCount"
        ),
        **root_increments,
        "identify_module_by": identify_module_by,
        "slots": parsed_slots,
        "slot_group_data": slot_group_data,
        "module_pdo_groups": module_pdo_groups,
        "slot_selections": slot_selections,
    }



def _parse_pdo_entry(entry: ET.Element, field: str) -> dict[str, Any]:
    index_element = _one(entry, "Index", required=True)
    assert index_element is not None
    index_text = _text(index_element, f"{field}.Index", required=True)
    index_value = _number(index_text, f"{field}.Index")
    assert index_value is not None
    index = _require_range(index_value, f"{field}.Index", 0xFFFF)
    index_depend_on_slot = _optional_boolean(
        index_element.attrib.get("DependOnSlot"),
        f"{field}.Index.DependOnSlot",
    ) or False
    index_depend_on_slot_group = _optional_boolean(
        index_element.attrib.get("DependOnSlotGroup"),
        f"{field}.Index.DependOnSlotGroup",
    ) or False

    sub_index_element = _one(entry, "SubIndex")
    sub_index_present = sub_index_element is not None
    if sub_index_element is None:
        # SubIndex is optional in EntryType. Preserve its omission separately,
        # while using the standard zero value for an omitted sub-index.
        sub_index = 0
    else:
        sub_index_text = _text(
            sub_index_element, f"{field}.SubIndex", required=True
        )
        sub_index_value = _number(sub_index_text, f"{field}.SubIndex")
        assert sub_index_value is not None
        sub_index = _require_range(sub_index_value, f"{field}.SubIndex", 0xFF)

    bit_length = _required_child_number(entry, "BitLen", f"{field}.BitLen")
    if not 1 <= bit_length <= 64:
        raise EsiError(
            f"{field} 0x{index:04X}:{sub_index:02X} has unsupported bit length {bit_length}"
        )

    name_values = _localized_names(entry, f"{field}.Name")
    data_type_element = _one(entry, "DataType")
    data_type_value = _text(data_type_element, f"{field}.DataType")
    return {
        "index": index,
        "index_depend_on_slot": index_depend_on_slot,
        "index_depend_on_slot_group": index_depend_on_slot_group,
        "sub_index": sub_index,
        "sub_index_present": sub_index_present,
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
            entry.attrib.get("Fixed"), f"{field}.Fixed"
        ),
        "safety_connection_number": _xs_integer(
            entry.attrib.get("SafetyConnNumber"), f"{field}.SafetyConnNumber"
        ),
        "comment": _text(_one(entry, "Comment"), f"{field}.Comment"),
        "is_padding": index == 0,
    }


def _parse_pdo_type_attributes(
    element: ET.Element,
    field: str,
) -> dict[str, Any]:
    """Normalize attributes shared by RxPDO, TxPDO and SafetyParaMapping."""
    safety_pdo_type = element.attrib.get("SafetyPdoType")
    allowed_safety_pdo_types = {
        "FSoESlave",
        "FSoEMultipleSlave",
        "FSoEMaster",
        "FSoEMultipleMaster",
    }
    if safety_pdo_type is not None and safety_pdo_type not in allowed_safety_pdo_types:
        raise EsiError(f"invalid {field}.SafetyPdoType: {safety_pdo_type!r}")

    sync_manager = _xs_integer(element.attrib.get("Sm"), f"{field}.Sm")
    if sync_manager is not None and not 0 <= sync_manager <= 0xFF:
        raise EsiError(f"{field}.Sm value {sync_manager} is outside 0..255")
    return {
        "fixed": _optional_boolean(element.attrib.get("Fixed"), f"{field}.Fixed"),
        "mandatory": _optional_boolean(
            element.attrib.get("Mandatory"), f"{field}.Mandatory"
        ),
        "virtual": _optional_boolean(element.attrib.get("Virtual"), f"{field}.Virtual"),
        "sync_manager": sync_manager,
        "sync_unit": _xs_integer(element.attrib.get("Su"), f"{field}.Su"),
        "pdo_order": _xs_integer(element.attrib.get("PdoOrder"), f"{field}.PdoOrder"),
        "oversampling_factor": _xs_integer(
            element.attrib.get("OSFac"), f"{field}.OSFac"
        ),
        "oversampling_min": _xs_integer(
            element.attrib.get("OSMin"), f"{field}.OSMin"
        ),
        "oversampling_max": _xs_integer(
            element.attrib.get("OSMax"), f"{field}.OSMax"
        ),
        "oversampling_index_increment": _xs_integer(
            element.attrib.get("OSIndexInc"), f"{field}.OSIndexInc"
        ),
        "overwritten_by_module": _optional_boolean(
            element.attrib.get("OverwrittenByModule"),
            f"{field}.OverwrittenByModule",
        ),
        "sra_parameter": _optional_boolean(
            element.attrib.get("SRA_Parameter"), f"{field}.SRA_Parameter"
        ),
        "safety_pdo_type": safety_pdo_type,
        "safety_connection_number": _xs_integer(
            element.attrib.get("SafetyConnNumber"), f"{field}.SafetyConnNumber"
        ),
    }


def _parse_safety_parameter_mapping(mapping: ET.Element) -> dict[str, Any]:
    field = "Module.SafetyParaMapping"
    index_element = _one(mapping, "Index", required=True)
    assert index_element is not None
    index_text = _text(index_element, f"{field}.Index", required=True)
    index_value = _number(index_text, f"{field}.Index")
    assert index_value is not None
    index_depend_on_slot = _optional_boolean(
        index_element.attrib.get("DependOnSlot"), f"{field}.Index.DependOnSlot"
    ) or False
    index_depend_on_slot_group = _optional_boolean(
        index_element.attrib.get("DependOnSlotGroup"),
        f"{field}.Index.DependOnSlotGroup",
    ) or False
    names = _localized_names(mapping, f"{field}.Name", required=True)
    entries = [
        _parse_pdo_entry(entry, f"{field}.Entry")
        for entry in _children(mapping, "Entry")
    ]
    exclusions: list[dict[str, Any]] = []
    for exclude in _children(mapping, "Exclude"):
        value = _text(exclude, f"{field}.Exclude", required=True)
        excluded_index = _number(value, f"{field}.Exclude")
        assert excluded_index is not None
        exclusions.append({
            "index": _require_range(excluded_index, f"{field}.Exclude", 0xFFFF),
            "depend_on_slot": _optional_boolean(
                exclude.attrib.get("DependOnSlot"), f"{field}.Exclude.DependOnSlot"
            ),
            "depend_on_slot_group": _optional_boolean(
                exclude.attrib.get("DependOnSlotGroup"),
                f"{field}.Exclude.DependOnSlotGroup",
            ),
        })
    attributes = _parse_pdo_type_attributes(mapping, field)
    return {
        "index": _require_range(index_value, f"{field}.Index", 0xFFFF),
        "index_depend_on_slot": index_depend_on_slot,
        "index_depend_on_slot_group": index_depend_on_slot_group,
        "name": _primary_name(names),
        "localized_names": names,
        **attributes,
        "entries": entries,
        "excluded_pdos": exclusions,
        "is_safety_relevant": True,
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
    entries = [
        _parse_pdo_entry(entry, f"{direction}PDO.Entry")
        for entry in _children(pdo, "Entry")
    ]
    attributes = _parse_pdo_type_attributes(pdo, f"{direction}PDO")
    sm_index = attributes["sync_manager"]
    mandatory = attributes["mandatory"] or False
    virtual = attributes["virtual"] or False
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
        excluded_sm_index = _xs_integer(value, f"{direction}PDO.ExcludedSm")
        assert excluded_sm_index is not None
        if not 0 <= excluded_sm_index <= 0xFF:
            raise EsiError(
                f"{direction}PDO.ExcludedSm value {excluded_sm_index} is outside 0..255"
            )
        if excluded_sm_index in excluded_sync_managers:
            raise EsiError(
                f"{direction}PDO 0x{pdo_index:04X} has duplicate ExcludedSm "
                f"reference {excluded_sm_index}"
            )
        excluded_sync_managers.append(excluded_sm_index)

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
        **attributes,
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

    slots = _parse_slots(device, vendor_id)
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
        "module_pdo_group": _optional_nonnegative_xs_integer(
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


def _resolve_slot_dependent_index(
    base_index: int,
    *,
    depend_on_slot: bool,
    depend_on_slot_group: bool,
    slot_index: int,
    slot_group: int | None,
    increments: dict[str, int | None],
    index_kind: str,
    field: str,
) -> int:
    """Apply ETG slot and slot-group increments to one PDO or object index."""
    if depend_on_slot and depend_on_slot_group:
        raise EsiError(
            f"{field} declares both DependOnSlot and DependOnSlotGroup; "
            "combined offset semantics are not implemented"
        )
    if base_index == 0:
        if depend_on_slot or depend_on_slot_group:
            raise EsiError(f"{field} padding index 0 cannot depend on a slot")
        return 0

    if index_kind == "pdo":
        slot_increment_key = "slot_pdo_increment"
        group_increment_key = "slot_group_pdo_increment"
    elif index_kind == "object":
        slot_increment_key = "slot_index_increment"
        group_increment_key = "slot_group_index_increment"
    else:
        raise EsiError(f"{field} has unsupported index kind {index_kind!r}")

    offset = 0
    if depend_on_slot and slot_index:
        increment = increments.get(slot_increment_key)
        if increment is None:
            raise EsiError(
                f"{field} depends on the slot but {slot_increment_key} is not defined"
            )
        offset += slot_index * increment

    if depend_on_slot_group:
        if slot_group is None:
            raise EsiError(
                f"{field} depends on a slot group but the selected slot has no SlotGroup"
            )
        if slot_group:
            increment = increments.get(group_increment_key)
            if increment is None:
                raise EsiError(
                    f"{field} depends on the slot group but "
                    f"{group_increment_key} is not defined"
                )
            offset += slot_group * increment

    resolved = base_index + offset
    if not 0 <= resolved <= 0xFFFF:
        raise EsiError(
            f"{field} resolved index 0x{resolved:X} is outside 0x0000..0xFFFF"
        )
    return resolved


def _resolve_module_pdo_for_slot(
    pdo: dict[str, Any],
    *,
    slot_index: int,
    slot_group: int | None,
    increments: dict[str, int | None],
    slot_descriptor_index: int,
) -> dict[str, Any]:
    direction = pdo["direction"]
    base_index = pdo["index"]
    resolved_index = _resolve_slot_dependent_index(
        base_index,
        depend_on_slot=pdo["index_depend_on_slot"],
        depend_on_slot_group=pdo["index_depend_on_slot_group"],
        slot_index=slot_index,
        slot_group=slot_group,
        increments=increments,
        index_kind="pdo",
        field=f"Module slot {slot_index} {direction.upper()}PDO index",
    )
    allowed_range = (
        (0x1600, 0x17FF) if direction == "rx" else (0x1A00, 0x1BFF)
    )
    if not allowed_range[0] <= resolved_index <= allowed_range[1]:
        raise EsiError(
            f"Module slot {slot_index} resolved {direction.upper()}PDO index "
            f"0x{resolved_index:04X} is outside "
            f"0x{allowed_range[0]:04X}..0x{allowed_range[1]:04X}"
        )

    resolved = {
        **pdo,
        "base_index": base_index,
        "index": resolved_index,
        "index_depend_on_slot": False,
        "index_depend_on_slot_group": False,
        "source_slot_index": slot_index,
        "source_slot_descriptor_index": slot_descriptor_index,
    }

    resolved_entries: list[dict[str, Any]] = []
    for entry_ordinal, entry in enumerate(pdo["entries"]):
        entry_base_index = entry["index"]
        entry_index = _resolve_slot_dependent_index(
            entry_base_index,
            depend_on_slot=entry["index_depend_on_slot"],
            depend_on_slot_group=entry["index_depend_on_slot_group"],
            slot_index=slot_index,
            slot_group=slot_group,
            increments=increments,
            index_kind="object",
            field=(
                f"Module slot {slot_index} {direction.upper()}PDO "
                f"0x{resolved_index:04X} entry[{entry_ordinal}]"
            ),
        )
        resolved_entries.append({
            **entry,
            "base_index": entry_base_index,
            "index": entry_index,
            "index_depend_on_slot": False,
            "index_depend_on_slot_group": False,
        })
    resolved["entries"] = resolved_entries

    resolved_exclusions: list[dict[str, Any]] = []
    seen_exclusions: set[int] = set()
    for exclusion_ordinal, exclusion in enumerate(pdo["excluded_pdos"]):
        exclusion_base_index = exclusion["index"]
        exclusion_index = _resolve_slot_dependent_index(
            exclusion_base_index,
            depend_on_slot=bool(exclusion["depend_on_slot"]),
            depend_on_slot_group=bool(exclusion["depend_on_slot_group"]),
            slot_index=slot_index,
            slot_group=slot_group,
            increments=increments,
            index_kind="pdo",
            field=(
                f"Module slot {slot_index} {direction.upper()}PDO "
                f"0x{resolved_index:04X} Exclude[{exclusion_ordinal}]"
            ),
        )
        if not allowed_range[0] <= exclusion_index <= allowed_range[1]:
            raise EsiError(
                f"Module slot {slot_index} resolved {direction.upper()}PDO "
                f"Exclude index 0x{exclusion_index:04X} is outside the "
                f"allowed PDO range"
            )
        if exclusion_index in seen_exclusions:
            raise EsiError(
                f"Module slot {slot_index} {direction.upper()}PDO "
                f"0x{resolved_index:04X} has duplicate resolved Exclude "
                f"index 0x{exclusion_index:04X}"
            )
        seen_exclusions.add(exclusion_index)
        resolved_exclusions.append({
            **exclusion,
            "base_index": exclusion_base_index,
            "index": exclusion_index,
            "depend_on_slot": False,
            "depend_on_slot_group": False,
        })
    resolved["excluded_pdos"] = resolved_exclusions
    return resolved


def resolve_slot_pdo_configuration(
    document: dict[str, Any],
    device: dict[str, Any],
    configured_slots: list[dict[str, int]],
) -> dict[str, Any]:
    """Resolve module PDO indices for an explicit ordered slot configuration.

    Each configured slot is a mapping with exactly:
      - slot_descriptor_index: zero-based index into DeviceType/Slots/Slot
      - module_ident: selected ModuleIdent (0 denotes an explicitly declared empty slot)

    List order is the configured module position used by DependOnSlot. The
    function resolves module PDO indices, PDO entries and exclusions only.
    It does not parse/merge Module Profile dictionaries, execute InitCmds, infer
    defaults, expand SlotSelections, synthesize ModulePdoGroup alignment PDOs,
    or perform CoE readback. Alignment PDO declarations are returned separately.
    """
    slots = device.get("slots")
    if slots is None:
        raise EsiError("device has no Slots description")
    if not isinstance(configured_slots, list):
        raise EsiError("configured_slots must be an ordered list")

    descriptors = slots["slots"]
    module_by_ident = {
        module["module_ident"]: module for module in document.get("modules", [])
    }
    descriptor_counts = [0] * len(descriptors)
    resolved_slots: list[dict[str, Any]] = []
    flattened_pdos: list[dict[str, Any]] = []
    seen_pdos: dict[tuple[str, int], int] = {}
    active_alignment_groups: dict[int, dict[str, Any]] = {}
    previous_descriptor_index = -1

    device_group = device.get("module_pdo_group")
    if device_group is not None:
        definitions = slots["module_pdo_groups"]
        if device_group >= len(definitions):
            raise EsiError(
                f"Device {device['name']!r} references undefined "
                f"ModulePdoGroup {device_group}"
            )
        definition = definitions[device_group]
        active_alignment_groups[device_group] = {
            "index": device_group,
            "alignment": definition["alignment"],
            "rx_pdo": definition["rx_pdo"],
            "tx_pdo": definition["tx_pdo"],
            "device": True,
            "device_name": device["name"],
            "module_idents": [],
            "slot_indices": [],
        }

    max_slot_count = slots.get("max_slot_count")
    if max_slot_count is not None and len(configured_slots) > max_slot_count:
        raise EsiError(
            f"configured slot count {len(configured_slots)} exceeds "
            f"Slots.MaxSlotCount {max_slot_count}"
        )

    for slot_index, assignment in enumerate(configured_slots):
        if not isinstance(assignment, dict) or set(assignment) != {
            "slot_descriptor_index", "module_ident"
        }:
            raise EsiError(
                f"configured_slots[{slot_index}] must contain exactly "
                "slot_descriptor_index and module_ident"
            )

        descriptor_index = assignment["slot_descriptor_index"]
        module_ident = assignment["module_ident"]
        if (
            isinstance(descriptor_index, bool)
            or not isinstance(descriptor_index, int)
            or not 0 <= descriptor_index < len(descriptors)
        ):
            raise EsiError(
                f"configured_slots[{slot_index}] has invalid "
                "slot_descriptor_index"
            )
        if (
            isinstance(module_ident, bool)
            or not isinstance(module_ident, int)
            or not 0 <= module_ident <= 0xFFFFFFFF
        ):
            raise EsiError(
                f"configured_slots[{slot_index}] has invalid module_ident"
            )
        if descriptor_index < previous_descriptor_index:
            raise EsiError(
                "configured_slots must follow the ESI Slot descriptor order"
            )
        previous_descriptor_index = descriptor_index

        descriptor_counts[descriptor_index] += 1
        descriptor = descriptors[descriptor_index]
        module = None
        if descriptor["module_options"]:
            allowed_idents = {
                option["module_ident"]
                for option in descriptor["module_options"]
            }
            if module_ident not in allowed_idents:
                raise EsiError(
                    f"ModuleIdent 0x{module_ident:08X} is not an allowed choice "
                    f"for Slots.Slot[{descriptor_index}]"
                )
            if module_ident != 0:
                module = module_by_ident.get(module_ident)
                if module is None:
                    raise EsiError(
                        f"configured slot {slot_index} selects ModuleIdent "
                        f"0x{module_ident:08X}, but that module is not loaded; "
                        "attach its explicitly referenced EtherCATModule catalog"
                    )
        else:
            if module_ident == 0:
                raise EsiError(
                    f"Slots.Slot[{descriptor_index}] uses ModuleClass choices; "
                    "ModuleIdent 0 is not an allowed empty-slot choice"
                )
            module = module_by_ident.get(module_ident)
            if module is None:
                raise EsiError(
                    f"configured slot {slot_index} selects unknown ModuleIdent "
                    f"0x{module_ident:08X}"
                )
            vendor_id = document["vendor"]["id"]
            matches = [
                choice for choice in descriptor["module_class_options"]
                if (
                    module.get("module_class") == choice["class"]
                    and vendor_id == choice["vendor_id"]
                )
            ]
            if not matches:
                raise EsiError(
                    f"ModuleIdent 0x{module_ident:08X} (class "
                    f"{module.get('module_class')!r}, vendor 0x{vendor_id:08X}) "
                    f"does not match a ModuleClass choice in "
                    f"Slots.Slot[{descriptor_index}]"
                )

        slot_group = descriptor["slot_group"]
        increments = descriptor["effective_increments"]
        resolved_module_pdos: list[dict[str, Any]] = []
        module_pdo_group = None
        module_name = None
        if module is not None:
            module_pdo_group = module["module_pdo_group"]
            module_name = module["name"]
            for pdo in module["pdos"]:
                resolved_pdo = _resolve_module_pdo_for_slot(
                    pdo,
                    slot_index=slot_index,
                    slot_group=slot_group,
                    increments=increments,
                    slot_descriptor_index=descriptor_index,
                )
                key = (resolved_pdo["direction"], resolved_pdo["index"])
                previous_slot = seen_pdos.get(key)
                if previous_slot is not None:
                    raise EsiError(
                        f"resolved module {resolved_pdo['direction'].upper()}PDO "
                        f"0x{resolved_pdo['index']:04X} collides between "
                        f"configured slots {previous_slot} and {slot_index}"
                    )
                seen_pdos[key] = slot_index
                flattened_pdos.append(resolved_pdo)
                resolved_module_pdos.append(resolved_pdo)

            if module_pdo_group is not None:
                definitions = slots["module_pdo_groups"]
                if module_pdo_group >= len(definitions):
                    raise EsiError(
                        f"ModuleIdent 0x{module_ident:08X} references undefined "
                        f"ModulePdoGroup {module_pdo_group}"
                    )
                definition = definitions[module_pdo_group]
                group_record = active_alignment_groups.setdefault(
                    module_pdo_group,
                    {
                        "index": module_pdo_group,
                        "alignment": definition["alignment"],
                        "rx_pdo": definition["rx_pdo"],
                        "tx_pdo": definition["tx_pdo"],
                        "device": False,
                        "device_name": None,
                        "module_idents": [],
                        "slot_indices": [],
                    },
                )
                if module_ident not in group_record["module_idents"]:
                    group_record["module_idents"].append(module_ident)
                group_record["slot_indices"].append(slot_index)

        resolved_slots.append({
            "slot_index": slot_index,
            "slot_descriptor_index": descriptor_index,
            "slot_group": slot_group,
            "module_ident": module_ident,
            "module_name": module_name,
            "module_pdo_group": module_pdo_group,
            "effective_increments": dict(increments),
            "pdos": resolved_module_pdos,
            "is_empty_slot": module_ident == 0,
        })

    for descriptor_index, descriptor in enumerate(descriptors):
        count = descriptor_counts[descriptor_index]
        if not descriptor["min_instances"] <= count <= descriptor["max_instances"]:
            raise EsiError(
                f"Slots.Slot[{descriptor_index}] has {count} configured instance(s); "
                f"expected {descriptor['min_instances']}..{descriptor['max_instances']}"
            )

    max_group_count = slots.get("max_slot_group_count")
    configured_group_count = len({
        item["slot_group"] for item in resolved_slots
        if item["slot_group"] is not None
    })
    if max_group_count is not None and configured_group_count > max_group_count:
        raise EsiError(
            f"configured SlotGroup count {configured_group_count} exceeds "
            f"Slots.MaxSlotGroupCount {max_group_count}"
        )

    for direction, allowed_range in (
        ("rx", (0x1600, 0x17FF)),
        ("tx", (0x1A00, 0x1BFF)),
    ):
        for pdo in device.get("pdos", []):
            if pdo["direction"] != direction:
                continue
            has_slot_dependent_reference = (
                pdo["index_depend_on_slot"]
                or pdo["index_depend_on_slot_group"]
                or any(
                    bool(entry["index_depend_on_slot"])
                    or bool(entry["index_depend_on_slot_group"])
                    for entry in pdo["entries"]
                )
                or any(
                    bool(exclusion["depend_on_slot"])
                    or bool(exclusion["depend_on_slot_group"])
                    for exclusion in pdo["excluded_pdos"]
                )
            )
            if has_slot_dependent_reference:
                raise EsiError(
                    "device-level PDO contains slot-dependent indices without "
                    "an owning module slot; only module PDOs are resolved"
                )
            if not allowed_range[0] <= pdo["index"] <= allowed_range[1]:
                raise EsiError(
                    f"device-level {direction.upper()}PDO index "
                    f"0x{pdo['index']:04X} is outside its allowed range"
                )

    device_pdo_by_key = {
        (pdo["direction"], pdo["index"]): pdo
        for pdo in device.get("pdos", [])
    }
    module_pdo_keys = {
        (pdo["direction"], pdo["index"]) for pdo in flattened_pdos
    }
    for pdo in flattened_pdos:
        key = (pdo["direction"], pdo["index"])
        device_pdo = device_pdo_by_key.get(key)
        if device_pdo is not None and not device_pdo["overwritten_by_module"]:
            raise EsiError(
                f"resolved module {pdo['direction'].upper()}PDO "
                f"0x{pdo['index']:04X} collides with a device-level PDO that "
                "is not marked OverwrittenByModule"
            )

    for group_index, group in active_alignment_groups.items():
        for direction, alignment_index in (
            ("rx", group["rx_pdo"]),
            ("tx", group["tx_pdo"]),
        ):
            if (direction, alignment_index) in seen_pdos:
                raise EsiError(
                    f"ModulePdoGroup {group_index} alignment {direction.upper()}PDO "
                    f"0x{alignment_index:04X} collides with a resolved module PDO"
                )

    device_pdos = device.get("pdos", [])
    overridden_device_pdos = [
        pdo for pdo in device_pdos
        if pdo["overwritten_by_module"]
        and (pdo["direction"], pdo["index"]) in module_pdo_keys
    ]
    effective_device_pdos = [
        pdo for pdo in device_pdos
        if pdo not in overridden_device_pdos
    ]
    unresolved_safety_mappings = [
        {
            "slot_index": slot["slot_index"],
            "module_ident": slot["module_ident"],
            "mappings": module_by_ident[slot["module_ident"]]["safety_parameter_mappings"],
        }
        for slot in resolved_slots
        if slot["module_ident"] != 0
        and module_by_ident[slot["module_ident"]]["safety_parameter_mappings"]
    ]

    return {
        "configured_slots": resolved_slots,
        "module_pdos": flattened_pdos,
        "module_pdo_alignment_groups": [
            active_alignment_groups[index]
            for index in sorted(active_alignment_groups)
        ],
        "device_pdos": device_pdos,
        "effective_device_pdos": effective_device_pdos,
        "overridden_device_pdos": overridden_device_pdos,
        "unresolved_safety_parameter_mappings": unresolved_safety_mappings,
        "runtime_readback_performed": False,
        "is_complete_process_data_mapping": False,
    }


def resolve_cia402_candidate(
    device: dict[str, Any], rx_pdo_indices: list[int], tx_pdo_indices: list[int]
) -> dict[str, Any]:
    if device.get("slots") is not None:
        raise EsiError(
            "slot-configurable device requires an explicit slot-resolved PDO map; "
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

    for pdo in selected:
        if pdo["overwritten_by_module"]:
            raise EsiError(
                f"selected {pdo['direction'].upper()}PDO 0x{pdo['index']:04X} "
                "is marked OverwrittenByModule and is not an effective device PDO"
            )
        if (
            pdo["safety_pdo_type"] is not None
            or pdo["sra_parameter"]
            or pdo["safety_connection_number"] is not None
        ):
            raise EsiError(
                f"selected {pdo['direction'].upper()}PDO 0x{pdo['index']:04X} "
                "contains FSoE/SRA safety metadata and cannot be treated as a "
                "generic CiA 402 process-data mapping"
            )
        if any(
            pdo[name] is not None
            for name in (
                "oversampling_factor",
                "oversampling_min",
                "oversampling_max",
                "oversampling_index_increment",
            )
        ):
            raise EsiError(
                f"selected {pdo['direction'].upper()}PDO 0x{pdo['index']:04X} "
                "contains oversampling metadata that the static CiA 402 "
                "candidate resolver does not expand"
            )
        if any(
            entry["safety_connection_number"] is not None
            for entry in pdo["entries"]
        ):
            raise EsiError(
                f"selected {pdo['direction'].upper()}PDO 0x{pdo['index']:04X} "
                "contains safety-connection entries and cannot be treated as a "
                "generic CiA 402 process-data mapping"
            )

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
                "sync_unit": pdo["sync_unit"],
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




def render_pdo_snapshot(candidate: dict[str, Any]) -> dict[str, Any]:
    """Render an explicitly selected, non-slot ESI candidate as a layered snapshot.

    This is an ESI expectation only. It does not imply that the selected
    assignments are active on a slave or have been read back through CoE/IgH.
    """
    identity = candidate["identity"]
    assignments: list[dict[str, int]] = []
    mappings: list[dict[str, Any]] = []
    for direction, assignment_object, key in (
        ("rx", 0x1C12, "selected_rx_pdo_indices"),
        ("tx", 0x1C13, "selected_tx_pdo_indices"),
    ):
        pdo_indices = candidate[key]
        for assignment_position, pdo_index in enumerate(pdo_indices, start=1):
            assignments.append({
                "assignment_object": assignment_object,
                "assignment_position": assignment_position,
                "pdo_index": pdo_index,
            })
            entries = [
                entry for entry in candidate["ordered_entries"]
                if entry["direction"] == direction and entry["pdo_index"] == pdo_index
            ]
            if not entries:
                raise EsiError(
                    f"selected {direction.upper()}PDO 0x{pdo_index:04X} "
                    "has no resolved mapping entries"
                )
            for entry_position, entry in enumerate(entries, start=1):
                index = entry["index"]
                subindex = entry["sub_index"]
                width = entry["bit_length"]
                if not 0 <= index <= 0xFFFF or not 0 <= subindex <= 0xFF:
                    raise EsiError("PDO mapping object index/subindex is out of range")
                if not 1 <= width <= 0xFF:
                    raise EsiError("PDO mapping bit length is outside 1..255")
                raw = (index << 16) | (subindex << 8) | width
                mappings.append({
                    "assignment_object": assignment_object,
                    "assignment_position": assignment_position,
                    "pdo_index": pdo_index,
                    "sync_manager": entry["sync_manager"],
                    "direction": direction,
                    "entry_position": entry_position,
                    "raw_mapping_entry": raw,
                })
    return {
        "schema_version": 1,
        "source_kind": "esi_expected",
        "identity": {
            "vendor_id": identity["vendor_id"],
            "product_code": identity["product_code"],
        },
        "assignments": assignments,
        "mappings": mappings,
        "runtime_readback_performed": False,
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

def _read_xml_document(path: Path, expected_root: str) -> tuple[bytes, ET.Element]:
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
    actual_root = _local_name(root.tag)
    if actual_root != expected_root:
        raise EsiError(f"root element must be <{expected_root}>")
    return raw, root


def parse_module_esi(path: Path) -> dict[str, Any]:
    """Parse one explicitly supplied EtherCATModule XML document."""
    raw, root = _read_xml_document(path, "EtherCATModule")
    vendor = _one(root, "Vendor", required=True)
    assert vendor is not None
    vendor_id = _require_range(
        _required_child_number(vendor, "Id", "Vendor.Id"),
        "Vendor.Id",
        0xFFFFFFFF,
    )
    vendor_names = _localized_names(vendor, "Vendor.Name", required=True)
    modules_element = _one(root, "Modules", required=True)
    assert modules_element is not None
    modules = [
        _parse_module(node)
        for node in _children(modules_element, "Module")
    ]
    seen_module_idents: set[int] = set()
    for module in modules:
        ident = module["module_ident"]
        if ident in seen_module_idents:
            raise EsiError(
                f"duplicate ModuleIdent 0x{ident:08X} in EtherCATModule catalog"
            )
        seen_module_idents.add(ident)
    return {
        "source": {
            "file_name": path.name,
            "sha256": hashlib.sha256(raw).hexdigest(),
        },
        "vendor": {
            "id": vendor_id,
            "name": _primary_name(vendor_names),
            "localized_names": vendor_names,
        },
        "modules": modules,
    }


def _validate_slot_module_references(document: dict[str, Any]) -> None:
    """Validate ModuleIdent references only when the module catalog is complete."""
    if not document.get("module_catalog_complete", False):
        return
    available = {
        module["module_ident"] for module in document.get("modules", [])
    }
    for device in document.get("devices", []):
        slots = device.get("slots")
        if slots is None:
            continue
        for slot in slots["slots"]:
            for option in slot["module_options"]:
                ident = option["module_ident"]
                if ident == 0:
                    continue
                if ident not in available:
                    raise EsiError(
                        f"Slots.Slot[{slot['ordinal']}] references unknown "
                        f"ModuleIdent 0x{ident:08X}"
                    )
        for selection_index, selection in enumerate(slots["slot_selections"]):
            for ident in selection["module_idents"]:
                if ident == 0:
                    continue
                if ident not in available:
                    raise EsiError(
                        f"Slots.SlotSelections[{selection_index}] references "
                        f"unknown ModuleIdent 0x{ident:08X}"
                    )


def _validate_module_pdo_group_references(document: dict[str, Any]) -> None:
    """Validate module PDO-group references against the ordered Slots descriptors."""
    modules = document.get("modules", [])
    for device in document.get("devices", []):
        slots = device.get("slots")
        device_group = device.get("module_pdo_group")
        if slots is None:
            if device_group is not None:
                raise EsiError(
                    f"Device {device['name']!r} uses ModulePdoGroup {device_group} "
                    "without a Slots/ModulePdoGroup description"
                )
            continue

        group_descriptions = slots["module_pdo_groups"]
        referenced_ids = {
            option["module_ident"]
            for slot in slots["slots"]
            for option in slot["module_options"]
            if option["module_ident"] != 0
        }
        referenced_ids.update(
            ident
            for selection in slots["slot_selections"]
            for ident in selection["module_idents"]
            if ident != 0
        )
        referenced_classes = {
            (option["class"], option["vendor_id"])
            for slot in slots["slots"]
            for option in slot["module_class_options"]
        }

        used_groups: dict[int, list[str]] = {}
        if device_group is not None:
            used_groups.setdefault(device_group, []).append(
                f"Device {device['name']!r}"
            )
        for module in modules:
            ident = module["module_ident"]
            module_class = module.get("module_class")
            is_referenced = ident in referenced_ids or (
                module_class is not None
                and (module_class, document["vendor"]["id"]) in referenced_classes
            )
            if not is_referenced:
                continue
            group = module.get("module_pdo_group")
            if group is not None:
                used_groups.setdefault(group, []).append(
                    f"ModuleIdent 0x{ident:08X}"
                )

        for group_index, owners in sorted(used_groups.items()):
            if group_index >= len(group_descriptions):
                raise EsiError(
                    f"{owners[0]} references ModulePdoGroup {group_index}, but "
                    f"Slots declares only {len(group_descriptions)} group(s)"
                )
            description = group_descriptions[group_index]
            missing = [
                field for field in ("alignment", "rx_pdo", "tx_pdo")
                if description[field] is None
            ]
            if missing:
                raise EsiError(
                    f"Slots.ModulePdoGroup[{group_index}] is used by "
                    f"{', '.join(owners)} but is missing required "
                    f"{', '.join(missing)}"
                )


def attach_external_module_files(
    document: dict[str, Any],
    module_paths: list[Path],
) -> None:
    """Merge only explicitly provided files named by the ESI InfoReference list.

    InfoReference values are never opened automatically. Callers must explicitly
    provide the module files, preventing path traversal or implicit file access.
    """
    if not module_paths:
        return

    def reference_name(value: str) -> str:
        return value.replace("\\", "/").rsplit("/", 1)[-1]

    referenced_names = {reference_name(value) for value in document["info_references"]}
    sources: list[dict[str, Any]] = []
    all_modules = list(document["modules"])
    existing_idents = {item["module_ident"] for item in all_modules}

    for path in module_paths:
        if path.name not in referenced_names:
            raise EsiError(
                f"external module file {path.name!r} is not listed in InfoReference"
            )
        external = parse_module_esi(path)
        if external["vendor"]["id"] != document["vendor"]["id"]:
            raise EsiError(
                f"external module file {path.name!r} Vendor ID "
                f"0x{external['vendor']['id']:08X} does not match ESI Vendor ID "
                f"0x{document['vendor']['id']:08X}"
            )
        for module in external["modules"]:
            ident = module["module_ident"]
            if ident in existing_idents:
                raise EsiError(
                    f"duplicate ModuleIdent 0x{ident:08X} across embedded and "
                    "external module catalogs"
                )
            existing_idents.add(ident)
            all_modules.append(module)
        sources.append(external["source"])

    document["modules"] = all_modules
    document["external_module_sources"] = sources
    provided_names = {path.name for path in module_paths}
    document["module_catalog_complete"] = referenced_names.issubset(provided_names)
    _validate_slot_module_references(document)
    _validate_module_pdo_group_references(document)


def parse_esi(path: Path) -> dict[str, Any]:
    raw, root = _read_xml_document(path, "EtherCATInfo")

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
    document = {
        "schema_version": 1,
        "source": {"file_name": path.name, "sha256": hashlib.sha256(raw).hexdigest()},
        "vendor": {
            "id": vendor_id,
            "name": vendor_name,
            "localized_names": vendor_names,
        },
        "info_references": info_references,
        "modules": module_catalog,
        "module_catalog_complete": modules_element is not None and not info_references,
        "devices": devices,
    }
    _validate_slot_module_references(document)
    _validate_module_pdo_group_references(document)
    return document


def _slot_config_integer(value: Any, field: str) -> int:
    if isinstance(value, bool):
        raise EsiError(f"{field} must be an integer, not a boolean")
    if isinstance(value, int):
        if value < 0:
            raise EsiError(f"{field} must be non-negative")
        return value
    if isinstance(value, str):
        parsed = _number(value, field)
        assert parsed is not None
        if parsed < 0:
            raise EsiError(f"{field} must be non-negative")
        return parsed
    raise EsiError(f"{field} must be a decimal, 0x, or #x integer")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("esi", type=Path, help="input EtherCAT ESI XML file")
    parser.add_argument("-o", "--output", type=Path, help="write JSON to this path (default: stdout)")
    parser.add_argument("--product-code", help="include only devices matching this decimal, 0x..., or #x... product code")
    parser.add_argument("--rx-pdo", action="append", default=[], help="select an RxPDO index (repeatable; decimal, 0x..., or #x...) for CiA 402 candidate resolution")
    parser.add_argument("--tx-pdo", action="append", default=[], help="select a TxPDO index (repeatable; decimal, 0x..., or #x...) for CiA 402 candidate resolution")
    parser.add_argument("--module-esi", type=Path, action="append", default=[], help="explicitly load an EtherCATModule XML file listed by InfoReference (repeatable); references are never followed automatically")
    parser.add_argument("--slot-config", type=Path, help="JSON array of explicit slot assignments; each item has slot_descriptor_index and module_ident (decimal, 0x, or #x)")
    parser.add_argument("--emit-cpp-fixture", action="store_true", help="emit a C++ golden fixture instead of normalized JSON; requires explicit PDO selections")
    parser.add_argument("--emit-pdo-snapshot", action="store_true", help="emit layered ESI expected assignment/mapping snapshot; requires explicit PDO selections")
    args = parser.parse_args(argv)
    try:
        document = parse_esi(args.esi)
        if args.module_esi:
            attach_external_module_files(document, args.module_esi)
        if args.product_code:
            wanted = _number(args.product_code, "--product-code")
            document["devices"] = [
                item for item in document["devices"]
                if item["identity"]["product_code"] == wanted
            ]
            if not document["devices"]:
                raise EsiError(f"no device has product code 0x{wanted:08X}")
        if args.slot_config is not None:
            if args.rx_pdo or args.tx_pdo or args.emit_cpp_fixture:
                raise EsiError(
                    "--slot-config cannot be combined with --rx-pdo, --tx-pdo, "
                    "or --emit-cpp-fixture; slot-resolved CiA 402 binding is a "
                    "separate validation stage"
                )
            if len(document["devices"]) != 1:
                raise EsiError(
                    "--slot-config requires exactly one selected device; use --product-code"
                )
            try:
                configured_slots = json.loads(
                    args.slot_config.read_text(encoding="utf-8")
                )
            except json.JSONDecodeError as exc:
                raise EsiError(f"invalid --slot-config JSON: {exc}") from exc
            if not isinstance(configured_slots, list):
                raise EsiError("--slot-config JSON root must be an array")
            normalized_slots = []
            for index, assignment in enumerate(configured_slots):
                if not isinstance(assignment, dict) or set(assignment) != {
                    "slot_descriptor_index", "module_ident"
                }:
                    raise EsiError(
                        f"--slot-config item[{index}] must contain exactly "
                        "slot_descriptor_index and module_ident"
                    )
                normalized_slots.append({
                    "slot_descriptor_index": _slot_config_integer(
                        assignment["slot_descriptor_index"],
                        f"--slot-config item[{index}].slot_descriptor_index",
                    ),
                    "module_ident": _slot_config_integer(
                        assignment["module_ident"],
                        f"--slot-config item[{index}].module_ident",
                    ),
                })
            document["slot_pdo_configuration"] = resolve_slot_pdo_configuration(
                document, document["devices"][0], normalized_slots
            )
        if args.rx_pdo or args.tx_pdo:
            if len(document["devices"]) != 1:
                raise EsiError("CiA 402 candidate resolution requires exactly one selected device; use --product-code")
            rx_indices = [_number(value, "--rx-pdo") for value in args.rx_pdo]
            tx_indices = [_number(value, "--tx-pdo") for value in args.tx_pdo]
            document["cia402_candidate"] = resolve_cia402_candidate(
                document["devices"][0], rx_indices, tx_indices
            )
        if args.emit_cpp_fixture and args.emit_pdo_snapshot:
            raise EsiError("--emit-cpp-fixture and --emit-pdo-snapshot are mutually exclusive")
        if args.emit_cpp_fixture:
            if "cia402_candidate" not in document:
                raise EsiError("--emit-cpp-fixture requires explicit --rx-pdo and --tx-pdo selections")
            rendered = render_cia402_cpp_fixture(document["cia402_candidate"])
        elif args.emit_pdo_snapshot:
            if "cia402_candidate" not in document:
                raise EsiError("--emit-pdo-snapshot requires explicit --rx-pdo and --tx-pdo selections")
            rendered = json.dumps(render_pdo_snapshot(document["cia402_candidate"]), indent=2, sort_keys=True) + "\n"
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
