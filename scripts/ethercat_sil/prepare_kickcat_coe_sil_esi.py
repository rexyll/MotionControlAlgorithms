#!/usr/bin/env python3
"""Create a KickCAT/IgH CoE-only SIL ESI without unsupported mailbox services."""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


class EsiPreparationError(ValueError):
    pass


def _pdo_signature(xml_text: str) -> list[bytes]:
    root = ET.fromstring(xml_text)
    return [
        ET.tostring(node, encoding="utf-8")
        for node in root.iter()
        if node.tag.rsplit("}", 1)[-1] in {"RxPdo", "TxPdo", "Sm"}
    ]


def prepare_esi_text(xml_text: str) -> str:
    """Disable only EoE and CoE SDO Information; preserve CoE PDO semantics."""
    try:
        ET.fromstring(xml_text)
    except ET.ParseError as exc:
        raise EsiPreparationError(f"input ESI is not valid XML: {exc}") from exc

    eoe_matches = list(re.finditer(r"<EoE\s*/>", xml_text))
    if len(eoe_matches) != 1:
        raise EsiPreparationError(
            f"expected exactly one empty EoE capability element, found {len(eoe_matches)}"
        )
    result = xml_text[:eoe_matches[0].start()] + xml_text[eoe_matches[0].end():]

    coe_matches = list(re.finditer(r"<CoE\b[^>]*>", result))
    if len(coe_matches) != 1:
        raise EsiPreparationError(
            f"expected exactly one CoE capability element, found {len(coe_matches)}"
        )
    coe_tag = coe_matches[0].group(0)
    sdo_info_matches = list(re.finditer(r'\bSdoInfo="(?:true|false)"', coe_tag))
    if len(sdo_info_matches) != 1:
        raise EsiPreparationError(
            "CoE capability must declare exactly one boolean SdoInfo attribute"
        )
    updated_coe_tag = re.sub(r'\bSdoInfo="(?:true|false)"',
                             'SdoInfo="false"', coe_tag, count=1)
    result = result[:coe_matches[0].start()] + updated_coe_tag + result[coe_matches[0].end():]

    try:
        ET.fromstring(result)
    except ET.ParseError as exc:
        raise EsiPreparationError(f"transformed ESI is not valid XML: {exc}") from exc
    if _pdo_signature(xml_text) != _pdo_signature(result):
        raise EsiPreparationError("PDO/SyncManager declarations changed during ESI transformation")
    if re.search(r"<EoE\s*/>", result):
        raise EsiPreparationError("EoE capability remains enabled")
    if not re.search(r'<CoE\b[^>]*\bSdoInfo="false"', result):
        raise EsiPreparationError("CoE SdoInfo capability was not disabled")
    return result


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: prepare_kickcat_coe_sil_esi.py INPUT.xml OUTPUT.xml", file=sys.stderr)
        return 2
    source, destination = map(Path, sys.argv[1:])
    try:
        transformed = prepare_esi_text(source.read_text(encoding="utf-8"))
        destination.write_text(transformed, encoding="utf-8")
    except (OSError, EsiPreparationError) as exc:
        print(f"CoE SIL ESI preparation error: {exc}", file=sys.stderr)
        return 1
    print("PASS: CoE-only SIL ESI; EoE and SDO Information disabled; PDO/SM declarations unchanged")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
