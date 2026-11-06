import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "ethercat_sil"))
from prepare_kickcat_coe_sil_esi import EsiPreparationError, prepare_esi_text


ESI = """<?xml version="1.0"?>
<EtherCATInfo>
  <Device>
    <Sm>2</Sm>
    <RxPdo><Index>0x1600</Index><Entry><Index>0x6040</Index></Entry></RxPdo>
    <TxPdo><Index>0x1A00</Index><Entry><Index>0x6041</Index></Entry></TxPdo>
    <Mailbox DataLinkLayer="true">
      <EoE />
      <CoE SdoInfo="true" CompleteAccess="true" PdoAssign="true" PdoConfig="true" />
      <FoE />
    </Mailbox>
  </Device>
</EtherCATInfo>
"""


class PrepareKickcatCoeSilEsiTests(unittest.TestCase):
    def test_disables_only_unsupported_mailbox_services(self):
        result = prepare_esi_text(ESI)
        self.assertNotIn("<EoE", result)
        self.assertIn('<CoE SdoInfo="false"', result)
        self.assertIn('CompleteAccess="true"', result)
        self.assertIn('PdoAssign="true"', result)
        self.assertIn('PdoConfig="true"', result)
        self.assertIn("<FoE />", result)

    def test_preserves_pdo_and_sync_manager_declarations(self):
        result = prepare_esi_text(ESI)
        for fragment in ("<Sm>2</Sm>", "<RxPdo>", "<Index>0x1600</Index>",
                         "<TxPdo>", "<Index>0x1A00</Index>"):
            self.assertIn(fragment, result)

    def test_rejects_missing_eoe_capability(self):
        with self.assertRaisesRegex(EsiPreparationError, "EoE capability"):
            prepare_esi_text(ESI.replace("      <EoE />\n", ""))

    def test_rejects_missing_sdo_info_attribute(self):
        invalid = ESI.replace('SdoInfo="true" ', "")
        with self.assertRaisesRegex(EsiPreparationError, "SdoInfo attribute"):
            prepare_esi_text(invalid)

    def test_rejects_invalid_xml(self):
        with self.assertRaisesRegex(EsiPreparationError, "valid XML"):
            prepare_esi_text("<EtherCATInfo>")


if __name__ == "__main__":
    unittest.main()
