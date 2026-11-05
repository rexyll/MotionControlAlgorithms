import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "ethercat_esi" / "esi_to_profile.py"
sys.path.insert(0, str(SCRIPT.parent))
import esi_to_profile  # noqa: E402


ESI = """<?xml version="1.0"?>
<EtherCATInfo xmlns="https://www.ethercat.org/ESI">
  <Vendor><Id>#x000000AB</Id><Name>Example Vendor</Name></Vendor>
  <Descriptions>
    <Devices>
      <Device>
        <Type ProductCode="#x00001001" RevisionNo="#x00000002">Servo-A</Type>
        <Name>Servo A</Name>
        <Sm StartAddress="#x1000" ControlByte="#x26" Enable="1">MBoxOut</Sm>
        <Sm StartAddress="#x1080" ControlByte="#x22" Enable="1">MBoxIn</Sm>
        <Sm StartAddress="#x1100" ControlByte="#x64" Enable="1">Outputs</Sm>
        <Sm StartAddress="#x1180" ControlByte="#x20" Enable="1">Inputs</Sm>
        <RxPdo Sm="2" Fixed="true" Mandatory="true">
          <Index>#x1600</Index><Name>RxPDO</Name>
          <Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Controlword</Name><DataType>UINT</DataType></Entry>
          <Entry><Index>#x6060</Index><SubIndex>0</SubIndex><BitLen>8</BitLen><Name>Mode</Name><DataType>SINT</DataType></Entry>
          <Entry><Index>#x0000</Index><SubIndex>0</SubIndex><BitLen>8</BitLen><Name>Gap</Name></Entry>
        </RxPdo>
        <TxPdo Sm="3">
          <Index>#x1A00</Index><Name>TxPDO</Name>
          <Entry><Index>#x6041</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Statusword</Name><DataType>UINT</DataType></Entry>
          <Entry><Index>#x6061</Index><SubIndex>0</SubIndex><BitLen>8</BitLen><Name>Mode display</Name><DataType>SINT</DataType></Entry>
        </TxPdo>
      </Device>
      <Device>
        <Type ProductCode="4098" RevisionNo="3">Servo-B</Type>
        <Name>Servo B</Name>
        <RxPdo Sm="2"><Index>5632</Index><Name>Rx</Name>
          <Entry><Index>24640</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>CW</Name></Entry>
        </RxPdo>
      </Device>
    </Devices>
  </Descriptions>
</EtherCATInfo>
"""


class EsiToProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "fixture.xml"
        self.path.write_text(ESI, encoding="utf-8")

    def test_normalizes_vendor_multiple_devices_and_pdo_directions(self):
        result = esi_to_profile.parse_esi(self.path)
        self.assertEqual(result["vendor"], {"id": 0xAB, "name": "Example Vendor"})
        self.assertEqual(len(result["devices"]), 2)
        first = result["devices"][0]
        self.assertEqual(first["identity"], {
            "vendor_id": 0xAB, "product_code": 0x1001, "revision": 2
        })
        self.assertEqual(first["sync_managers"][2]["name"], "Outputs")
        self.assertEqual(first["pdos"][0]["direction"], "rx")
        self.assertEqual(first["pdos"][0]["sync_manager"], 2)
        self.assertTrue(first["pdos"][0]["fixed"])
        self.assertEqual(first["pdos"][0]["entries"][0]["index"], 0x6040)
        self.assertTrue(first["pdos"][0]["entries"][2]["is_padding"])
        self.assertEqual(first["pdos"][1]["direction"], "tx")
        self.assertEqual(first["pdos"][1]["entries"][0]["bit_length"], 16)

    def test_decimal_and_hex_identity_fields_are_normalized(self):
        second = esi_to_profile.parse_esi(self.path)["devices"][1]
        self.assertEqual(second["identity"]["product_code"], 4098)
        self.assertEqual(second["identity"]["revision"], 3)

    def test_rejects_wrong_root(self):
        self.path.write_text("<NotEsi/>", encoding="utf-8")
        with self.assertRaisesRegex(esi_to_profile.EsiError, "root element"):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_dtd_and_entity_declarations(self):
        self.path.write_text('<!DOCTYPE x [<!ENTITY e "boom">]><EtherCATInfo/>', encoding="utf-8")
        with self.assertRaisesRegex(esi_to_profile.EsiError, "DOCTYPE"):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_missing_device_revision(self):
        self.path.write_text(ESI.replace(' RevisionNo="#x00000002"', ""), encoding="utf-8")
        with self.assertRaisesRegex(esi_to_profile.EsiError, "RevisionNo"):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_invalid_pdo_entry_bit_length(self):
        self.path.write_text(ESI.replace("<BitLen>16</BitLen>", "<BitLen>128</BitLen>", 1), encoding="utf-8")
        with self.assertRaisesRegex(esi_to_profile.EsiError, "unsupported bit length"):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_empty_pdo(self):
        malformed = ESI.replace(
            '<Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Controlword</Name><DataType>UINT</DataType></Entry>\n          <Entry><Index>#x6060</Index><SubIndex>0</SubIndex><BitLen>8</BitLen><Name>Mode</Name><DataType>SINT</DataType></Entry>\n          <Entry><Index>#x0000</Index><SubIndex>0</SubIndex><BitLen>8</BitLen><Name>Gap</Name></Entry>',
            ""
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(esi_to_profile.EsiError, "contains no entries"):
            esi_to_profile.parse_esi(self.path)


if __name__ == "__main__":
    unittest.main()
