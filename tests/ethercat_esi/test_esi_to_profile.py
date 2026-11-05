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


ESI_WITH_VELOCITY = ESI.replace(
    "      </Device>",
    """        <RxPdo Sm="2"><Index>#x1601</Index><Name>Velocity command</Name>
          <Entry><Index>#x60FF</Index><SubIndex>0</SubIndex><BitLen>32</BitLen><Name>Target velocity</Name><DataType>DINT</DataType></Entry>
        </RxPdo>
        <TxPdo Sm="3"><Index>#x1A01</Index><Name>Velocity feedback</Name>
          <Entry><Index>#x606C</Index><SubIndex>0</SubIndex><BitLen>32</BitLen><Name>Actual velocity</Name><DataType>DINT</DataType></Entry>
        </TxPdo>
      </Device>""",
    1,
)


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


    def test_resolves_cia402_candidate_from_explicit_pdo_selection(self):
        self.path.write_text(ESI_WITH_VELOCITY, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        result = esi_to_profile.resolve_cia402_candidate(
            device, [0x1600, 0x1601], [0x1A00, 0x1A01]
        )
        self.assertEqual(result["profile"], "CiA402")
        self.assertEqual(result["bindings"]["controlword"]["index"], 0x6040)
        self.assertEqual(result["bindings"]["target_velocity"]["pdo_index"], 0x1601)
        self.assertEqual(result["bindings"]["target_velocity"]["sync_manager"], 2)
        self.assertEqual(result["bindings"]["actual_velocity"]["direction"], "tx")
        self.assertEqual(result["bindings"]["target_velocity"]["bit_offset"], 32)
        self.assertTrue(result["supports"]["cyclic_synchronous_velocity"])
        self.assertFalse(result["supports"]["cyclic_synchronous_position"])
        self.assertFalse(result["supports"]["homing"])
        self.assertFalse(result["is_runtime_readback"])

    def test_requires_explicit_selection_and_all_core_objects(self):
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(esi_to_profile.EsiError, "both required"):
            esi_to_profile.resolve_cia402_candidate(device, [0x1600], [])
        with self.assertRaisesRegex(esi_to_profile.EsiError, "missing required"):
            esi_to_profile.resolve_cia402_candidate(device, [], [0x1A00])

    def test_rejects_wrong_direction_or_width_for_cia402_objects(self):
        wrong_width = ESI_WITH_VELOCITY.replace(
            "<BitLen>32</BitLen><Name>Target velocity</Name>",
            "<BitLen>16</BitLen><Name>Target velocity</Name>",
        )
        self.path.write_text(wrong_width, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(esi_to_profile.EsiError, "must be RXPDO/32-bit"):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_rejects_duplicate_object_across_selected_pdos(self):
        duplicate = ESI_WITH_VELOCITY.replace(
            '<Entry><Index>#x60FF</Index><SubIndex>0</SubIndex><BitLen>32</BitLen><Name>Target velocity</Name><DataType>DINT</DataType></Entry>',
            '<Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Duplicate controlword</Name><DataType>UINT</DataType></Entry>',
        )
        self.path.write_text(duplicate, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(esi_to_profile.EsiError, "duplicate selected object"):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

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
