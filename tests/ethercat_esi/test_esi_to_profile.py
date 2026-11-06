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


ESI_WITH_ALL_MODES = ESI_WITH_VELOCITY.replace(
    "      </Device>",
    """        <RxPdo Sm="2"><Index>#x1602</Index><Name>Position and torque commands</Name>
          <Entry><Index>#x607A</Index><SubIndex>0</SubIndex><BitLen>32</BitLen><Name>Target position</Name><DataType>DINT</DataType></Entry>
          <Entry><Index>#x6071</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Target torque</Name><DataType>INT</DataType></Entry>
        </RxPdo>
        <TxPdo Sm="3"><Index>#x1A02</Index><Name>Position and torque feedback</Name>
          <Entry><Index>#x6064</Index><SubIndex>0</SubIndex><BitLen>32</BitLen><Name>Actual position</Name><DataType>DINT</DataType></Entry>
          <Entry><Index>#x6077</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Actual torque</Name><DataType>INT</DataType></Entry>
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

    def test_rejects_duplicate_pdo_index_declarations(self):
        malformed = ESI.replace(
            "      </Device>",
            """        <RxPdo Sm="2"><Index>#x1600</Index><Name>Duplicate RxPDO</Name>
          <Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Controlword</Name></Entry>
        </RxPdo>
      </Device>""",
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "duplicate PDO index 0x1600 declared by RXPDO and RXPDO",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_parses_xml_schema_boolean_lexical_forms_and_sm_metadata(self):
        valid = ESI.replace(
            'Fixed="true" Mandatory="true"',
            'Fixed="1" Mandatory="0"',
            1,
        ).replace(
            'ControlByte="#x64" Enable="1">Outputs',
            'ControlByte="#x64" Enable="1" Virtual="1" OpOnly="true">Outputs',
            1,
        )
        self.path.write_text(valid, encoding="utf-8")
        first = esi_to_profile.parse_esi(self.path)["devices"][0]
        self.assertTrue(first["pdos"][0]["fixed"])
        self.assertFalse(first["pdos"][0]["mandatory"])
        self.assertTrue(first["sync_managers"][2]["virtual"])
        self.assertTrue(first["sync_managers"][2]["op_only"])

    def test_rejects_invalid_xml_schema_boolean_value(self):
        malformed = ESI.replace('Fixed="true"', 'Fixed="yes"', 1)
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "invalid boolean value for rxPDO.Fixed",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_parses_pdo_exclusion_and_virtual_metadata(self):
        valid = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">
          <ExcludedSm>3</ExcludedSm>',
            1,
        ).replace(
            "      </Device>",
            """        <RxPdo Sm="2" Virtual="1">
          <Index>#x1603</Index><Name>Virtual register mapping</Name>
        </RxPdo>
      </Device>""",
            1,
        )
        self.path.write_text(valid, encoding="utf-8")
        first = esi_to_profile.parse_esi(self.path)["devices"][0]
        self.assertEqual(first["pdos"][0]["excluded_sync_managers"], [3])
        self.assertTrue(first["pdos"][-1]["virtual"])
        self.assertEqual(first["pdos"][-1]["entries"], [])

    def test_rejects_selected_mutually_exclusive_pdos(self):
        incompatible = ESI_WITH_VELOCITY.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">
          <Exclude>#x1601</Exclude>',
            1,
        )
        self.path.write_text(incompatible, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "selected PDO 0x1600 excludes selected PDO 0x1601",
        ):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_rejects_slot_dependent_exclusions_in_static_candidate(self):
        incompatible = ESI_WITH_VELOCITY.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">
          <Exclude DependOnSlot="true">#x1601</Exclude>',
            1,
        )
        self.path.write_text(incompatible, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "slot-dependent Exclude reference",
        ):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_rejects_default_sync_manager_that_is_explicitly_excluded(self):
        malformed = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">
          <ExcludedSm>2</ExcludedSm>',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "declares SyncManager 2 as both its default and excluded assignment",
        ):
            esi_to_profile.parse_esi(self.path)

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


    def test_cpp_fixture_is_generated_from_the_same_normalized_candidate(self):
        self.path.write_text(ESI_WITH_VELOCITY, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        candidate = esi_to_profile.resolve_cia402_candidate(
            device, [0x1600, 0x1601], [0x1A00, 0x1A01]
        )
        generated = esi_to_profile.render_cia402_cpp_fixture(candidate)
        fixture_path = (
            Path(__file__).resolve().parents[1]
            / "fixtures" / "ethercat_esi" / "cia402_bindings_fixture.hpp"
        )
        self.assertEqual(generated, fixture_path.read_text(encoding="utf-8"))
        self.assertEqual(len(candidate["ordered_entries"]), 7)
        self.assertTrue(candidate["ordered_entries"][2]["is_padding"])
        self.assertEqual(candidate["ordered_entries"][3]["bit_offset"], 32)
        self.assertEqual(candidate["ordered_entries"][6]["bit_offset"], 24)


    def test_cpp_all_modes_fixture_matches_python_candidate(self):
        self.path.write_text(ESI_WITH_ALL_MODES, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        candidate = esi_to_profile.resolve_cia402_candidate(
            device, [0x1600, 0x1601, 0x1602], [0x1A00, 0x1A01, 0x1A02]
        )
        self.assertTrue(candidate["supports"]["cyclic_synchronous_position"])
        self.assertTrue(candidate["supports"]["cyclic_synchronous_velocity"])
        self.assertTrue(candidate["supports"]["cyclic_synchronous_torque"])
        self.assertEqual(candidate["bindings"]["target_position"]["bit_offset"], 64)
        self.assertEqual(candidate["bindings"]["target_torque"]["bit_offset"], 96)
        self.assertEqual(candidate["bindings"]["actual_position"]["bit_offset"], 56)
        self.assertEqual(candidate["bindings"]["actual_torque"]["bit_offset"], 88)
        generated = esi_to_profile.render_cia402_cpp_fixture(
            candidate, array_name="kEsiCia402AllModesEntries",
            include_common_declarations=False,
        )
        fixture_path = (
            Path(__file__).resolve().parents[1]
            / "fixtures" / "ethercat_esi" / "cia402_all_modes_fixture.hpp"
        )
        self.assertEqual(generated, fixture_path.read_text(encoding="utf-8"))

    def test_requires_explicit_selection_and_all_core_objects(self):
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(esi_to_profile.EsiError, "both required"):
            esi_to_profile.resolve_cia402_candidate(device, [0x1600], [])
        device["pdos"][1]["entries"] = [
            entry for entry in device["pdos"][1]["entries"]
            if entry["index"] != 0x6041
        ]
        with self.assertRaisesRegex(esi_to_profile.EsiError, "missing required"):
            esi_to_profile.resolve_cia402_candidate(device, [0x1600], [0x1A00])

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


    def test_rejects_duplicate_selected_pdo_indices(self):
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(esi_to_profile.EsiError, "duplicate RXPDO selection"):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1600], [0x1A00]
            )

    def test_rejects_cia402_object_declared_in_wrong_pdo_direction(self):
        self.path.write_text(ESI_WITH_VELOCITY, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        rx_velocity = next(
            pdo for pdo in device["pdos"] if pdo["index"] == 0x1601
        )
        velocity_entry = rx_velocity["entries"].pop()
        tx_velocity = next(
            pdo for pdo in device["pdos"] if pdo["index"] == 0x1A01
        )
        tx_velocity["entries"].append(velocity_entry)
        with self.assertRaisesRegex(esi_to_profile.EsiError, "must be RXPDO/32-bit"):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_incomplete_optional_mode_pair_is_not_reported_as_supported(self):
        self.path.write_text(ESI_WITH_VELOCITY, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        # The device declares velocity feedback, but this selected mapping omits it.
        device["pdos"] = [
            pdo for pdo in device["pdos"] if pdo["index"] != 0x1A01
        ]
        result = esi_to_profile.resolve_cia402_candidate(
            device, [0x1600, 0x1601], [0x1A00]
        )
        self.assertIn("target_velocity", result["bindings"])
        self.assertNotIn("actual_velocity", result["bindings"])
        self.assertFalse(result["supports"]["profile_velocity"])
        self.assertFalse(result["supports"]["cyclic_synchronous_velocity"])

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

    def test_rejects_out_of_range_sync_manager_index(self):
        malformed = ESI.replace('<RxPdo Sm="2"', '<RxPdo Sm="256"', 1)
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(esi_to_profile.EsiError, "rxPDO.Sm value 256"):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_pdo_reference_to_undeclared_sync_manager(self):
        malformed = ESI.replace('<RxPdo Sm="2"', '<RxPdo Sm="4"', 1)
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "references SyncManager 4, but the device declares only 4",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_candidate_requires_selected_pdo_sync_manager_association(self):
        malformed = ESI_WITH_VELOCITY.replace(
            '<RxPdo Sm="2"><Index>#x1601</Index>',
            '<RxPdo><Index>#x1601</Index>',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(esi_to_profile.EsiError, "no SyncManager association"):
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


    def test_rejects_out_of_range_object_dictionary_fields(self):
        self.path.write_text(ESI.replace(
            "<Index>#x6040</Index>", "<Index>#x160040</Index>", 1
        ), encoding="utf-8")
        with self.assertRaisesRegex(esi_to_profile.EsiError, "outside 0..65535"):
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
