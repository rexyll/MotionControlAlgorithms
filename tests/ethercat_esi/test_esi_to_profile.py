import json
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
          <Entry><Index>24640</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>CW</Name><DataType>UINT</DataType></Entry>
        </RxPdo>
      </Device>
    </Devices>
  </Descriptions>
</EtherCATInfo>
"""


ESI_WITH_VELOCITY = ESI.replace(
    "      </Device>",
    """        <RxPdo Sm="2" Su="5"><Index>#x1601</Index><Name>Velocity command</Name>
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

SLOT_RUNTIME_ESI = """<?xml version="1.0"?>
<EtherCATInfo xmlns="https://www.ethercat.org/ESI">
  <Vendor><Id>#x000000AB</Id><Name>Example Vendor</Name></Vendor>
  <Descriptions><Devices><Device>
    <Type ProductCode="#x1001" RevisionNo="#x2">Modular Device</Type>
    <Name>Modular Device</Name>
    <Slots MaxSlotCount="#x2" MaxSlotGroupCount="#x2"
           SlotPdoIncrement="#x1" SlotGroupPdoIncrement="#x80"
           SlotIndexIncrement="#x10" SlotGroupIndexIncrement="#x400">
      <Slot SlotGroup="#x0" MinInstances="#x0" MaxInstances="#x1"
            SlotPdoIncrement="#x2" SlotIndexIncrement="#x20">
        <ModuleIdent>#x0</ModuleIdent>
      </Slot>
      <Slot SlotGroup="#x1" MinInstances="#x1" MaxInstances="#x1">
        <ModuleClass><Class>DI</Class><VendorId>#xAB</VendorId></ModuleClass>
      </Slot>
      <ModulePdoGroup Alignment="4" RxPdo="#x1700" TxPdo="#x1B80">
        Safety group
      </ModulePdoGroup>
    </Slots>
  </Device></Devices></Descriptions>
  <Modules>
    <Module>
      <Type ModuleIdent="#x4711" ModuleClass="DI" ModulePdoGroup="0">
        Digital input module
      </Type>
      <Name>Digital input module</Name>
      <RxPdo Sm="2">
        <Index DependOnSlot="true">#x1600</Index>
        <Name>Inputs</Name>
        <Exclude DependOnSlotGroup="true">#x1602</Exclude>
        <Entry>
          <Index DependOnSlotGroup="true">#x6000</Index>
          <SubIndex>1</SubIndex><BitLen>16</BitLen>
          <Name>Input bits</Name><DataType>UINT</DataType>
        </Entry>
        <Entry><Index>#x0</Index><BitLen>8</BitLen><Name>Padding</Name></Entry>
      </RxPdo>
      <TxPdo Sm="3">
        <Index DependOnSlotGroup="true">#x1A00</Index>
        <Name>Status</Name>
        <Entry>
          <Index DependOnSlot="true">#x6041</Index>
          <SubIndex>0</SubIndex><BitLen>16</BitLen>
          <Name>Statusword</Name><DataType>UINT</DataType>
        </Entry>
      </TxPdo>
    </Module>
  </Modules>
</EtherCATInfo>
"""


class EsiToProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "fixture.xml"
        self.path.write_text(ESI, encoding="utf-8")

    def test_preserves_localized_vendor_pdo_and_entry_names(self):
        localized = ESI.replace(
            "<Name>Example Vendor</Name>",
            '<Name LcId="1033">Example Vendor</Name><Name LcId="1031">Beispielanbieter</Name>',
            1,
        ).replace(
            "<Name>RxPDO</Name>",
            '<Name LcId="1033">RxPDO</Name><Name LcId="1031">Empfangs-PDO</Name>',
            1,
        ).replace(
            "<Name>Controlword</Name>",
            '<Name LcId="1033">Controlword</Name><Name LcId="1031">Steuerwort</Name>',
            1,
        )
        self.path.write_text(localized, encoding="utf-8")
        result = esi_to_profile.parse_esi(self.path)
        first = result["devices"][0]
        entry = first["pdos"][0]["entries"][0]
        self.assertEqual(result["vendor"]["name"], "Example Vendor")
        self.assertEqual(
            result["vendor"]["localized_names"],
            [
                {"language_id": 1033, "text": "Example Vendor"},
                {"language_id": 1031, "text": "Beispielanbieter"},
            ],
        )
        self.assertEqual(first["pdos"][0]["name"], "RxPDO")
        self.assertEqual(
            first["pdos"][0]["localized_names"][1]["text"], "Empfangs-PDO"
        )
        self.assertEqual(entry["name"], "Controlword")
        self.assertEqual(entry["localized_names"][1]["text"], "Steuerwort")

    def test_allows_optional_entry_name_data_type_and_subindex(self):
        normalized = ESI.replace(
            '<Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Controlword</Name><DataType>UINT</DataType></Entry>',
            '<Entry><Index>#x6040</Index><BitLen>16</BitLen></Entry>',
            1,
        )
        self.path.write_text(normalized, encoding="utf-8")
        entry = esi_to_profile.parse_esi(self.path)["devices"][0]["pdos"][0]["entries"][0]
        self.assertEqual(entry["index"], 0x6040)
        self.assertEqual(entry["sub_index"], 0)
        self.assertFalse(entry["sub_index_present"])
        self.assertIsNone(entry["name"])
        self.assertEqual(entry["localized_names"], [])
        self.assertIsNone(entry["data_type"])

    def test_preserves_full_safety_parameter_mapping_pdo_entries(self):
        modular = ESI.replace(
            "</EtherCATInfo>",
            """  <Modules>
    <Module><Type ModuleIdent="#x4711">FSoE input module</Type>
      <Name>FSoE module</Name>
      <SafetyParaMapping Fixed="true" Su="2" PdoOrder="3"
                         OSFac="4" OSMin="1" OSMax="8" OSIndexInc="2"
                         OverwrittenByModule="true" SRA_Parameter="false"
                         SafetyPdoType="FSoESlave" SafetyConnNumber="3">
        <Index DependOnSlotGroup="true">#x8000</Index>
        <Name LcId="1033">Safety parameter set</Name>
        <Entry><Index>#x2000</Index><BitLen>16</BitLen></Entry>
        <Entry><Index>#x2001</Index><BitLen>32</BitLen><Name LcId="1033">Safety value</Name><DataType>UDINT</DataType></Entry>
      </SafetyParaMapping>
    </Module>
  </Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        module = esi_to_profile.parse_esi(self.path)["modules"][0]
        mapping = module["safety_parameter_mappings"][0]
        self.assertTrue(module["is_safety_relevant"])
        self.assertEqual(mapping["index"], 0x8000)
        self.assertFalse(mapping["index_depend_on_slot"])
        self.assertTrue(mapping["index_depend_on_slot_group"])
        self.assertTrue(mapping["fixed"])
        self.assertEqual(mapping["sync_unit"], 2)
        self.assertEqual(mapping["pdo_order"], 3)
        self.assertEqual(mapping["oversampling_factor"], 4)
        self.assertEqual(mapping["oversampling_min"], 1)
        self.assertEqual(mapping["oversampling_max"], 8)
        self.assertEqual(mapping["oversampling_index_increment"], 2)
        self.assertTrue(mapping["overwritten_by_module"])
        self.assertFalse(mapping["sra_parameter"])
        self.assertEqual(mapping["safety_pdo_type"], "FSoESlave")
        self.assertEqual(mapping["safety_connection_number"], 3)
        self.assertEqual(len(mapping["entries"]), 2)
        self.assertFalse(mapping["entries"][0]["sub_index_present"])
        self.assertIsNone(mapping["entries"][0]["name"])
        self.assertIsNone(mapping["entries"][0]["data_type"])
        self.assertEqual(mapping["entries"][1]["index"], 0x2001)
        self.assertEqual(mapping["entries"][1]["name"], "Safety value")

    def test_parses_embedded_module_catalog_and_device_slots(self):
        modular = ESI.replace(
            "<Vendor>",
            '<InfoReference>VendorModules.xml</InfoReference><Vendor>',
            1,
        ).replace(
            "      </Device>",
            """        <Slots DownloadModuleIdentList="1" DownloadModuleAddressList="0"
               MaxSlotCount="#x2" SlotPdoIncrement="#x1" SlotIndexIncrement="#x10">
          <Slot SlotGroup="#x1" SlotPdoIncrement="#x2" SlotIndexIncrement="#x20"
                TreeView="SLOT" MinInstances="#x1" MaxInstances="0x1">
            <Name LcId="1033">FirstSlot</Name>
            <ModuleIdent Default="#x1">#x00004711</ModuleIdent>
            <ModuleIdent>#x00004712</ModuleIdent>
          </Slot>
          <Slot MinInstances="0" MaxInstances="1">
            <Name>OptionalSlot</Name>
            <ModuleIdent>#x00000000</ModuleIdent>
            <ModuleIdent>#x00004712</ModuleIdent>
          </Slot>
          <Slot MinInstances="1" MaxInstances="1" SlotGroup="#x1"
                SlotGroupPdoIncrement="#x3" SlotGroupIndexIncrement="#x30">
            <Name>Class-selected slot</Name>
            <ModuleClass>
              <Class>DI</Class><VendorId>#x000000AB</VendorId>
              <Name LcId="1033">Digital input family</Name>
              <ImageFile16x14>di-module.bmp</ImageFile16x14>
            </ModuleClass>
            <ModuleClass><Class>DI-Unqualified</Class></ModuleClass>
            <ImageData16x14>00FF10</ImageData16x14>
          </Slot>
          <SlotGroupData SlotGroup="#x1"><Name>InputGroup</Name></SlotGroupData>
          <ModulePdoGroup Alignment="4" RxPdo="#x1600" TxPdo="#x1A00">Basic IO</ModulePdoGroup>
          <SlotSelections>
            <Name>Two module option</Name>
            <ModuleIdent>#x00004711</ModuleIdent>
            <ModuleIdent>#x00004712</ModuleIdent>
          </SlotSelections>
        </Slots>
      </Device>""",
            1,
        ).replace(
            "</EtherCATInfo>",
            """  <Modules>
    <Module Crc32="#xDEADBEEF">
      <Type ModuleIdent="#x00004711" ModuleClass="DI" ModulePdoGroup="0"
            SRA_ParameterSupported="true">Digital input module</Type>
      <Name LcId="1033">2-channel input</Name>
      <RxPdo Sm="2">
        <Index DependOnSlot="true">#x1600</Index><Name>Module input</Name>
        <Entry><Index>#x6000</Index><SubIndex>1</SubIndex><BitLen>16</BitLen><Name>Input bits</Name><DataType>UINT</DataType></Entry>
      </RxPdo>
    </Module>
    <Module>
      <Type ModuleIdent="18194" ModuleClass="DI">Digital input alternative</Type>
      <Name>4-channel input</Name>
    </Module>
  </Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        result = esi_to_profile.parse_esi(self.path)
        self.assertEqual(result["info_references"], ["VendorModules.xml"])
        self.assertEqual(
            [module["module_ident"] for module in result["modules"]],
            [0x4711, 0x4712],
        )
        module = result["modules"][0]
        self.assertEqual(module["module_class"], "DI")
        self.assertEqual(module["module_pdo_group"], 0)
        self.assertTrue(module["sra_parameter_supported"])
        self.assertEqual(module["crc32"], 0xDEADBEEF)
        self.assertTrue(module["pdos"][0]["index_depend_on_slot"])
        self.assertEqual(module["pdos"][0]["entries"][0]["index"], 0x6000)

        slots = result["devices"][0]["slots"]
        self.assertTrue(slots["download_module_ident_list"])
        self.assertFalse(slots["download_module_address_list"])
        self.assertEqual(slots["slot_pdo_increment"], 1)
        self.assertEqual(slots["slot_index_increment"], 0x10)
        self.assertEqual(slots["slots"][0]["slot_group"], 1)
        self.assertEqual(slots["slots"][0]["min_instances"], 1)
        self.assertEqual(slots["slots"][0]["increments"]["slot_pdo_increment"], 2)
        self.assertEqual(slots["slots"][0]["effective_increments"]["slot_index_increment"], 0x20)
        self.assertEqual(slots["slots"][0]["tree_view"], "SLOT")
        self.assertEqual(slots["slots"][2]["module_class_options"][0]["class"], "DI")
        self.assertEqual(slots["slots"][2]["module_class_options"][0]["vendor_id"], 0xAB)
        self.assertEqual(slots["slots"][2]["module_class_options"][0]["vendor_id_explicit"], 0xAB)
        self.assertEqual(slots["slots"][2]["module_class_options"][0]["image"]["image_file_16x14"], "di-module.bmp")
        self.assertEqual(slots["slots"][2]["module_class_options"][1]["vendor_id"], 0xAB)
        self.assertIsNone(slots["slots"][2]["module_class_options"][1]["vendor_id_explicit"])
        self.assertEqual(slots["slots"][2]["image"]["image_data_16x14"], "00FF10")
        self.assertEqual(slots["slots"][0]["module_list_subindex"], 1)
        self.assertEqual(slots["slots"][0]["default_module_ident"], 0x4711)
        self.assertEqual(
            [item["module_ident"] for item in slots["slots"][1]["module_options"]],
            [0, 0x4712],
        )
        self.assertTrue(slots["slots"][1]["module_options"][0]["is_empty_slot"])
        self.assertEqual(slots["slot_group_data"][0]["slot_group"], 1)
        self.assertEqual(slots["module_pdo_groups"][0]["alignment"], 4)
        self.assertEqual(slots["module_pdo_groups"][0]["rx_pdo"], 0x1600)
        self.assertEqual(slots["slot_selections"][0]["module_idents"], [0x4711, 0x4712])

    def test_rejects_module_pdo_group_reference_without_group_definition(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots>
          <Slot MinInstances="1" MaxInstances="1">
            <ModuleIdent>#x4711</ModuleIdent>
          </Slot>
          <ModulePdoGroup Alignment="4" RxPdo="#x1600" TxPdo="#x1A00"/>
        </Slots>
      </Device>""",
            1,
        ).replace(
            "</EtherCATInfo>",
            """  <Modules>
    <Module><Type ModuleIdent="#x4711" ModulePdoGroup="1">Input module</Type>
      <Name>Input module</Name></Module>
  </Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "ModuleIdent 0x00004711 references ModulePdoGroup 1, but Slots declares only 1 group",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_used_module_pdo_group_with_missing_alignment_fields(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots>
          <Slot MinInstances="1" MaxInstances="1">
            <ModuleIdent>#x4711</ModuleIdent>
          </Slot>
          <ModulePdoGroup Alignment="4" RxPdo="#x1600"/>
        </Slots>
      </Device>""",
            1,
        ).replace(
            "</EtherCATInfo>",
            """  <Modules>
    <Module><Type ModuleIdent="#x4711" ModulePdoGroup="0">Input module</Type>
      <Name>Input module</Name></Module>
  </Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Slots.ModulePdoGroup\\[0\\] is used by ModuleIdent 0x00004711 but is missing required tx_pdo",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_duplicate_module_pdo_group_alignment_indices(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots>
          <ModulePdoGroup Alignment="2" RxPdo="#x1600" TxPdo="#x1A00"/>
          <ModulePdoGroup Alignment="4" RxPdo="#x1600" TxPdo="#x1A01"/>
        </Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Slots.ModulePdoGroup RxPdo 0x1600 is duplicated by groups 0 and 1",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_validates_external_module_pdo_group_after_catalog_attachment(self):
        modular = ESI.replace(
            "<Vendor>",
            '<InfoReference>vendor-modules.xml</InfoReference><Vendor>',
            1,
        ).replace(
            "      </Device>",
            """        <Slots>
          <Slot MinInstances="1" MaxInstances="1">
            <ModuleIdent>#x4711</ModuleIdent>
          </Slot>
          <ModulePdoGroup Alignment="4" RxPdo="#x1600" TxPdo="#x1A00"/>
        </Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        module_path = Path(self.temp.name) / "vendor-modules.xml"
        module_path.write_text(
            """<EtherCATModule>
  <Vendor><Id>#xAB</Id><Name>Example Vendor</Name></Vendor>
  <Modules>
    <Module><Type ModuleIdent="#x4711" ModulePdoGroup="1">Input module</Type>
      <Name>Input module</Name></Module>
  </Modules>
</EtherCATModule>""",
            encoding="utf-8",
        )
        document = esi_to_profile.parse_esi(self.path)
        self.assertFalse(document["module_catalog_complete"])
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "ModuleIdent 0x00004711 references ModulePdoGroup 1, but Slots declares only 1 group",
        ):
            esi_to_profile.attach_external_module_files(document, [module_path])

    def test_rejects_negative_module_pdo_group_number(self):
        modular = ESI.replace(
            "</EtherCATInfo>",
            """  <Modules>
    <Module><Type ModuleIdent="#x4711" ModulePdoGroup="-1">Bad module</Type>
      <Name>Bad module</Name></Module>
  </Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Module.Type.ModulePdoGroup value -1 must be non-negative",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_negative_device_module_pdo_group_without_slots(self):
        malformed = ESI.replace(
            '<Type ProductCode="#x00001001" RevisionNo="#x00000002">',
            '<Type ProductCode="#x00001001" RevisionNo="#x00000002" ModulePdoGroup="-1">',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Device.Type.ModulePdoGroup value -1 must be non-negative",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_device_module_pdo_group_without_slots_description(self):
        malformed = ESI.replace(
            '<Type ProductCode="#x00001001" RevisionNo="#x00000002">',
            '<Type ProductCode="#x00001001" RevisionNo="#x00000002" ModulePdoGroup="0">',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "uses ModulePdoGroup 0 without a Slots/ModulePdoGroup description",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_negative_module_pdo_group_alignment(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots>
          <ModulePdoGroup Alignment="-1" RxPdo="#x1600" TxPdo="#x1A00"/>
        </Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Slots.ModulePdoGroup\\[0\\] Alignment must be non-negative",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_duplicate_tx_alignment_pdo_indices(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots>
          <ModulePdoGroup Alignment="2" RxPdo="#x1600" TxPdo="#x1A00"/>
          <ModulePdoGroup Alignment="4" RxPdo="#x1601" TxPdo="#x1A00"/>
        </Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Slots.ModulePdoGroup TxPdo 0x1A00 is duplicated by groups 0 and 1",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_parses_module_mailbox_coe_init_commands_without_executing_them(self):
        modular = ESI.replace(
            "</EtherCATInfo>",
            """  <Modules>
    <Module><Type ModuleIdent="#x4711">CoE module</Type>
      <Name>CoE module</Name>
      <Mailbox SdoInfo="1" PdoAssign="true" PdoConfig="0"
               PdoUpload="false" CompleteAccess="1" SegmentedSdo="true"
               ModuleOD="1" EdsFile="module.eds"
               DS402Channels="4" DiagHistory="true">
        <CoE>
          <InitCmd Fixed="true" CompleteAccess="false" OverwrittenByModule="1">
            <Transition>IP</Transition><Transition>PS</Transition>
            <Index DependOnSlot="true">#x8000</Index>
            <SubIndex>#x1</SubIndex>
            <Data AdaptAutomatically="true">01 02 aF</Data>
            <Comment>Initialize module parameters</Comment>
          </InitCmd>
        </CoE>
      </Mailbox>
    </Module>
  </Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        module = esi_to_profile.parse_esi(self.path)["modules"][0]
        self.assertTrue(module["has_mailbox"])
        mailbox = module["mailbox"]
        self.assertTrue(mailbox["has_coe"])
        self.assertEqual(mailbox["protocols"], {"coe": True})
        self.assertTrue(mailbox["attributes"]["sdo_info"])
        self.assertTrue(mailbox["attributes"]["pdo_assign"])
        self.assertFalse(mailbox["attributes"]["pdo_config"])
        self.assertFalse(mailbox["attributes"]["pdo_upload"])
        self.assertTrue(mailbox["attributes"]["complete_access"])
        self.assertTrue(mailbox["attributes"]["segmented_sdo"])
        self.assertTrue(mailbox["attributes"]["module_od"])
        self.assertEqual(mailbox["attributes"]["eds_file"], "module.eds")
        self.assertEqual(mailbox["attributes"]["ds402_channels"], 4)
        self.assertTrue(mailbox["attributes"]["diag_history"])
        command = mailbox["coe_init_commands"][0]
        self.assertTrue(command["fixed"])
        self.assertFalse(command["complete_access"])
        self.assertTrue(command["overwritten_by_module"])
        self.assertEqual(command["transitions"], ["IP", "PS"])
        self.assertEqual(command["index"], 0x8000)
        self.assertTrue(command["index_depend_on_slot"])
        self.assertEqual(command["sub_index"], 1)
        self.assertEqual(command["data_hex"], "0102AF")
        self.assertTrue(command["adapt_automatically"])
        self.assertEqual(command["comment"], "Initialize module parameters")
        self.assertNotIn("executed", command)

    def test_rejects_non_coe_children_in_module_mailbox(self):
        for protocol in ("AoE", "EoE", "FoE", "SoE", "VoE"):
            with self.subTest(protocol=protocol):
                modular = ESI.replace(
                    "</EtherCATInfo>",
                    f"""  <Modules>
    <Module><Type ModuleIdent="#x4711">Module with {protocol}</Type>
      <Name>Module with {protocol}</Name>
      <Mailbox><{protocol}/></Mailbox>
    </Module>
  </Modules>
</EtherCATInfo>""",
                    1,
                )
                self.path.write_text(modular, encoding="utf-8")
                with self.assertRaisesRegex(
                    esi_to_profile.EsiError,
                    f"Module.Mailbox supports only the CoE child in ModuleType; unsupported child <{protocol}>",
                ):
                    esi_to_profile.parse_esi(self.path)

    def test_rejects_invalid_module_coe_init_command_transition(self):
        modular = ESI.replace(
            "</EtherCATInfo>",
            """  <Modules><Module><Type ModuleIdent="#x4711">CoE</Type>
      <Name>CoE</Name><Mailbox><CoE><InitCmd><Transition>INVALID</Transition>
        <Index>#x8000</Index><SubIndex>0</SubIndex><Data>00</Data>
      </InitCmd></CoE></Mailbox>
    </Module></Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "contains invalid Transition 'INVALID'",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_invalid_module_coe_init_hex_data(self):
        modular = ESI.replace(
            "</EtherCATInfo>",
            """  <Modules><Module><Type ModuleIdent="#x4711">CoE</Type>
      <Name>CoE</Name><Mailbox><CoE><InitCmd><Transition>IP</Transition>
        <Index>#x8000</Index><SubIndex>0</SubIndex><Data>0G</Data>
      </InitCmd></CoE></Mailbox>
    </Module></Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Data is not valid xs:hexBinary data",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_preserves_common_pdo_type_attributes_on_regular_pdo(self):
        annotated = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true" Su="3" PdoOrder="7" OSFac="4" OSMin="1" OSMax="8" OSIndexInc="2" OverwrittenByModule="true" SRA_Parameter="false">',
            1,
        )
        self.path.write_text(annotated, encoding="utf-8")
        pdo = esi_to_profile.parse_esi(self.path)["devices"][0]["pdos"][0]
        self.assertEqual(pdo["sync_manager"], 2)
        self.assertEqual(pdo["sync_unit"], 3)
        self.assertEqual(pdo["pdo_order"], 7)
        self.assertEqual(pdo["oversampling_factor"], 4)
        self.assertEqual(pdo["oversampling_min"], 1)
        self.assertEqual(pdo["oversampling_max"], 8)
        self.assertEqual(pdo["oversampling_index_increment"], 2)
        self.assertTrue(pdo["overwritten_by_module"])
        self.assertFalse(pdo["sra_parameter"])

    def test_rejects_non_decimal_xs_int_in_pdo_type_attribute(self):
        malformed = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true" PdoOrder="#x1">',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "invalid rxPDO.PdoOrder",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_invalid_safety_para_mapping_type(self):
        modular = ESI.replace(
            "</EtherCATInfo>",
            """  <Modules><Module><Type ModuleIdent="#x4711">Safety Module</Type>
      <Name>Safety Module</Name>
      <SafetyParaMapping SafetyPdoType="UnknownSafetyType">
        <Index>#x8000</Index><Name>Safety parameters</Name>
      </SafetyParaMapping>
    </Module></Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "invalid Module.SafetyParaMapping.SafetyPdoType",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_parses_external_ethercat_module_document(self):
        module_doc = """<?xml version="1.0"?>
<EtherCATModule>
  <Vendor><Id>#xAB</Id><Name>Example Vendor</Name></Vendor>
  <Modules>
    <Module Crc32="#x1234">
      <Type ModuleIdent="#x4711" ModuleClass="DI">External digital input</Type>
      <Name>External module</Name>
      <RxPdo Sm="2"><Index>#x1600</Index><Name>Inputs</Name>
        <Entry><Index>#x6000</Index><SubIndex>1</SubIndex><BitLen>16</BitLen><Name>Input</Name><DataType>UINT</DataType></Entry>
      </RxPdo>
    </Module>
  </Modules>
</EtherCATModule>
"""
        module_path = Path(self.temp.name) / "vendor-modules.xml"
        module_path.write_text(module_doc, encoding="utf-8")
        parsed = esi_to_profile.parse_module_esi(module_path)
        self.assertEqual(parsed["vendor"]["id"], 0xAB)
        self.assertEqual(parsed["modules"][0]["module_ident"], 0x4711)
        self.assertEqual(parsed["modules"][0]["crc32"], 0x1234)

    def test_explicitly_merges_info_referenced_external_module_catalog(self):
        base = ESI.replace(
            "<Vendor>",
            '<InfoReference>vendor-modules.xml</InfoReference><Vendor>',
            1,
        )
        self.path.write_text(base, encoding="utf-8")
        module_path = Path(self.temp.name) / "vendor-modules.xml"
        module_path.write_text(
            """<EtherCATModule>
  <Vendor><Id>#xAB</Id><Name>Example Vendor</Name></Vendor>
  <Modules><Module><Type ModuleIdent="#x4711">Digital input</Type>
    <Name>Input module</Name></Module></Modules>
</EtherCATModule>""",
            encoding="utf-8",
        )
        document = esi_to_profile.parse_esi(self.path)
        self.assertEqual(document["modules"], [])
        esi_to_profile.attach_external_module_files(document, [module_path])
        self.assertEqual(
            [module["module_ident"] for module in document["modules"]],
            [0x4711],
        )
        self.assertEqual(
            document["external_module_sources"][0]["file_name"],
            "vendor-modules.xml",
        )

    def test_rejects_external_module_file_not_named_by_info_reference(self):
        self.path.write_text(ESI, encoding="utf-8")
        module_path = Path(self.temp.name) / "unreferenced.xml"
        module_path.write_text(
            """<EtherCATModule><Vendor><Id>#xAB</Id><Name>V</Name></Vendor>
<Modules/></EtherCATModule>""",
            encoding="utf-8",
        )
        document = esi_to_profile.parse_esi(self.path)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "is not listed in InfoReference",
        ):
            esi_to_profile.attach_external_module_files(document, [module_path])

    def test_rejects_external_module_vendor_id_mismatch(self):
        self.path.write_text(
            ESI.replace(
                "<Vendor>",
                '<InfoReference>external.xml</InfoReference><Vendor>',
                1,
            ),
            encoding="utf-8",
        )
        module_path = Path(self.temp.name) / "external.xml"
        module_path.write_text(
            """<EtherCATModule><Vendor><Id>#xAC</Id><Name>Other</Name></Vendor>
<Modules/></EtherCATModule>""",
            encoding="utf-8",
        )
        document = esi_to_profile.parse_esi(self.path)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "does not match ESI Vendor ID",
        ):
            esi_to_profile.attach_external_module_files(document, [module_path])

    def test_rejects_duplicate_ident_across_embedded_and_external_modules(self):
        embedded = ESI.replace(
            "</EtherCATInfo>",
            """<Modules><Module><Type ModuleIdent="#x4711">Embedded</Type>
<Name>Embedded module</Name></Module></Modules>
</EtherCATInfo>""",
            1,
        ).replace(
            "<Vendor>",
            '<InfoReference>external.xml</InfoReference><Vendor>',
            1,
        )
        self.path.write_text(embedded, encoding="utf-8")
        module_path = Path(self.temp.name) / "external.xml"
        module_path.write_text(
            """<EtherCATModule><Vendor><Id>#xAB</Id><Name>Example Vendor</Name></Vendor>
<Modules><Module><Type ModuleIdent="#x4711">External</Type>
<Name>Duplicate module</Name></Module></Modules></EtherCATModule>""",
            encoding="utf-8",
        )
        document = esi_to_profile.parse_esi(self.path)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "duplicate ModuleIdent 0x00004711 across embedded and external module catalogs",
        ):
            esi_to_profile.attach_external_module_files(document, [module_path])



    def _parse_slot_runtime_fixture(self, xml=SLOT_RUNTIME_ESI):
        self.path.write_text(xml, encoding="utf-8")
        return esi_to_profile.parse_esi(self.path)

    def test_resolves_explicit_slot_positions_groups_and_alignment_metadata(self):
        document = self._parse_slot_runtime_fixture()
        device = document["devices"][0]
        resolved = esi_to_profile.resolve_slot_pdo_configuration(
            document,
            device,
            [
                {"slot_descriptor_index": 0, "module_ident": 0},
                {"slot_descriptor_index": 1, "module_ident": 0x4711},
            ],
        )

        self.assertEqual(
            [item["index"] for item in resolved["module_pdos"]],
            [0x1601, 0x1A80],
        )
        rx_pdo, tx_pdo = resolved["module_pdos"]
        self.assertEqual(rx_pdo["base_index"], 0x1600)
        self.assertEqual(rx_pdo["entries"][0]["index"], 0x6400)
        self.assertEqual(rx_pdo["entries"][1]["index"], 0)
        self.assertEqual(rx_pdo["excluded_pdos"][0]["index"], 0x1682)
        self.assertEqual(tx_pdo["entries"][0]["index"], 0x6051)
        self.assertFalse(rx_pdo["index_depend_on_slot"])
        self.assertFalse(rx_pdo["entries"][0]["index_depend_on_slot_group"])
        self.assertEqual(
            resolved["module_pdo_alignment_groups"],
            [{
                "index": 0,
                "alignment": 4,
                "rx_pdo": 0x1700,
                "tx_pdo": 0x1B80,
                "module_idents": [0x4711],
                "slot_indices": [1],
            }],
        )
        self.assertFalse(resolved["runtime_readback_performed"])
        self.assertEqual(resolved["configured_slots"][0]["module_ident"], 0)

    def test_includes_device_level_module_pdo_group_alignment(self):
        xml = SLOT_RUNTIME_ESI.replace(
            '<Type ProductCode="#x1001" RevisionNo="#x2">Modular Device</Type>',
            '<Type ProductCode="#x1001" RevisionNo="#x2" ModulePdoGroup="0">Modular Device</Type>',
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        resolved = esi_to_profile.resolve_slot_pdo_configuration(
            document,
            document["devices"][0],
            [
                {"slot_descriptor_index": 0, "module_ident": 0},
                {"slot_descriptor_index": 1, "module_ident": 0x4711},
            ],
        )
        group = resolved["module_pdo_alignment_groups"][0]
        self.assertTrue(group["device"])
        self.assertEqual(group["device_name"], "Modular Device")
        self.assertEqual(group["module_idents"], [0x4711])

    def test_rejects_collision_with_non_overwritten_device_pdo(self):
        xml = SLOT_RUNTIME_ESI.replace(
            '    <Slots MaxSlotCount=',
            """    <RxPdo Sm="2"><Index>#x1601</Index><Name>Device PDO</Name>
      <Entry><Index>#x6000</Index><SubIndex>1</SubIndex><BitLen>16</BitLen><Name>Device input</Name><DataType>UINT</DataType></Entry>
    </RxPdo>
    <Slots MaxSlotCount=""",
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "collides with a device-level PDO that is not marked OverwrittenByModule",
        ):
            esi_to_profile.resolve_slot_pdo_configuration(
                document,
                document["devices"][0],
                [
                    {"slot_descriptor_index": 0, "module_ident": 0},
                    {"slot_descriptor_index": 1, "module_ident": 0x4711},
                ],
            )

    def test_module_pdo_replaces_device_pdo_marked_overwritten_by_module(self):
        xml = SLOT_RUNTIME_ESI.replace(
            '    <Slots MaxSlotCount=',
            """    <RxPdo Sm="2" OverwrittenByModule="true"><Index>#x1601</Index><Name>Device PDO</Name>
      <Entry><Index>#x6000</Index><SubIndex>1</SubIndex><BitLen>16</BitLen><Name>Device input</Name><DataType>UINT</DataType></Entry>
    </RxPdo>
    <Slots MaxSlotCount=""",
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        resolved = esi_to_profile.resolve_slot_pdo_configuration(
            document,
            document["devices"][0],
            [
                {"slot_descriptor_index": 0, "module_ident": 0},
                {"slot_descriptor_index": 1, "module_ident": 0x4711},
            ],
        )
        self.assertEqual(len(resolved["device_pdos"]), 1)
        self.assertEqual(len(resolved["effective_device_pdos"]), 0)
        self.assertEqual(len(resolved["overridden_device_pdos"]), 1)

    def test_cli_accepts_explicit_slot_configuration_json(self):
        document_path = self.path
        document_path.write_text(SLOT_RUNTIME_ESI, encoding="utf-8")
        config_path = Path(self.temp.name) / "slots.json"
        config_path.write_text(
            json.dumps([
                {"slot_descriptor_index": 0, "module_ident": 0},
                {"slot_descriptor_index": 1, "module_ident": "#x4711"},
            ]),
            encoding="utf-8",
        )
        output_path = Path(self.temp.name) / "resolved.json"
        result = esi_to_profile.main([
            str(document_path),
            "--slot-config", str(config_path),
            "--output", str(output_path),
        ])
        self.assertEqual(result, 0)
        output = json.loads(output_path.read_text(encoding="utf-8"))
        resolved = output["slot_pdo_configuration"]
        self.assertEqual(
            [pdo["index"] for pdo in resolved["module_pdos"]],
            [0x1601, 0x1A80],
        )
        self.assertFalse(resolved["runtime_readback_performed"])
        self.assertFalse(resolved["is_complete_process_data_mapping"])

    def test_cli_rejects_combining_slot_config_with_static_cia402_selection(self):
        document_path = self.path
        document_path.write_text(SLOT_RUNTIME_ESI, encoding="utf-8")
        config_path = Path(self.temp.name) / "slots.json"
        config_path.write_text("[]", encoding="utf-8")
        result = esi_to_profile.main([
            str(document_path),
            "--slot-config", str(config_path),
            "--rx-pdo", "0x1600",
        ])
        self.assertEqual(result, 2)

    def test_rejects_missing_required_slot_instance(self):
        document = self._parse_slot_runtime_fixture()
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            r"Slots\.Slot\[1\] has 0 configured instance",
        ):
            esi_to_profile.resolve_slot_pdo_configuration(
                document,
                document["devices"][0],
                [{"slot_descriptor_index": 0, "module_ident": 0}],
            )

    def test_rejects_module_class_choice_that_does_not_match(self):
        xml = SLOT_RUNTIME_ESI.replace(
            "<Class>DI</Class><VendorId>#xAB</VendorId>",
            "<Class>AI</Class><VendorId>#xAB</VendorId>",
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "does not match a ModuleClass choice",
        ):
            esi_to_profile.resolve_slot_pdo_configuration(
                document,
                document["devices"][0],
                [
                    {"slot_descriptor_index": 0, "module_ident": 0},
                    {"slot_descriptor_index": 1, "module_ident": 0x4711},
                ],
            )

    def test_rejects_slot_dependent_index_when_increment_is_missing(self):
        xml = SLOT_RUNTIME_ESI.replace(
            'SlotIndexIncrement="#x10" ',
            "",
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "slot_index_increment is not defined",
        ):
            esi_to_profile.resolve_slot_pdo_configuration(
                document,
                document["devices"][0],
                [
                    {"slot_descriptor_index": 0, "module_ident": 0},
                    {"slot_descriptor_index": 1, "module_ident": 0x4711},
                ],
            )

    def test_rejects_combined_slot_and_slot_group_index_flags(self):
        xml = SLOT_RUNTIME_ESI.replace(
            '<Index DependOnSlotGroup="true">#x6000</Index>',
            '<Index DependOnSlot="true" DependOnSlotGroup="true">#x6000</Index>',
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "combined offset semantics are not implemented",
        ):
            esi_to_profile.resolve_slot_pdo_configuration(
                document,
                document["devices"][0],
                [
                    {"slot_descriptor_index": 0, "module_ident": 0},
                    {"slot_descriptor_index": 1, "module_ident": 0x4711},
                ],
            )

    def test_rejects_group_dependent_index_without_slot_group(self):
        xml = SLOT_RUNTIME_ESI.replace(
            '<Slot SlotGroup="#x1" MinInstances="#x1" MaxInstances="#x1">',
            '<Slot MinInstances="#x1" MaxInstances="#x1">',
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "selected slot has no SlotGroup",
        ):
            esi_to_profile.resolve_slot_pdo_configuration(
                document,
                document["devices"][0],
                [
                    {"slot_descriptor_index": 0, "module_ident": 0},
                    {"slot_descriptor_index": 1, "module_ident": 0x4711},
                ],
            )

    def test_rejects_duplicate_resolved_module_pdo_indices(self):
        xml = SLOT_RUNTIME_ESI.replace(
            'MinInstances="#x1" MaxInstances="#x1">',
            'MinInstances="#x1" MaxInstances="#x2">',
            1,
        ).replace(
            '<Index DependOnSlot="true">#x1600</Index>',
            '<Index>#x1600</Index>',
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "collides between configured slots",
        ):
            esi_to_profile.resolve_slot_pdo_configuration(
                document,
                document["devices"][0],
                [
                    {"slot_descriptor_index": 1, "module_ident": 0x4711},
                    {"slot_descriptor_index": 1, "module_ident": 0x4711},
                ],
            )

    def test_rejects_resolved_pdo_index_outside_direction_range(self):
        xml = SLOT_RUNTIME_ESI.replace(
            '<Index DependOnSlot="true">#x1600</Index>',
            '<Index DependOnSlot="true">#x17FF</Index>',
            1,
        )
        document = self._parse_slot_runtime_fixture(xml)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "resolved RXPDO index 0x1800 is outside",
        ):
            esi_to_profile.resolve_slot_pdo_configuration(
                document,
                document["devices"][0],
                [
                    {"slot_descriptor_index": 0, "module_ident": 0},
                    {"slot_descriptor_index": 1, "module_ident": 0x4711},
                ],
            )

    def test_accepts_module_only_device_with_slots(self):
        base = ESI
        start = base.index('        <RxPdo Sm="2" Fixed="true" Mandatory="true">')
        end = base.index("        </TxPdo>", start) + len("        </TxPdo>")
        base = base[:start] + base[end:]
        modular = base.replace(
            "      </Device>",
            """        <Slots>
          <Slot MinInstances="1" MaxInstances="1">
            <ModuleIdent Default="#x1">#x4711</ModuleIdent>
          </Slot>
        </Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        self.assertEqual(device["pdos"], [])
        self.assertEqual(device["slots"]["slots"][0]["default_module_ident"], 0x4711)
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "slot-configurable device requires an explicit slot-resolved PDO map",
        ):
            esi_to_profile.resolve_cia402_candidate(device, [0x1600], [0x1A00])

    def test_rejects_static_candidate_resolution_when_device_has_slots(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="1" MaxInstances="1">
          <ModuleIdent Default="#x1">#x4711</ModuleIdent>
        </Slot></Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        self.assertTrue(device["pdos"])
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "slot-configurable device requires an explicit slot-resolved PDO map",
        ):
            esi_to_profile.resolve_cia402_candidate(device, [0x1600], [0x1A00])

    def test_rejects_slot_module_ident_missing_from_complete_embedded_catalog(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="0" MaxInstances="1">
          <ModuleIdent>#x4712</ModuleIdent>
        </Slot></Slots>
      </Device>""",
            1,
        ).replace(
            "</EtherCATInfo>",
            """  <Modules><Module><Type ModuleIdent="#x4711">Known module</Type>
    <Name>Known module</Name></Module></Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Slots.Slot\\[0\\] references unknown ModuleIdent 0x00004712",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_validates_slot_module_ident_after_all_external_catalogs_are_attached(self):
        modular = ESI.replace(
            "<Vendor>",
            '<InfoReference>external-modules.xml</InfoReference><Vendor>',
            1,
        ).replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="0" MaxInstances="1">
          <ModuleIdent>#x4711</ModuleIdent>
        </Slot></Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        external = Path(self.temp.name) / "external-modules.xml"
        external.write_text(
            """<EtherCATModule><Vendor><Id>#xAB</Id><Name>Example Vendor</Name></Vendor>
<Modules><Module><Type ModuleIdent="#x4712">Different module</Type>
<Name>Different module</Name></Module></Modules></EtherCATModule>""",
            encoding="utf-8",
        )
        document = esi_to_profile.parse_esi(self.path)
        self.assertFalse(document["module_catalog_complete"])
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Slots.Slot\\[0\\] references unknown ModuleIdent 0x00004711",
        ):
            esi_to_profile.attach_external_module_files(document, [external])

    def test_rejects_duplicate_module_idents(self):
        modular = ESI.replace(
            "</EtherCATInfo>",
            """  <Modules>
    <Module><Type ModuleIdent="#x4711">Module One</Type><Name>One</Name></Module>
    <Module><Type ModuleIdent="18193">Module Duplicate</Type><Name>Duplicate</Name></Module>
  </Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "duplicate ModuleIdent 0x00004711",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_multiple_default_module_choices_in_slot(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="1" MaxInstances="1">
          <ModuleIdent Default="1">#x4711</ModuleIdent>
          <ModuleIdent Default="#x1">#x4712</ModuleIdent>
        </Slot></Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "more than one default ModuleIdent",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_slot_without_module_choice(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="0" MaxInstances="1">
          <Name>No choices</Name>
        </Slot></Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "must define at least one ModuleIdent or ModuleClass option",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_mixed_slot_module_ident_and_module_class_choices(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="0" MaxInstances="1">
          <ModuleIdent>#x4711</ModuleIdent>
          <ModuleClass><Class>DI</Class></ModuleClass>
        </Slot></Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "cannot mix ModuleIdent and ModuleClass choices",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_unknown_identify_module_by_value(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots IdentifyModuleBy="Unknown">
          <Slot MinInstances="0" MaxInstances="1">
            <ModuleIdent>#x4711</ModuleIdent>
          </Slot>
        </Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "invalid Slots.IdentifyModuleBy",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_unknown_slot_tree_view(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="0" MaxInstances="1" TreeView="UNKNOWN">
          <ModuleIdent>#x4711</ModuleIdent>
        </Slot></Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "invalid TreeView",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_module_pdo_group_indices_in_wrong_direction(self):
        malformed = ESI.replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="0" MaxInstances="1">
          <ModuleIdent>#x4711</ModuleIdent>
        </Slot>
        <ModulePdoGroup Alignment="1" RxPdo="#x1A00"/>
        </Slots>
      </Device>""",
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Slots.ModulePdoGroup RxPdo 0x1A00 is outside 0x1600..0x17FF",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_slot_minimum_above_maximum(self):
        modular = ESI.replace(
            "      </Device>",
            """        <Slots><Slot MinInstances="2" MaxInstances="1">
          <ModuleIdent>#x4711</ModuleIdent>
        </Slot></Slots>
      </Device>""",
            1,
        )
        self.path.write_text(modular, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "MinInstances exceeds MaxInstances",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_normalizes_vendor_multiple_devices_and_pdo_directions(self):
        result = esi_to_profile.parse_esi(self.path)
        self.assertEqual(result["vendor"]["id"], 0xAB)
        self.assertEqual(result["vendor"]["name"], "Example Vendor")
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
          <Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Controlword</Name><DataType>UINT</DataType></Entry>
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
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">\n          <ExcludedSm>3</ExcludedSm>',
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

    def test_rejects_duplicate_static_indices_after_dynamic_declaration(self):
        dynamic = ESI.replace(
            "<Index>#x1600</Index>",
            '<Index DependOnSlot="true">#x1600</Index>',
            1,
        )
        malformed = dynamic.replace(
            "      </Device>",
            """        <RxPdo Sm="2"><Index>#x1600</Index><Name>Static duplicate A</Name>
          <Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Duplicate A entry</Name><DataType>UINT</DataType></Entry>
        </RxPdo>
        <RxPdo Sm="2"><Index>#x1600</Index><Name>Static duplicate B</Name>
          <Entry><Index>#x6041</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Duplicate B entry</Name><DataType>UINT</DataType></Entry>
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

    def test_rejects_slot_dependent_object_entry_indices_in_static_candidate(self):
        dynamic = ESI_WITH_VELOCITY.replace(
            "<Index>#x6040</Index>",
            '<Index DependOnSlotGroup="true">#x6040</Index>',
            1,
        )
        self.path.write_text(dynamic, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        controlword = device["pdos"][0]["entries"][0]
        self.assertTrue(controlword["index_depend_on_slot_group"])
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "slot-dependent object-dictionary entry index",
        ):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_preserves_entry_fixed_scaling_swap_and_safety_metadata(self):
        annotated = ESI.replace(
            '<Entry><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Controlword</Name><DataType>UINT</DataType></Entry>',
            '<Entry Fixed="true" SafetyConnNumber="7"><Index>#x6040</Index><SubIndex>0</SubIndex><BitLen>16</BitLen><Name>Controlword</Name><DataType DScale="+/-10" SwapData="Swap_HB_LB">UINT</DataType></Entry>',
            1,
        )
        self.path.write_text(annotated, encoding="utf-8")
        entry = esi_to_profile.parse_esi(self.path)["devices"][0]["pdos"][0]["entries"][0]
        self.assertTrue(entry["fixed"])
        self.assertEqual(entry["safety_connection_number"], 7)
        self.assertEqual(entry["data_scale"], "+/-10")
        self.assertEqual(entry["swap_data"], "Swap_HB_LB")

    def test_rejects_slot_dependent_pdo_indices_in_static_candidate(self):
        dynamic = ESI_WITH_VELOCITY.replace(
            "<Index>#x1601</Index>",
            '<Index DependOnSlot="true">#x1601</Index>',
            1,
        )
        self.path.write_text(dynamic, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        velocity_pdo = next(pdo for pdo in device["pdos"] if pdo["index"] == 0x1601)
        self.assertTrue(velocity_pdo["index_depend_on_slot"])
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "slot-dependent PDO indices",
        ):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_rejects_selected_pdo_marked_overwritten_by_module(self):
        overwritten = ESI_WITH_VELOCITY.replace(
            '<RxPdo Sm="2" Su="5"><Index>#x1601</Index>',
            '<RxPdo Sm="2" Su="5" OverwrittenByModule="true"><Index>#x1601</Index>',
            1,
        )
        self.path.write_text(overwritten, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        velocity_pdo = next(pdo for pdo in device["pdos"] if pdo["index"] == 0x1601)
        self.assertTrue(velocity_pdo["overwritten_by_module"])
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "OverwrittenByModule and is not an effective device PDO",
        ):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_rejects_oversampling_pdo_in_static_candidate(self):
        oversampled = ESI_WITH_VELOCITY.replace(
            '<RxPdo Sm="2" Su="5"><Index>#x1601</Index>',
            '<RxPdo Sm="2" Su="5" OSFac="2"><Index>#x1601</Index>',
            1,
        )
        self.path.write_text(oversampled, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "oversampling metadata that the static CiA 402 candidate resolver does not expand",
        ):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_applies_sync_manager_range_to_safety_parameter_mapping(self):
        malformed = ESI.replace(
            "</EtherCATInfo>",
            """  <Modules><Module><Type ModuleIdent="#x4711">Safety module</Type>
      <Name>Safety module</Name>
      <SafetyParaMapping Sm="256"><Index>#x8000</Index><Name>Safety map</Name></SafetyParaMapping>
    </Module></Modules>
</EtherCATInfo>""",
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "Module.SafetyParaMapping.Sm value 256 is outside 0..255",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_fsoe_or_sra_pdo_as_generic_cia402_mapping(self):
        safety = ESI_WITH_VELOCITY.replace(
            '<RxPdo Sm="2" Su="5"><Index>#x1601</Index>',
            '<RxPdo Sm="2" Su="5" SafetyPdoType="FSoESlave"><Index>#x1601</Index>',
            1,
        )
        self.path.write_text(safety, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "FSoE/SRA safety metadata",
        ):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_rejects_selected_mutually_exclusive_pdos(self):
        incompatible = ESI_WITH_VELOCITY.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">\n          <Exclude>#x1601</Exclude>',
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
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">\n          <Exclude DependOnSlot="true">#x1601</Exclude>',
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

    def test_rejects_excluded_sync_manager_outside_declared_inventory(self):
        malformed = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">\\n          <ExcludedSm>4</ExcludedSm>',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "excludes SyncManager 4, but the device declares only 4",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_mandatory_pdo_without_default_sync_manager(self):
        malformed = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Fixed="true" Mandatory="true">',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "is mandatory but has no default SyncManager assignment",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_requires_all_mandatory_pdos_in_selected_assignment(self):
        with_mandatory_aux = ESI_WITH_VELOCITY.replace(
            "      </Device>",
            """        <RxPdo Sm="2" Mandatory="true"><Index>#x1604</Index>
          <Name>Mandatory auxiliary data</Name>
          <Entry><Index>#x2000</Index><SubIndex>1</SubIndex><BitLen>16</BitLen><Name>Aux</Name><DataType>UINT</DataType></Entry>
        </RxPdo>
      </Device>""",
            1,
        )
        self.path.write_text(with_mandatory_aux, encoding="utf-8")
        device = esi_to_profile.parse_esi(self.path)["devices"][0]
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "mandatory RXPDO 0x1604 is missing from the selected assignment",
        ):
            esi_to_profile.resolve_cia402_candidate(
                device, [0x1600, 0x1601], [0x1A00, 0x1A01]
            )

    def test_rejects_exclusion_reference_to_opposite_direction_pdo(self):
        malformed = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">\\n          <Exclude>#x1A00</Exclude>',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "rxPDO 0x1600 excludes PDO index 0x1A00 outside the allowed RXPDO range",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_hex_lexical_form_for_xs_int_excluded_sm(self):
        malformed = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">\\n          <ExcludedSm>#x3</ExcludedSm>',
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "invalid rxPDO.ExcludedSm",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_default_sync_manager_that_is_explicitly_excluded(self):
        malformed = ESI.replace(
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">',
            '<RxPdo Sm="2" Fixed="true" Mandatory="true">\n          <ExcludedSm>2</ExcludedSm>',
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
        self.assertEqual(result["bindings"]["target_velocity"]["sync_unit"], 5)
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

    def test_rejects_rxpdo_index_in_txpdo_range(self):
        malformed = ESI.replace(
            "<Index>#x1600</Index>",
            "<Index>#x1A00</Index>",
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "rxPDO index 0x1A00 is outside the allowed range 0x1600..0x17FF",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_rejects_txpdo_index_in_rxpdo_range(self):
        malformed = ESI.replace(
            "<Index>#x1A00</Index>",
            "<Index>#x1600</Index>",
            1,
        )
        self.path.write_text(malformed, encoding="utf-8")
        with self.assertRaisesRegex(
            esi_to_profile.EsiError,
            "txPDO index 0x1600 is outside the allowed range 0x1A00..0x1BFF",
        ):
            esi_to_profile.parse_esi(self.path)

    def test_accepts_padding_entry_without_subindex(self):
        valid = ESI.replace(
            '<Entry><Index>#x0000</Index><SubIndex>0</SubIndex>',
            '<Entry><Index>#x0000</Index>',
            1,
        )
        self.path.write_text(valid, encoding="utf-8")
        padding = esi_to_profile.parse_esi(self.path)["devices"][0]["pdos"][0]["entries"][2]
        self.assertTrue(padding["is_padding"])
        self.assertEqual(padding["sub_index"], 0)

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
            '<RxPdo Sm="2" Su="5"><Index>#x1601</Index>',
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
