import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "ethercat_sil"))
from igh_snapshot import IghSnapshotError, parse_igh_log


def fixture():
    record = {
        "schema_version": 1, "source_kind": "igh_discovery",
        "entries": [
            {"ring_position": 0, "vendor_id": 0x29C, "product_code": 0x03B11002,
             "sync_manager": 2, "direction": "output", "pdo_index": 0x1600,
             "entry_position": 1, "raw_mapping_entry": 0x60400010},
            {"ring_position": 0, "vendor_id": 0x29C, "product_code": 0x03B11002,
             "sync_manager": 2, "direction": "output", "pdo_index": 0x1600,
             "entry_position": 2, "raw_mapping_entry": 0x00000008},
            {"ring_position": 0, "vendor_id": 0x29C, "product_code": 0x03B11002,
             "sync_manager": 3, "direction": "input", "pdo_index": 0x1A00,
             "entry_position": 1, "raw_mapping_entry": 0x60410010},
        ],
    }
    return "diagnostic line\nMCA_IGH_PDO_SNAPSHOT=" + json.dumps(record) + "\n"


class IghSnapshotTests(unittest.TestCase):
    def test_extracts_ordered_sm_pdo_entry_records(self):
        snapshot = parse_igh_log(fixture())
        self.assertTrue(snapshot["runtime_discovery_performed"])
        self.assertEqual(len(snapshot["slaves"]), 1)
        self.assertEqual(len(snapshot["slaves"][0]["mappings"]), 3)
        self.assertEqual(snapshot["slaves"][0]["mappings"][1]["raw_mapping_entry"], 0x00000008)

    def test_rejects_missing_or_duplicate_snapshot_record(self):
        with self.assertRaisesRegex(IghSnapshotError, "found 0"):
            parse_igh_log("no snapshot")
        with self.assertRaisesRegex(IghSnapshotError, "found 2"):
            parse_igh_log(fixture() + fixture())

    def test_rejects_entry_position_gap(self):
        rows = json.loads(fixture().split("MCA_IGH_PDO_SNAPSHOT=")[1])
        rows["entries"][1]["entry_position"] = 3
        with self.assertRaisesRegex(IghSnapshotError, "entry position gap"):
            parse_igh_log("MCA_IGH_PDO_SNAPSHOT=" + json.dumps(rows))

    def test_rejects_duplicate_real_mapping_object(self):
        rows = json.loads(fixture().split("MCA_IGH_PDO_SNAPSHOT=")[1])
        rows["entries"][1]["raw_mapping_entry"] = 0x60400010
        with self.assertRaisesRegex(IghSnapshotError, "duplicate mapped object"):
            parse_igh_log("MCA_IGH_PDO_SNAPSHOT=" + json.dumps(rows))

    def test_rejects_sync_manager_direction_change(self):
        rows = json.loads(fixture().split("MCA_IGH_PDO_SNAPSHOT=")[1])
        rows["entries"][1]["direction"] = "input"
        with self.assertRaisesRegex(IghSnapshotError, "direction changed"):
            parse_igh_log("MCA_IGH_PDO_SNAPSHOT=" + json.dumps(rows))


if __name__ == "__main__":
    unittest.main()
