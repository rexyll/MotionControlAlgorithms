import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "ethercat_sil"))
from esi_igh_parity import IghParityError, compare_snapshots


def snapshots():
    esi = {
        "schema_version": 1, "source_kind": "esi_expected",
        "runtime_readback_performed": False,
        "identity": {"vendor_id": 0x29C, "product_code": 0x03B11002},
        "assignments": [], "mappings": [
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600,
             "sync_manager": 2, "direction": "rx", "entry_position": 1, "raw_mapping_entry": 0x60400010},
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600,
             "sync_manager": 2, "direction": "rx", "entry_position": 2, "raw_mapping_entry": 0x00000008},
            {"assignment_object": 0x1C13, "assignment_position": 1, "pdo_index": 0x1A00,
             "sync_manager": 3, "direction": "tx", "entry_position": 1, "raw_mapping_entry": 0x60410010},
        ],
    }
    entries = [
        {"sync_manager": 2, "direction": "output", "pdo_index": 0x1600,
         "entry_position": 1, "raw_mapping_entry": 0x60400010},
        {"sync_manager": 2, "direction": "output", "pdo_index": 0x1600,
         "entry_position": 2, "raw_mapping_entry": 0x00000008},
        {"sync_manager": 3, "direction": "input", "pdo_index": 0x1A00,
         "entry_position": 1, "raw_mapping_entry": 0x60410010},
    ]
    igh = {
        "schema_version": 1, "source_kind": "igh_discovery",
        "runtime_discovery_performed": True,
        "slaves": [
            {"ring_position": 0, "vendor_id": 0x29C, "product_code": 0x03B11002,
             "mappings": [dict(item) for item in entries]},
            {"ring_position": 1, "vendor_id": 0x29C, "product_code": 0x03B11002,
             "mappings": [dict(item) for item in entries]},
        ],
    }
    return esi, igh


class EsiIghParityTests(unittest.TestCase):
    def test_matches_sm_direction_pdo_entry_order_and_padding(self):
        esi, igh = snapshots()
        report = compare_snapshots(esi, igh)
        self.assertTrue(report["match"])
        self.assertEqual(len(report["slaves"]), 2)

    def test_rejects_sm_or_direction_mismatch(self):
        esi, igh = snapshots()
        igh["slaves"][0]["mappings"][0]["sync_manager"] = 3
        report = compare_snapshots(esi, igh)
        self.assertFalse(report["match"])

    def test_rejects_entry_order_mismatch(self):
        esi, igh = snapshots()
        igh["slaves"][1]["mappings"][0], igh["slaves"][1]["mappings"][1] = (
            igh["slaves"][1]["mappings"][1], igh["slaves"][1]["mappings"][0]
        )
        report = compare_snapshots(esi, igh)
        self.assertFalse(report["match"])

    def test_rejects_identity_mismatch(self):
        esi, igh = snapshots()
        igh["slaves"][0]["vendor_id"] += 1
        report = compare_snapshots(esi, igh)
        self.assertFalse(report["match"])

    def test_rejects_missing_runtime_provenance(self):
        esi, igh = snapshots()
        igh["runtime_discovery_performed"] = False
        with self.assertRaisesRegex(IghParityError, "runtime discovery"):
            compare_snapshots(esi, igh)


if __name__ == "__main__":
    unittest.main()
