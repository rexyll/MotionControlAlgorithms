import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "ethercat_sil"))
from esi_coe_parity import ParityError, compare_snapshots


def snapshots():
    esi = {
        "schema_version": 1, "source_kind": "esi_expected",
        "runtime_readback_performed": False,
        "identity": {"vendor_id": 0x29C, "product_code": 0x03B11002},
        "assignments": [
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600},
            {"assignment_object": 0x1C13, "assignment_position": 1, "pdo_index": 0x1A00},
        ],
        "mappings": [
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600,
             "sync_manager": 2, "direction": "rx", "entry_position": 1, "raw_mapping_entry": 0x60400010},
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600,
             "sync_manager": 2, "direction": "rx", "entry_position": 2, "raw_mapping_entry": 0x00000008},
            {"assignment_object": 0x1C13, "assignment_position": 1, "pdo_index": 0x1A00,
             "sync_manager": 3, "direction": "tx", "entry_position": 1, "raw_mapping_entry": 0x60410010},
        ],
    }
    coe_slave = {
        "ring_position": 0, "vendor_id": 0x29C, "product_code": 0x03B11002,
        "assignments": [dict(item) for item in esi["assignments"]],
        "mappings": [
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600,
             "entry_position": 1, "raw_mapping_entry": 0x60400010},
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600,
             "entry_position": 2, "raw_mapping_entry": 0x00000008},
            {"assignment_object": 0x1C13, "assignment_position": 1, "pdo_index": 0x1A00,
             "entry_position": 1, "raw_mapping_entry": 0x60410010},
        ],
    }
    coe = {
        "schema_version": 1, "source_kind": "coe_sdo_readback",
        "runtime_readback_performed": True, "slaves": [coe_slave],
    }
    return esi, coe


class EsiCoeParityTests(unittest.TestCase):
    def test_matches_identity_assignment_and_raw_mapping_order(self):
        esi, coe = snapshots()
        result = compare_snapshots(esi, coe)
        self.assertTrue(result["match"])
        self.assertTrue(result["slaves"][0]["match"])
        self.assertIn("SyncManager", result["limitations"][0])

    def test_rejects_assignment_order_mismatch(self):
        esi, coe = snapshots()
        coe["slaves"][0]["assignments"].reverse()
        result = compare_snapshots(esi, coe)
        self.assertFalse(result["match"])
        self.assertEqual(result["slaves"][0]["mismatches"][0]["layer"], "assignment")

    def test_rejects_raw_mapping_mismatch(self):
        esi, coe = snapshots()
        coe["slaves"][0]["mappings"][0]["raw_mapping_entry"] = 0x60400020
        result = compare_snapshots(esi, coe)
        self.assertFalse(result["match"])
        self.assertEqual(result["slaves"][0]["mismatches"][0]["layer"], "mapping")

    def test_rejects_identity_mismatch(self):
        esi, coe = snapshots()
        coe["slaves"][0]["product_code"] += 1
        result = compare_snapshots(esi, coe)
        self.assertFalse(result["match"])
        self.assertEqual(result["slaves"][0]["mismatches"][0]["layer"], "identity")

    def test_rejects_wrong_snapshot_provenance(self):
        esi, coe = snapshots()
        esi["runtime_readback_performed"] = True
        with self.assertRaisesRegex(ParityError, "must not claim runtime readback"):
            compare_snapshots(esi, coe)


if __name__ == "__main__":
    unittest.main()
