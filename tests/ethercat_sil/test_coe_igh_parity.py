import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "ethercat_sil"))
from coe_igh_parity import CoeIghParityError, compare_snapshots


def fixtures():
    coe = {"schema_version": 1, "source_kind": "coe_sdo_readback",
           "runtime_readback_performed": True, "slaves": [{
        "ring_position": 0, "vendor_id": 0x29C, "product_code": 0x03B11002,
        "assignments": [
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600},
            {"assignment_object": 0x1C13, "assignment_position": 1, "pdo_index": 0x1A00}],
        "mappings": [
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600, "entry_position": 1, "raw_mapping_entry": 0x60400010},
            {"assignment_object": 0x1C12, "assignment_position": 1, "pdo_index": 0x1600, "entry_position": 2, "raw_mapping_entry": 0x00000008},
            {"assignment_object": 0x1C13, "assignment_position": 1, "pdo_index": 0x1A00, "entry_position": 1, "raw_mapping_entry": 0x60410010}]}]}
    igh = {"schema_version": 1, "source_kind": "igh_discovery",
           "runtime_discovery_performed": True, "slaves": [{
        "ring_position": 0, "vendor_id": 0x29C, "product_code": 0x03B11002,
        "mappings": [
            {"sync_manager": 2, "direction": "output", "pdo_index": 0x1600, "entry_position": 1, "raw_mapping_entry": 0x60400010},
            {"sync_manager": 2, "direction": "output", "pdo_index": 0x1600, "entry_position": 2, "raw_mapping_entry": 0x00000008},
            {"sync_manager": 3, "direction": "input", "pdo_index": 0x1A00, "entry_position": 1, "raw_mapping_entry": 0x60410010}]}]}
    return coe, igh


class CoeIghParityTests(unittest.TestCase):
    def test_matches_identity_pdo_and_ordered_raw_entries(self):
        coe, igh = fixtures()
        result = compare_snapshots(coe, igh)
        self.assertTrue(result["match"])
        self.assertIn("never inferred", result["limitations"][2])

    def test_rejects_mapping_value_mismatch(self):
        coe, igh = fixtures()
        igh["slaves"][0]["mappings"][0]["raw_mapping_entry"] = 0x60400020
        result = compare_snapshots(coe, igh)
        self.assertFalse(result["match"])
        self.assertEqual(result["slaves"][0]["mismatches"][0]["layer"], "mapping_entries")

    def test_rejects_missing_pdo(self):
        coe, igh = fixtures()
        igh["slaves"][0]["mappings"] = igh["slaves"][0]["mappings"][:2]
        result = compare_snapshots(coe, igh)
        self.assertFalse(result["match"])
        self.assertEqual(result["slaves"][0]["mismatches"][0]["layer"], "pdo_set")

    def test_rejects_ring_position_mismatch(self):
        coe, igh = fixtures()
        extra = dict(igh["slaves"][0])
        extra["ring_position"] = 1
        igh["slaves"].append(extra)
        with self.assertRaisesRegex(CoeIghParityError, "ring-position sets differ"):
            compare_snapshots(coe, igh)

    def test_rejects_wrong_provenance(self):
        coe, igh = fixtures()
        coe["runtime_readback_performed"] = False
        with self.assertRaisesRegex(CoeIghParityError, "provenance"):
            compare_snapshots(coe, igh)


if __name__ == "__main__":
    unittest.main()
