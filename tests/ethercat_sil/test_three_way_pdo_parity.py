import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "ethercat_sil"))
from three_way_pdo_parity import ThreeWayParityError, consolidate_reports

def reports():
    return [
        {"schema_version": 1, "source_kind": "esi_coe_parity", "match": True,
         "slaves": [{"ring_position": 0, "match": True}, {"ring_position": 1, "match": True}]},
        {"schema_version": 1, "source_kind": "esi_igh_parity", "match": True,
         "slaves": [{"ring_position": 0, "match": True}, {"ring_position": 1, "match": True}]},
        {"schema_version": 1, "source_kind": "coe_igh_parity", "match": True,
         "slaves": [{"ring_position": 0, "match": True}, {"ring_position": 1, "match": True}]},
    ]

class ThreeWayParityTests(unittest.TestCase):
    def test_requires_all_three_reports(self):
        result = consolidate_reports(*reports(), 2)
        self.assertTrue(result["match"])
        self.assertEqual([x["comparison"] for x in result["checks"]], ["esi_coe", "esi_igh", "coe_igh"])
    def test_any_pairwise_failure_fails_gate(self):
        values = reports()
        values[1]["slaves"][1]["match"] = False
        result = consolidate_reports(*values, 2)
        self.assertFalse(result["match"])
        self.assertEqual(result["checks"][1]["mismatch_count"], 1)
    def test_rejects_missing_slave_result(self):
        values = reports()
        values[2]["slaves"].pop()
        with self.assertRaisesRegex(ThreeWayParityError, "exactly 2"):
            consolidate_reports(*values, 2)
    def test_rejects_different_ring_coverage(self):
        values = reports()
        values[2]["slaves"][1]["ring_position"] = 3
        with self.assertRaisesRegex(ThreeWayParityError, "ring positions"):
            consolidate_reports(*values, 2)
    def test_rejects_wrong_report_kind(self):
        values = reports()
        values[0]["source_kind"] = "coe_igh_parity"
        with self.assertRaisesRegex(ThreeWayParityError, "source_kind"):
            consolidate_reports(*values, 2)

if __name__ == "__main__":
    unittest.main()
