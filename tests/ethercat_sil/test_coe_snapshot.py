import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "ethercat_sil"))
from coe_snapshot import SnapshotError, parse_rows


def fixture():
    rows = [
        "S\t0\t0x29c\t0x03b11002",
        "C\t0\t0x1c12\t1",
        "C\t0\t0x1c13\t1",
        "A\t0\t0x1c12\t1\t0x1600",
        "A\t0\t0x1c13\t1\t0x1a00",
        "P\t0\t0x1c12\t1\t0x1600\t2",
        "P\t0\t0x1c13\t1\t0x1a00\t1",
        "M\t0\t0x1c12\t1\t0x1600\t1\t0x60400010",
        "M\t0\t0x1c12\t1\t0x1600\t2\t0x00000008",
        "M\t0\t0x1c13\t1\t0x1a00\t1\t0x60410010",
    ]
    return "\n".join(rows) + "\n"


class CoeSnapshotTests(unittest.TestCase):
    def test_preserves_assignment_and_entry_order_and_padding(self):
        snapshot = parse_rows(fixture())
        self.assertEqual(snapshot["source_kind"], "coe_sdo_readback")
        self.assertTrue(snapshot["runtime_readback_performed"])
        slave = snapshot["slaves"][0]
        self.assertEqual([a["pdo_index"] for a in slave["assignments"]], [0x1600, 0x1A00])
        self.assertEqual([m["raw_mapping_entry"] for m in slave["mappings"][:2]],
                         [0x60400010, 0x00000008])

    def test_rejects_assignment_gap_or_reorder(self):
        rows = fixture().replace("A\t0\t0x1c13\t1", "A\t0\t0x1c13\t2")
        with self.assertRaisesRegex(SnapshotError, "assignment order/gap"):
            parse_rows(rows)

    def test_rejects_mapping_entry_gap(self):
        rows = fixture().replace("0x1600\t2\t0x00000008", "0x1600\t3\t0x00000008")
        with self.assertRaisesRegex(SnapshotError, "entry order/gap"):
            parse_rows(rows)

    def test_rejects_mapping_for_unassigned_pdo(self):
        rows = fixture().replace("0x1c12\t1\t0x1600\t1", "0x1c12\t1\t0x1601\t1")
        with self.assertRaisesRegex(SnapshotError, "unknown assignment/PDO"):
            parse_rows(rows)

    def test_rejects_duplicate_real_mapping_object(self):
        rows = fixture().replace("0x60410010", "0x60400010")
        with self.assertRaisesRegex(SnapshotError, "duplicate mapped object"):
            parse_rows(rows)

    def test_rejects_malformed_padding(self):
        rows = fixture().replace("0x00000008", "0x00000108")
        with self.assertRaisesRegex(SnapshotError, "malformed mapping entry/padding"):
            parse_rows(rows)

    def test_rejects_interleaved_pdo_entry_groups(self):
        rows = fixture().replace(
            "M\t0\t0x1c12\t1\t0x1600\t2\t0x00000008\n"
            "M\t0\t0x1c13\t1\t0x1a00\t1\t0x60410010",
            "M\t0\t0x1c13\t1\t0x1a00\t1\t0x60410010\n"
            "M\t0\t0x1c12\t1\t0x1600\t2\t0x00000008",
        )
        with self.assertRaisesRegex(SnapshotError, "not contiguous"):
            parse_rows(rows)

    def test_rejects_non_contiguous_ring_positions(self):
        rows = fixture().replace("S\t0\t", "S\t1\t").replace("\t0\t0x1c", "\t1\t0x1c").replace(
            "M\t0\t", "M\t1\t"
        )
        with self.assertRaisesRegex(SnapshotError, "contiguous from zero"):
            parse_rows(rows)

    def test_rejects_assignment_cardinality_mismatch(self):
        rows = fixture().replace("C\t0\t0x1c12\t1", "C\t0\t0x1c12\t2")
        with self.assertRaisesRegex(SnapshotError, "assignment cardinality mismatch"):
            parse_rows(rows)

    def test_rejects_mapping_cardinality_mismatch(self):
        rows = fixture()
        rows = rows.replace("P\t0\t0x1c12\t1\t0x1600\t2",
                            "P\t0\t0x1c12\t1\t0x1600\t3")
        with self.assertRaisesRegex(SnapshotError, "cardinality mismatch"):
            parse_rows(rows)

    def test_rejects_missing_slave_identity(self):
        with self.assertRaisesRegex(SnapshotError, "no slave identity"):
            parse_rows("")


if __name__ == "__main__":
    unittest.main()
