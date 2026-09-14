"""Unit tests for person class name resolution (no ROS / MPS)."""

from __future__ import annotations

import unittest

from person_class import resolve_person_class_ids


class ResolvePersonClassIdsTests(unittest.TestCase):
    def test_coco_person_at_id_zero(self) -> None:
        names = {0: "person", 1: "bicycle"}
        self.assertEqual(resolve_person_class_ids(names), [0])

    def test_person_at_id_one(self) -> None:
        names = {0: "head", 1: "person"}
        self.assertEqual(resolve_person_class_ids(names), [1])

    def test_no_person_class(self) -> None:
        self.assertEqual(resolve_person_class_ids({0: "head"}), [])
        self.assertEqual(resolve_person_class_ids({0: "pedestrian"}), [])
        self.assertEqual(resolve_person_class_ids({}), [])

    def test_string_keys(self) -> None:
        names = {"0": "head", "1": "person"}
        self.assertEqual(resolve_person_class_ids(names), [1])


if __name__ == "__main__":
    unittest.main()
