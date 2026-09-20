#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path


ROBOT_SCRIPTS = Path(__file__).resolve().parents[2] / "robot_base_bridge" / "scripts"
sys.path.insert(0, str(ROBOT_SCRIPTS))

from catalog_rules import current_map_tasks, task_name  # noqa: E402


class CurrentMapCatalogTests(unittest.TestCase):
    @staticmethod
    def names(records):
        return [task_name(record) for record in records]

    def test_matches_top_level_and_nested_map_metadata(self):
        tasks = [
            {"name": "东区任务0", "map_name": "东区"},
            {"name": "东区任务1", "tasks": [{"task_type": "TASK_MOVE_TO", "map": {"name": "东区"}}]},
            {"name": "西区任务1", "map": "西区"},
        ]
        selected = current_map_tasks(tasks, {"name": "东区"})
        self.assertEqual(["东区任务0", "东区任务1"], self.names(selected))

    def test_matches_map_id_across_field_variants(self):
        tasks = [
            {"task_name": "任务0", "mapId": 7},
            {"task_name": "任务1", "map_id": 8},
        ]
        selected = current_map_tasks(tasks, {"name": "装卸区", "id": 7})
        self.assertEqual(["任务0"], self.names(selected))

    def test_unscoped_robot_response_is_already_current_map(self):
        tasks = [{"name": "任务0"}, {"name": "任务1"}, {"name": "返航充电"}]
        self.assertEqual(tasks, current_map_tasks(tasks, {"name": "东区"}))

    def test_mixed_catalog_excludes_unscoped_and_other_map_tasks(self):
        tasks = [
            {"name": "东区任务0", "map": "东区"},
            {"name": "未标地图任务"},
            {"name": "西区任务0", "map": "西区"},
        ]
        selected = current_map_tasks(tasks, {"name": "东区"})
        self.assertEqual(["东区任务0"], self.names(selected))

    def test_missing_current_map_fails_closed(self):
        self.assertEqual([], current_map_tasks([{"name": "任务0"}], {}))


if __name__ == "__main__":
    unittest.main()
