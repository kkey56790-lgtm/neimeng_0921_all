#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from real_flow_rules import (  # noqa: E402
    TaskCatalogResolver, available_task_numbers, extract_battery_percent,
    extract_obstacle_state, next_task_number, start_point_number,
    task_area_prefix, task_number,
)


class RealFlowRuleTests(unittest.TestCase):
    def test_number_keywords_do_not_confuse_task1_and_task10(self):
        self.assertEqual(1, task_number("东区巡检任务1"))
        self.assertEqual(10, task_number("北区任务10"))
        self.assertEqual(2, start_point_number("北区起始点02"))

    def test_area_prefix_selects_matching_real_task(self):
        resolver = TaskCatalogResolver(10)
        resolver.update(["东区任务1", "北区任务1", "东区任务2"])
        self.assertEqual("东区任务1", resolver.resolve_number(1, preferred_area="东区"))
        self.assertEqual("北区任务1", resolver.resolve_number(1, requested_name="北区任务1"))
        self.assertEqual("东区", task_area_prefix("东区任务1"))

    def test_ambiguous_area_requires_full_name(self):
        resolver = TaskCatalogResolver(10)
        resolver.update(["东区任务4", "北区任务4"])
        with self.assertRaises(ValueError):
            resolver.resolve_number(4)

    def test_current_map_task_cycle_wraps_after_last_real_task(self):
        two_tasks = ["西任务1", "西任务2"]
        self.assertEqual([1, 2], available_task_numbers(two_tasks, 10))
        self.assertEqual(2, next_task_number(1, two_tasks, 10))
        self.assertEqual(1, next_task_number(2, two_tasks, 10))

        three_tasks = ["西任务1", "西任务2", "西任务3"]
        self.assertEqual(2, next_task_number(1, three_tasks, 10))
        self.assertEqual(3, next_task_number(2, three_tasks, 10))
        self.assertEqual(1, next_task_number(3, three_tasks, 10))

    def test_current_map_task_cycle_skips_missing_numbers(self):
        tasks = ["西任务1", "西任务3"]
        self.assertEqual(3, next_task_number(1, tasks, 10))
        self.assertEqual(1, next_task_number(3, tasks, 10))
        self.assertEqual(2, next_task_number(2, ["西任务2"], 10))
    def test_recharge_keyword_and_telemetry(self):
        resolver = TaskCatalogResolver(10)
        resolver.update(["东区自动回充", "东区任务1"])
        self.assertEqual("东区自动回充", resolver.resolve_keywords(["回充"], "东区"))
        telemetry = {"sensor": {"battery": {"soc": 29.5}, "obstacle_state": "有障碍"}}
        self.assertEqual(29.5, extract_battery_percent(telemetry))
        self.assertTrue(extract_obstacle_state(telemetry))


if __name__ == "__main__":
    unittest.main()
