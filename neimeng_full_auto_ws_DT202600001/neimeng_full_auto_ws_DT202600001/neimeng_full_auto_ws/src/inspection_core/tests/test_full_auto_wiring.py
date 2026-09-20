#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


SRC = Path(__file__).resolve().parents[2]


class FullAutoWiringTests(unittest.TestCase):
    def test_no_mqtt_and_current_map_entries(self):
        launch = SRC / "neimeng_bringup" / "launch" / "full_auto_test.launch"
        nodes = {node.attrib.get("name"): node.attrib.get("type") for node in ET.parse(str(launch)).iter("node")}
        self.assertEqual("safe_robot_bridge.py", nodes.get("robot_bridge"))
        self.assertEqual("full_auto_yolo_bridge.py", nodes.get("yolo_bridge"))
        self.assertEqual("map_scoped_full_auto_orchestrator.py", nodes.get("inspection_orchestrator"))
        self.assertEqual("full_auto_map_test_panel.py", nodes.get("full_auto_test_panel"))
        self.assertNotIn("mqtt_gateway", nodes)

    def test_full_auto_uses_separate_module_configs(self):
        launch = (SRC / "neimeng_bringup" / "launch" / "full_auto_test.launch").read_text(encoding="utf-8")
        for config_name in ("auto_drive.yaml", "ptz_points.yaml", "yolo_flow.yaml"):
            self.assertIn(config_name, launch)
        self.assertNotIn("inspection.yaml", launch)
        self.assertNotIn("full_auto_flow.yaml", launch)

    def test_yolo_bridge_is_result_only_and_matches_truck_by_name(self):
        bridge = (SRC / "inspection_core" / "scripts" / "full_auto_yolo_bridge.py").read_text(encoding="utf-8")
        self.assertNotIn("detection.frame", bridge)
        self.assertNotIn("frame_pub", bridge)
        config = (SRC / "neimeng_bringup" / "config" / "full_auto_yolo.yaml").read_text(encoding="utf-8")
        self.assertIn('target_class: "truck"', config)
        self.assertIn("target_class_id: -1", config)

    def test_ptz_fixed_point_action_only_runs_after_arrival(self):
        orchestrator = (SRC / "inspection_core" / "scripts" / "inspection_orchestrator.py").read_text(encoding="utf-8")
        approach = orchestrator.split('if event == "approach":', 1)[1].split('elif event == "arrived":', 1)[0]
        arrived = orchestrator.split('elif event == "arrived":', 1)[1].split('elif event == "leave":', 1)[0]
        self.assertNotIn("_send_ptz", approach)
        self.assertIn("_begin_point()", arrived)
        self.assertIn("_start_point_detection", arrived)
        protected = (SRC / "ptz_control" / "scripts" / "protected_ptz_controller.py").read_text(encoding="utf-8")
        self.assertIn("deferred_sweeps", protected)
        self.assertIn("superseded_by_new_ptz_intent", protected)
    def test_truck_and_non_truck_collection_are_separate(self):
        bridge = (SRC / "inspection_core" / "scripts" / "gated_yolo_bridge.py").read_text(encoding="utf-8")
        self.assertIn('collection_mode == "inspection"', bridge)
        self.assertIn("wanted = not is_truck", bridge)

        orchestrator = (SRC / "inspection_core" / "scripts" / "real_inspection_orchestrator.py").read_text(encoding="utf-8")
        self.assertIn('collection_mode = "truck" if truck_mode else "inspection"', orchestrator)
        self.assertIn('"decision": {"action": "record_only"}', orchestrator)
        self.assertIn("NON_TRUCK_INSPECTION_RESULT", orchestrator)

        config = (SRC / "neimeng_bringup" / "config" / "yolo_flow.yaml").read_text(encoding="utf-8")
        self.assertIn("collect_non_truck_at_other_points: true", config)
    def test_ui_refresh_is_lightweight(self):
        panel = (SRC / "inspection_ui" / "scripts" / "full_auto_test_panel.py").read_text(encoding="utf-8")
        self.assertIn("deque(maxlen=100)", panel)
        self.assertIn("self.timer.start(500)", panel)
        self.assertIn("QHeaderView.Interactive", panel)
        self.assertIn("self.timeline_table.insertRow(0)", panel)
        self.assertNotIn("QHeaderView.ResizeToContents", panel)
    def test_map_filter_is_fail_closed(self):
        orchestrator = (SRC / "inspection_core" / "scripts" / "map_scoped_full_auto_orchestrator.py").read_text(encoding="utf-8")
        self.assertIn('data["all_tasks"] = list(current_tasks)', orchestrator)
        panel = (SRC / "inspection_ui" / "scripts" / "full_auto_map_test_panel.py").read_text(encoding="utf-8")
        self.assertIn("当前地图全部真实任务", panel)
        base_panel = (SRC / "inspection_ui" / "scripts" / "full_auto_test_panel.py").read_text(encoding="utf-8")
        self.assertIn('tasks = data.get("tasks", [])', base_panel)
        self.assertIn("全流程时间线（最新在上）", base_panel)
        self.assertIn("OBSTACLE_TIMER_STARTED", base_panel)
        self.assertIn("RECHARGE_COMPLETED", base_panel)
        self.assertNotIn("模拟中台下发任务", base_panel)

        production_panel = (
            SRC / "inspection_ui" / "scripts" / "production_inspection_panel.py"
        ).read_text(encoding="utf-8")
        self.assertIn('source != "robot_bridge"', production_panel)


if __name__ == "__main__":
    unittest.main()
