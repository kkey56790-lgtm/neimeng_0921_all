#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SYSTEMS = ROOT / "independent_systems"


class IndependentSystemWiringTests(unittest.TestCase):
    def test_task_switch_is_ptz_independent_and_yolo_optional(self):
        launch = ET.parse(str(SYSTEMS / "task_switch_system" / "launch" / "task_switch.launch"))
        nodes = {node.attrib.get("name"): node.attrib.get("type") for node in launch.iter("node")}
        self.assertEqual("safe_robot_bridge.py", nodes.get("robot_bridge"))
        self.assertEqual("task_switch_only_orchestrator.py", nodes.get("inspection_orchestrator"))
        params = {node.attrib.get("name"): node.attrib.get("value") for node in launch.iter("param")}
        self.assertEqual("$(arg enable_yolo)", params.get("detection_collection_enabled"))
        source = (SYSTEMS / "task_switch_system" / "scripts" / "task_switch_only_orchestrator.py").read_text(encoding="utf-8")
        self.assertNotIn("_send_ptz", source)
        self.assertIn("point_ptz_ready = True", source)

    def test_ptz_follow_has_real_point_adapter_and_optional_bridge(self):
        launch = ET.parse(str(SYSTEMS / "ptz_follow_system" / "launch" / "ptz_follow.launch"))
        args = {item.attrib.get("name"): item.attrib.get("default") for item in launch.iter("arg")}
        self.assertEqual("true", args.get("enable_robot_bridge"))
        nodes = {node.attrib.get("name"): node.attrib.get("type") for node in launch.iter("node")}
        self.assertEqual("ptz_follow_adapter.py", nodes.get("ptz_follow_adapter"))
        self.assertEqual("full_auto_ptz_controller.py", nodes.get("ptz_controller"))
        adapter = (SYSTEMS / "ptz_follow_system" / "scripts" / "ptz_follow_adapter.py").read_text(encoding="utf-8")
        self.assertIn("Topics.POINT_EVENT", adapter)
        self.assertIn('"goto_preset"', adapter)
        self.assertIn('"sweep"', adapter)

    def test_yolo_bundle_has_model_bridge_and_manual_trigger(self):
        model = SYSTEMS / "yolo_upload_system" / "detector" / "models" / "best_truck.rknn"
        self.assertTrue(model.is_file())
        self.assertGreater(model.stat().st_size, 1_000_000)
        launch = ET.parse(str(SYSTEMS / "yolo_upload_system" / "launch" / "yolo_upload.launch"))
        nodes = {node.attrib.get("name"): node.attrib.get("type") for node in launch.iter("node")}
        self.assertEqual("full_auto_yolo_bridge.py", nodes.get("yolo_bridge"))
        run = (SYSTEMS / "yolo_upload_system" / "run.sh").read_text(encoding="utf-8")
        self.assertIn("conda activate rknn_yolo", run)
        self.assertIn("detector/models/yolov8n.rknn", run)
        self.assertIn("ROS YOLO桥启动失败", run)

    def test_full_system_wraps_original_launcher(self):
        run = (SYSTEMS / "full_system" / "run.sh").read_text(encoding="utf-8")
        self.assertIn('run_ubuntu20_all.sh', run)
        self.assertNotIn("roslaunch", run)


if __name__ == "__main__":
    unittest.main()
