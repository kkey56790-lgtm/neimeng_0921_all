#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


class RealNodeWiringTests(unittest.TestCase):
    EXPECTED_NODES = {
        "robot_bridge": "safe_robot_bridge.py",
        "yolo_bridge": "gated_yolo_bridge.py",
        "inspection_orchestrator": "production_inspection_orchestrator_v2.py",
        "mqtt_gateway": "scheduled_mqtt_gateway.py",
        "inspection_panel": "production_inspection_panel.py",
    }

    def test_real_and_hardware_launch_use_production_nodes(self):
        launch_dir = ROOT / "neimeng_bringup" / "launch"
        for filename in ("real.launch", "hardware_demo.launch"):
            tree = ET.parse(str(launch_dir / filename))
            nodes = {node.attrib.get("name"): node.attrib.get("type") for node in tree.iter("node")}
            for name, executable in self.EXPECTED_NODES.items():
                self.assertEqual(executable, nodes.get(name), "{} in {}".format(name, filename))

    def test_catkin_installs_every_production_executable(self):
        checks = {
            ROOT / "inspection_core" / "CMakeLists.txt": [
                "production_inspection_orchestrator_v2.py", "gated_yolo_bridge.py", "real_flow_rules.py"],
            ROOT / "robot_base_bridge" / "CMakeLists.txt": ["safe_robot_bridge.py"],
            ROOT / "mqtt_gateway" / "CMakeLists.txt": ["scheduled_mqtt_gateway.py"],
            ROOT / "inspection_ui" / "CMakeLists.txt": ["production_inspection_panel.py"],
        }
        for path, names in checks.items():
            text = path.read_text(encoding="utf-8")
            for name in names:
                self.assertIn(name, text, str(path))

    def test_real_flow_config_is_loaded(self):
        for filename in ("real.launch", "hardware_demo.launch"):
            text = (ROOT / "neimeng_bringup" / "launch" / filename).read_text(encoding="utf-8")
            self.assertIn("inspection_numbered_flow.yaml", text)
            self.assertIn("robot_safe.yaml", text)
            self.assertIn("yolo_gated.yaml", text)


if __name__ == "__main__":
    unittest.main()

