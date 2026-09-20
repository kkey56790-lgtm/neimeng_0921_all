#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


SRC = Path(__file__).resolve().parents[2]


class ProductionWiringTests(unittest.TestCase):
    def test_launch_and_catkin_wiring(self):
        shared = {
            "robot_bridge": "safe_robot_bridge.py",
            "yolo_bridge": "start_point_yolo_bridge.py",
            "mqtt_gateway": "scheduled_mqtt_gateway.py",
            "inspection_panel": "production_inspection_panel.py",
        }
        orchestrators = {
            "real.launch": "map_scoped_full_auto_orchestrator.py",
            "hardware_demo.launch": "production_inspection_orchestrator_final.py",
        }
        for filename in ("real.launch", "hardware_demo.launch"):
            expected = dict(shared)
            expected["inspection_orchestrator"] = orchestrators[filename]
            path = SRC / "neimeng_bringup" / "launch" / filename
            nodes = {
                node.attrib.get("name"): node.attrib.get("type")
                for node in ET.parse(str(path)).iter("node")
            }
            for name, executable in expected.items():
                self.assertEqual(executable, nodes.get(name), "{} in {}".format(name, filename))
            text = path.read_text(encoding="utf-8")
            for config in ("inspection_numbered_flow.yaml", "robot_safe.yaml", "yolo_gated.yaml"):
                self.assertIn(config, text)

            enable_mqtt = next(
                arg for arg in ET.parse(str(path)).iter("arg")
                if arg.attrib.get("name") == "enable_mqtt"
            )
            self.assertEqual("false", enable_mqtt.attrib.get("default"))

        cmake_checks = {
            SRC / "inspection_core" / "CMakeLists.txt": [
                "production_inspection_orchestrator_final.py", "start_point_yolo_bridge.py", "real_flow_rules.py"],
            SRC / "robot_base_bridge" / "CMakeLists.txt": ["safe_robot_bridge.py"],
            SRC / "inspection_ui" / "CMakeLists.txt": ["production_inspection_panel.py"],
        }
        for path, names in cmake_checks.items():
            text = path.read_text(encoding="utf-8")
            for name in names:
                self.assertIn(name, text, str(path))

    def test_catalog_helper_bypasses_catkin_executable_relay(self):
        bridge = (
            SRC / "robot_base_bridge" / "scripts" / "robot_bridge.py"
        ).read_text(encoding="utf-8")
        cmake = (
            SRC / "robot_base_bridge" / "CMakeLists.txt"
        ).read_text(encoding="utf-8")
        self.assertIn("spec_from_file_location", bridge)
        self.assertNotIn("from catalog_rules import", bridge)
        self.assertIn("install(FILES scripts/catalog_rules.py", cmake)
        install_python = cmake.split("install(FILES", 1)[0]
        self.assertNotIn("scripts/catalog_rules.py", install_python)


if __name__ == "__main__":
    unittest.main()
