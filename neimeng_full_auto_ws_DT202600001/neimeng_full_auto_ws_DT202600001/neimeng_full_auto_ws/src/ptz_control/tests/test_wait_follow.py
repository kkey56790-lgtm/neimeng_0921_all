#!/usr/bin/env python3
"""Exercise the real controller with ROS/ONVIF replaced by local fakes."""
import importlib.util
import sys
import threading
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

import yaml

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src/ptz_control/scripts"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class WaitFollowTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / "src/neimeng_bringup/config/ptz_points.yaml").read_text(encoding="utf-8"))
        ros = types.ModuleType("rospy")
        ros.get_param = lambda name, default=None: self.config.get(name.lstrip("~"), default)
        ros.Subscriber = Mock()
        ros.loginfo = ros.logwarn = ros.logerr = Mock()
        ros.is_shutdown = lambda: False
        msg = types.ModuleType("std_msgs.msg")
        msg.String = Mock()
        protocol = load("test_protocol", ROOT / "src/inspection_interfaces/src/inspection_interfaces/protocol.py")
        mocks = {"rospy": ros, "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": msg,
                 "onvif": types.SimpleNamespace(ONVIFCamera=Mock()),
                 "inspection_interfaces": types.ModuleType("inspection_interfaces"),
                 "inspection_interfaces.protocol": protocol}
        self.modules = patch.dict(sys.modules, mocks)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        base = load("ptz_controller", SCRIPTS / "ptz_controller.py")
        sys.modules["ptz_controller"] = base
        self.addCleanup(lambda: sys.modules.pop("ptz_controller", None))
        protected = load("test_protected", SCRIPTS / "protected_ptz_controller.py")
        def init(obj):
            obj.condition = threading.Condition()
            obj.pending = None
            obj.version = 0
            obj.mode = "auto"
            obj.driver = Mock()
            obj.device_state = "READY"
            obj._publish_result = Mock()
            obj._publish_health = Mock()
            obj._refresh_position = Mock()
            obj.preset_speed = 1.0
            obj.default_settle = 3.0
            obj._wait = lambda *args: True
        with patch.object(base.PTZController, "__init__", init):
            self.c = protected.ProtectedPTZController()

    def status(self, kind="TASK_WAIT", state="STATE_DOING"):
        self.c._task_status({"data": {"task_state": state, "task_type": kind}})

    def complete(self):
        command, self.c.pending = self.c.pending, None
        self.c._execute(command)
        with self.c.condition:
            self.c._continue_wait(command, True)
        return command

    def test_wait_runs_existing_preset_order_and_repeats(self):
        self.status()
        self.complete()
        self.complete()
        calls = self.c.driver.goto_preset.call_args_list
        self.assertEqual(["4", "5", "6", "4", "7", "8", "4"], [call.args[0] for call in calls])
        self.assertEqual("sweep", self.c.pending["action"])
        self.complete()
        self.assertEqual(["5", "6", "4", "7", "8", "4"],
                         [call.args[0] for call in self.c.driver.goto_preset.call_args_list[7:]])

    def test_repeated_status_does_not_restart(self):
        self.status()
        command = self.c.pending
        self.status()
        self.assertIs(command, self.c.pending)
        self.complete()
        command = self.c.pending
        self.status()
        self.assertIs(command, self.c.pending)

    def test_wait_defers_point_then_resumes_preset_and_sweep(self):
        self.status()
        self.c._command({"action": "goto_preset", "point_name": "cruise", "data": {"preset": "4"}})
        self.c._command({"action": "sweep", "point_name": "cruise", "data": {"steps": []}})
        self.assertTrue(self.c.pending["_wait_owned"])
        self.assertEqual(2, len(self.c.wait_deferred))
        self.status("TASK_MOVE_TO")
        self.assertEqual("goto_preset", self.c.pending["action"])
        self.assertEqual(1, len(self.c.deferred_sweeps))

    def test_latest_point_replaces_old_deferred_point(self):
        self.status()
        for name in ("old", "new"):
            self.c._command({"action": "mode", "point_name": name, "data": {"mode": "auto"}})
        self.status("TASK_MOVE_TO")
        self.assertEqual([], self.c.wait_deferred)
        successes = [call.args[0].get("point_name") for call in self.c._publish_result.call_args_list if call.args[1] == "success"]
        self.assertIn("new", successes)
        self.assertNotIn("old", successes)

    def test_exit_stops_wait_and_stale_completion_cannot_restart(self):
        self.status()
        old = self.c.pending
        self.status("TASK_MOVE_TO")
        self.c._continue_wait(old, True)
        self.assertIsNone(self.c.pending)
        self.c.driver.stop.assert_called()

    def test_stop_suppresses_until_next_wait(self):
        self.status()
        self.c._command({"action": "stop"})
        self.status()
        self.assertIsNone(self.c.pending)
        self.status("TASK_MOVE_TO")
        self.status()
        self.assertEqual("goto_preset", self.c.pending["action"])

    def test_manual_mode_is_not_overridden(self):
        self.c.mode = "manual"
        self.status()
        self.assertIsNone(self.c.pending)
        self.assertEqual("manual", self.c.mode)

    def test_pause_finish_and_task_names_do_not_trigger(self):
        for state in ("STATE_PAUSE", "STATE_FINISH", "STATE_FAIL", "STATE_CANCEL"):
            self.status(state=state)
            self.assertIsNone(self.c.pending)
        self.c._task_status({"data": {"task_state": "STATE_DOING", "task_name": "等待任务", "task_type": "TASK_MOVE_TO"}})
        self.assertIsNone(self.c.pending)

    def test_point_protection_still_operates_outside_wait(self):
        self.c._command({"action": "goto_preset", "point_name": "a", "data": {"preset": "4"}})
        version = self.c.version
        self.c._command({"action": "sweep", "point_name": "a", "data": {"steps": []}})
        self.assertEqual(version, self.c.version)
        self.assertEqual(1, len(self.c.deferred_sweeps))

    def test_wait_preempts_existing_point_and_preserves_its_sweep(self):
        self.c._command({"action": "goto_preset", "point_name": "a", "data": {"preset": "4"}})
        self.c._command({"action": "sweep", "point_name": "a", "data": {"steps": []}})
        old_version = self.c.version
        self.status()
        self.assertGreater(self.c.version, old_version)
        self.assertEqual(2, len(self.c.wait_deferred))
        self.assertEqual(0, len(self.c.deferred_sweeps))
        self.status("TASK_MOVE_TO")
        self.assertEqual("a", self.c.pending["point_name"])
        self.assertEqual(1, len(self.c.deferred_sweeps))

    def test_failure_does_not_automatically_repeat(self):
        self.status()
        command, self.c.pending = self.c.pending, None
        self.c._continue_wait(command, False)
        self.status()
        self.assertIsNone(self.c.pending)

    def test_launches_load_point_config_into_controller(self):
        for filename in ("full_auto_test.launch", "real.launch"):
            launch = ET.parse(ROOT / "src/neimeng_bringup/launch" / filename)
            node = next(item for item in launch.iter("node") if item.get("name") == "ptz_controller")
            self.assertTrue(any("ptz_points" in item.get("file", "") for item in node.findall("rosparam")))

    def test_source_and_independent_bundle_match(self):
        independent = ROOT / "independent_systems/ptz_follow_system"
        for name in ("ptz_controller.py", "protected_ptz_controller.py"):
            self.assertEqual((SCRIPTS / name).read_bytes(), (independent / "scripts" / name).read_bytes())
        self.assertEqual(self.config, yaml.safe_load((independent / "config/ptz_points.yaml").read_text(encoding="utf-8")))


if __name__ == "__main__":
    unittest.main()

