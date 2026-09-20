#!/usr/bin/env python3
"""Robot bridge extension with bounded, acknowledged reverse avoidance."""

import importlib.util
import os
import time

import rospy

from inspection_interfaces.protocol import parse_message
def _load_robot_bridge_class():
    """Load source directly instead of importing Catkin's executable relay."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_bridge.py")
    spec = importlib.util.spec_from_file_location("robot_base_bridge_source", path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load robot bridge source from " + path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RobotBridge


RobotBridge = _load_robot_bridge_class()


class SafeRobotBridge(RobotBridge):
    def __init__(self):
        self.max_reverse_duration = max(0.1, float(rospy.get_param("~max_reverse_duration", 5.0)))
        super().__init__()

    def _robot_command(self, raw):
        try:
            message = parse_message(raw)
            action = str(message.get("action", message.get("method", ""))).lower()
            normalized = action[6:] if action.startswith("robot.") else action
            if normalized != "reverse":
                return super()._robot_command(message)
            command_id = str(message.get("command_id", message.get("tid", message.get("message_id", ""))))
            data = message.get("data", {})
            speed = min(self.max_linear_speed, abs(float(data.get("speed", 0.15))))
            duration = min(self.max_reverse_duration, max(0.1, float(data.get("duration", 2.0))))
            if speed <= 0:
                raise ValueError("reverse speed must be greater than zero")
            self._cancel_move_timer()
            task_result = {}
            reverse_result = {}
            stop_result = {}
            with self.api_lock:
                task_result = self.api.stop_task()
                reverse_result = self.api.move(-speed, 0.0)
                try:
                    time.sleep(duration)
                finally:
                    stop_result = self.api.move(0.0, 0.0)
            ok = all(bool(item.get("result", False)) for item in (
                task_result, reverse_result, stop_result))
            self._publish_robot_result(
                "reverse", command_id, "success" if ok else "failed",
                {"speed": speed, "duration": duration, "task_stop": task_result,
                 "reverse": reverse_result, "motion_stop": stop_result},
            )
        except Exception as exc:
            rospy.logerr("安全倒车命令失败: %s", exc)
            try:
                with self.api_lock:
                    self.api.move(0.0, 0.0)
            except Exception:
                pass
            command_id = str(message.get("command_id", "")) if "message" in locals() else ""
            self._publish_robot_result("reverse", command_id, "failed", {"error": str(exc)})


if __name__ == "__main__":
    rospy.init_node("robot_bridge")
    SafeRobotBridge().run()
