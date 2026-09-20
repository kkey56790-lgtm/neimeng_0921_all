#!/usr/bin/env python3
"""底盘 ACTION_WAIT_TIME 运行期间触发一次高优先级云台扫描。"""

import copy
import os
import sys
from collections import deque

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

import rospy
from std_msgs.msg import String
from inspection_interfaces.protocol import Topics, make_message, new_id, parse_message
from ptz_controller import PTZController


class WaitTaskPTZController(PTZController):
    def __init__(self):
        # 基类会启动工作线程，调度成员必须提前初始化。
        self.normal_queue = deque()
        self.wait_queue = deque()
        self.current_command = None
        self.wait_seen_key = None
        self.wait_config = rospy.get_param("~wait_scan", {})
        super().__init__()
        self.task_subscriber = rospy.Subscriber(
            Topics.TASK_STATUS, String, self._task_status, queue_size=30)
        rospy.loginfo("等待随动：ACTION_WAIT_TIME + STATE_DOING；巡航点触发关闭")

    @staticmethod
    def _field(data, path):
        for part in str(path).split("."):
            if not isinstance(data, dict):
                return None
            data = data.get(part)
        return data

    def _task_status(self, raw):
        try:
            message = parse_message(raw)
            if message.get("source") != "robot_bridge":
                return
            data = message.get("data", {})
            if not isinstance(data, dict) or "task_state" not in data:
                return
            if not self.wait_config.get("enabled", True):
                return
            state = str(data.get("task_state", "")).strip()
            arg = data.get("task_arg", {})
            arg = arg if isinstance(arg, dict) else {}
            task_type = str(data.get("task_type") or arg.get("task_type") or "").strip()
            with self.condition:
                if state in ("STATE_FINISH", "STATE_CANCEL", "STATE_CANCELED",
                             "STATE_FAIL", "STATE_FAILED", "STATE_IDLE"):
                    self.wait_seen_key = None
                    return
                # 暂停不启动扫描，也不清空去重记录。
                if state != "STATE_DOING" or not task_type:
                    return
                if task_type != self.wait_config.get("task_type", "ACTION_WAIT_TIME"):
                    self.wait_seen_key = None
                    return
                fields = self.wait_config.get("identity_fields", ["main_task_name", "task_name"])
                key = tuple(str(self._field(data, field)) for field in fields) + (task_type,)
                if key == self.wait_seen_key or self.mode != "auto":
                    return
                presets = self.wait_config.get("presets", ["4", "5", "6", "4", "7", "8", "4"])
                if not presets:
                    raise ValueError("wait_scan.presets不能为空")
                command = make_message(
                    "ptz.command", source="wait_task_ptz", command_id=new_id("wait-scan"),
                    action="sweep", mode="auto", data={"repeat": 1, "steps": [
                        {"preset": str(preset), "speed": self.wait_config.get("speed", 1.0),
                         "settle_seconds": self.wait_config.get("settle_seconds", 3.0),
                         "pause": self.wait_config.get("pause", 0.0)} for preset in presets]})
                command["_wait_scan"] = True
                self.wait_seen_key = key
                self.wait_queue.append(command)
                current = self.current_command
                if (current and not current.get("_wait_scan")
                        and current.get("mode", "auto") == "auto"
                        and not current.get("_requeue")):
                    current["_requeue"] = True
                    self.version += 1
                self._publish_result(command, "accepted")
                self.condition.notify_all()
            rospy.loginfo("底盘等待步骤触发云台：主任务=%s，顺序=%s",
                          data.get("main_task_name", ""), presets)
        except Exception as exc:
            rospy.logerr("等待任务解析失败: %s", exc)

    def _clear_queues(self, reason):
        commands = list(self.normal_queue) + list(self.wait_queue)
        self.normal_queue.clear()
        self.wait_queue.clear()
        if self.current_command is not None:
            self.current_command["_requeue"] = False
        for command in commands:
            self._publish_result(command, "cancelled", reason)

    def _command(self, raw):
        command = None
        try:
            command = copy.deepcopy(parse_message(raw))
            action = str(command.get("action", command.get("method", ""))).lower()
            command["action"] = action
            mode = str(command.get("mode", "auto")).lower()
            command["mode"] = mode
            name = str(command.get("point_name", "")).replace(" ", "")
            initial = any(word in name for word in
                          ("初始点", "原点", "待命点", "充电点", "起始点", "起始"))
            with self.condition:
                if action == "stop":
                    self.version += 1
                    self._clear_queues("stopped")
                    self._require_driver().stop()
                    self.device_state = "STOPPED"
                    self._publish_result(command, "success")
                    return
                if mode == "auto" and "巡航" in name and not initial:
                    self._publish_result(command, "rejected", "巡航点触发已关闭，由ACTION_WAIT_TIME触发")
                    return
                if action == "mode":
                    target = str(command.get("data", {}).get("mode", "auto")).lower()
                    if target not in ("auto", "manual"):
                        raise ValueError("invalid mode")
                    if target != self.mode:
                        self.version += 1
                        self._clear_queues("mode changed")
                        self._require_driver().stop()
                        self.mode = target
                    self._publish_result(command, "success")
                    return
                if action not in ("goto_preset", "home", "move", "sweep"):
                    raise ValueError("unsupported action: " + action)
                if mode not in ("auto", "manual"):
                    raise ValueError("invalid mode")
                if mode == "manual":
                    self.version += 1
                    self._clear_queues("manual override")
                    self._require_driver().stop()
                    self.mode = "manual"
                elif self.mode == "manual":
                    self._publish_result(command, "rejected", "PTZ is in manual mode")
                    return
                # 保留初始/起始点1号定位，不受旧配置的2号预置位影响。
                if initial and mode == "auto" and action in ("goto_preset", "home"):
                    command["action"] = "goto_preset"
                    command.setdefault("data", {})["preset"] = "1"
                self.normal_queue.append(command)
                self._publish_result(command, "accepted")
                self.condition.notify_all()
        except Exception as exc:
            rospy.logerr("PTZ命令失败: %s", exc)
            if command is not None:
                self._publish_result(command, "failed", str(exc))

    def _worker(self):
        while not rospy.is_shutdown():
            with self.condition:
                while not self.wait_queue and not self.normal_queue and not rospy.is_shutdown():
                    self.condition.wait(timeout=0.5)
                if rospy.is_shutdown():
                    return
                command = (self.wait_queue or self.normal_queue).popleft()
                self.version += 1
                command["_version"] = self.version
                command["_requeue"] = False
                self.current_command = command
            try:
                self._execute(command)
                with self.condition:
                    if not self._valid(command["_version"]):
                        raise InterruptedError("superseded")
                    self._publish_result(command, "success")
            except InterruptedError:
                try:
                    if self.driver is not None:
                        self.driver.stop()
                except Exception:
                    pass
                with self.condition:
                    if command.get("_requeue") and self.mode == "auto" and not rospy.is_shutdown():
                        command["_requeue"] = False
                        self.normal_queue.appendleft(command)
                    else:
                        self._publish_result(command, "cancelled", "superseded")
            except Exception as exc:
                try:
                    if self.driver is not None:
                        self.driver.stop()
                except Exception:
                    pass
                self.device_state = "ERROR"
                self._publish_result(command, "failed", str(exc))
                rospy.logerr("云台动作失败: %s", exc)
            finally:
                with self.condition:
                    self.current_command = None


if __name__ == "__main__":
    rospy.init_node("ptz_controller")
    WaitTaskPTZController()
    rospy.spin()

