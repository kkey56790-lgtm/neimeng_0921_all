#!/usr/bin/env python3
"""PTZ controller that never lets a sweep interrupt a preset transition."""

from collections import deque

import rospy

from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, new_id, parse_message
from ptz_controller import PTZController


class ProtectedPTZController(PTZController):
    PROTECTED_ACTIONS = ("goto_preset", "home")

    def __init__(self):
        self.deferred_sweeps = deque()
        self.executing_command = None
        self.wait_active = False
        self.wait_suppressed = False
        self.wait_deferred = []
        self.wait_config = rospy.get_param("~wait_task_follow", {})
        self.wait_rules = rospy.get_param("~point_rules", [])
        self.wait_profiles = rospy.get_param("~sweep_profiles", {})
        super().__init__()
        rospy.Subscriber(Topics.TASK_STATUS, String, self._task_status, queue_size=20)

    def _waiting(self, data):
        # Only the currently executing action qualifies, never the task name,
        # a queued action, a paused task, or a generic idle/waiting state.
        types = self.wait_config.get("task_types", ["TASK_WAIT"])
        return (str(data.get("task_state", "")).upper() == "STATE_DOING"
                and str(data.get("task_type", "")).upper() in
                {str(value).upper() for value in types})

    def _wait_command(self, action, data=None):
        return {"action": action, "command_id": new_id("ptz-wait"),
                "point_name": "", "_wait_owned": True, "data": data or {}}

    def _wait_rule(self):
        return next((rule for rule in self.wait_rules
                     if isinstance(rule, dict) and rule.get("id") == "cruise"), {})

    def _task_status(self, raw):
        try:
            data = parse_message(raw).get("data", {})
            if not isinstance(data, dict) or not data.get("task_state"):
                return
            active = self._waiting(data) and self.wait_config.get("enabled", True)
            with self.condition:
                if active == self.wait_active:
                    return
                self.wait_active = active
                if active:
                    self.wait_suppressed = self.mode == "manual"
                    if self.wait_suppressed:
                        return
                    rule = self._wait_rule()
                    profile = self.wait_profiles.get(rule.get("sweep_profile", "normal"), {})
                    if not rule or not profile.get("steps"):
                        self.wait_suppressed = True
                        rospy.logwarn("等待任务云台随动缺少巡航规则或扫描步骤")
                        return
                    # Preserve the current point intent for after the wait.
                    current = self.pending or self.executing_command
                    if current and not current.get("_wait_owned"):
                        self.wait_deferred.append(dict(current))
                    self.wait_deferred.extend(self.deferred_sweeps)
                    self.deferred_sweeps.clear()
                    self._command(self._wait_command("goto_preset", {
                        "preset": str(rule.get("preset", "4")),
                        "speed": rule.get("speed", 1.0),
                        "settle_seconds": rule.get("settle_seconds", 2.0),
                    }))
                else:
                    deferred, self.wait_deferred = self.wait_deferred, []
                    if not self.wait_suppressed:
                        self._command(self._wait_command("stop"))
                        for command in deferred:
                            self._command(command)
                    self.wait_suppressed = False
        except Exception as exc:
            rospy.logwarn("等待任务云台状态处理失败: %s", exc)

    def _continue_wait(self, command, completed):
        if (not completed or not command.get("_wait_owned")
                or not self.wait_active or self.wait_suppressed
                or self.mode == "manual" or not self._valid(command["_version"])):
            return
        rule = self._wait_rule()
        profile = self.wait_profiles.get(rule.get("sweep_profile", "normal"), {})
        # One complete existing profile at a time; repeated telemetry never
        # restarts a step. Leaving the wait invalidates this command version.
        self._command(self._wait_command("sweep", {
            "steps": profile.get("steps", []), "repeat": profile.get("repeat", 1),
        }))

    @staticmethod
    def _action(command):
        return str((command or {}).get("action", "")).lower()

    def _protected_command(self):
        if self._action(self.executing_command) in self.PROTECTED_ACTIONS:
            return self.executing_command
        if self._action(self.pending) in self.PROTECTED_ACTIONS:
            return self.pending
        return None

    def _cancel_deferred_sweeps(self, reason):
        with self.condition:
            cancelled = list(self.deferred_sweeps)
            self.deferred_sweeps.clear()
        for command in cancelled:
            self._publish_result(command, "cancelled", reason)

    def _command(self, raw):
        with self.condition:
            return self._handle_command(raw)

    def _handle_command(self, raw):
        try:
            command = parse_message(raw)
            action = str(command.get("action", command.get("method", ""))).lower()
            command["action"] = action
        except Exception:
            return super()._command(raw)

        with self.condition:
            if self.wait_active and not command.get("_wait_owned"):
                manual = (str(command.get("mode", "auto")).lower() == "manual"
                          or (action == "mode" and
                              command.get("data", {}).get("mode") == "manual"))
                if action == "stop" or manual:
                    self.wait_suppressed = True
                    for old in self.wait_deferred:
                        self._publish_result(old, "cancelled", "manual_or_stop")
                    self.wait_deferred.clear()
                elif not self.wait_suppressed:
                    if action in ("mode", "goto_preset", "home"):
                        for old in self.wait_deferred:
                            self._publish_result(old, "cancelled", "superseded_by_new_point")
                        self.wait_deferred.clear()
                    self.wait_deferred.append(command)
                    self._publish_result(command, "accepted")
                    return

        if action == "sweep":
            with self.condition:
                protected = self._protected_command()
                requested_mode = str(command.get("mode", "auto")).lower()
                if protected and requested_mode == "auto" and self.mode != "manual":
                    protected_point = str(protected.get("point_name", "")).strip()
                    sweep_point = str(command.get("point_name", "")).strip()
                    if protected_point and sweep_point and protected_point != sweep_point:
                        self._publish_result(command, "cancelled", "sweep belongs to an older point")
                        return
                    self.deferred_sweeps.append(command)
                    self._publish_result(command, "accepted")
                    rospy.loginfo("云台扫描已排队，等待预置点完成: %s", protected_point or "unknown")
                    return

        # A new preset/home/stop/mode operation represents a new PTZ intent.
        # It may interrupt old work, and any old deferred scan must not run later.
        if action in ("goto_preset", "home", "stop", "mode"):
            self._cancel_deferred_sweeps("superseded_by_new_ptz_intent")
        super()._command(command)

    def _worker(self):
        while not rospy.is_shutdown():
            with self.condition:
                while self.pending is None and not rospy.is_shutdown():
                    self.condition.wait(timeout=0.5)
                command, self.pending = self.pending, None
                self.executing_command = command
            if command is None:
                continue

            completed = False
            try:
                self._execute(command)
                completed = self._valid(command["_version"])
                if completed:
                    self._publish_result(command, "success")
            except InterruptedError:
                self.device_state = "CANCELLED"
                self._publish_result(command, "cancelled")
            except Exception as exc:
                rospy.logerr("云台动作失败: %s", exc)
                try:
                    if self.driver is not None:
                        self.driver.stop()
                except Exception:
                    pass
                self.device_state = "ERROR"
                self._publish_result(command, "failed", str(exc))
                self._publish_health("error", str(exc))
            finally:
                with self.condition:
                    self.executing_command = None

            # Delayed sweep receives a fresh version only after the protected
            # command has already returned success, so it cannot cancel it.
            with self.condition:
                self._continue_wait(command, completed)
                if completed and self.pending is None and self.deferred_sweeps:
                    deferred = self.deferred_sweeps.popleft()
                    self.version += 1
                    deferred["_version"] = self.version
                    self.pending = deferred
                    self.device_state = "QUEUED_SWEEP"
                    self.condition.notify_all()


if __name__ == "__main__":
    rospy.init_node("ptz_controller")
    ProtectedPTZController()
    rospy.spin()


