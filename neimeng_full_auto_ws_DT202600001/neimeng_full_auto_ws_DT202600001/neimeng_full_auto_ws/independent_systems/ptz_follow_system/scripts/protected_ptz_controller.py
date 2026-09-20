#!/usr/bin/env python3
"""PTZ controller that never lets a sweep interrupt a preset transition."""

from collections import deque

import rospy

from inspection_interfaces.protocol import parse_message
from ptz_controller import PTZController


class ProtectedPTZController(PTZController):
    PROTECTED_ACTIONS = ("goto_preset", "home")

    def __init__(self):
        self.deferred_sweeps = deque()
        self.executing_command = None
        super().__init__()

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
        try:
            command = parse_message(raw)
            action = str(command.get("action", command.get("method", ""))).lower()
            command["action"] = action
        except Exception:
            return super()._command(raw)

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
            if command is None:
                continue

            completed = False
            self.executing_command = command
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
                self.executing_command = None

            # Delayed sweep receives a fresh version only after the protected
            # command has already returned success, so it cannot cancel it.
            with self.condition:
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

