#!/usr/bin/env python3
"""Production orchestrator with terminal command acknowledgements."""

import rospy

from inspection_interfaces.protocol import parse_message
from production_inspection_orchestrator_v2 import ProductionInspectionOrchestratorV2
from real_flow_rules import task_number


class ProductionInspectionOrchestratorV4(ProductionInspectionOrchestratorV2):
    """Add protocol-level rejection replies without changing the V2 state machine."""

    def _control(self, raw):
        message = None
        try:
            message = parse_message(raw)
            action = str(message.get("action", message.get("method", ""))).lower()
            data = dict(message.get("data", {}))
            command_id = str(message.get("command_id", message.get("tid", "")))

            if action in ("inspection.platform_task", "platform_task", "task.start"):
                try:
                    with self.lock:
                        self._queue_platform_task(data, command_id)
                        self._publish_status()
                except Exception as exc:
                    self._publish_command_status(command_id, "rejected", {
                        "reason": "task_resolution_failed",
                        "error": str(exc),
                    })
                    self._event("PLATFORM_TASK_REJECTED", {
                        "command_id": command_id,
                        "error": str(exc),
                        "request": data,
                    })
                    self._publish_status(str(exc))
                return

            if action in ("start", "inspection.start"):
                requested = str(data.get("task_name", data.get("name", ""))).strip()
                number = task_number(requested)
                if number is None:
                    raise ValueError("启动任务名称必须包含任务编号关键词")
                self._resolve_numbered_task(number, requested)

            super()._control(message)
        except Exception as exc:
            command_id = ""
            if isinstance(message, dict):
                command_id = str(message.get("command_id", message.get("tid", "")))
            self._publish_command_status(command_id, "rejected", {
                "reason": "inspection_start_rejected",
                "error": str(exc),
            })
            self.state = "ERROR"
            self._event("INSPECTION_START_REJECTED", {"error": str(exc)})
            self._publish_status(str(exc))

    def _task_status(self, raw):
        try:
            message = parse_message(raw)
            command_status = str(message.get("command_status", "")).lower()
            if command_status in ("failed", "rejected", "cancelled", "canceled"):
                if command_status in ("failed", "rejected"):
                    self.state = "ERROR"
                self._event("REAL_TASK_COMMAND_" + command_status.upper(), {
                    "command_id": message.get("command_id", ""),
                    "detail": message.get("data", {}),
                })
                detail = message.get("data", {})
                error = detail.get("error", "task command " + command_status) \
                    if isinstance(detail, dict) else "task command " + command_status
                self._publish_status(str(error))
                return
        except Exception:
            pass
        super()._task_status(raw)

    def _start_recharge(self, resume_number):
        try:
            super()._start_recharge(resume_number)
        except Exception as exc:
            self.state = "ERROR"
            self._event("RECHARGE_TASK_RESOLUTION_FAILED", {
                "keywords": self.recharge_keywords,
                "error": str(exc),
            })
            self._publish_status(str(exc))


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    ProductionInspectionOrchestratorV4()
    rospy.spin()
