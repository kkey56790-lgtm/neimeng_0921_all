#!/usr/bin/env python3
"""Final production orchestrator with task-finish/YOLO ordering protection."""

import copy

import rospy

from inspection_interfaces.protocol import parse_message
from production_inspection_orchestrator import ProductionInspectionOrchestrator


class ProductionInspectionOrchestratorV2(ProductionInspectionOrchestrator):
    def __init__(self):
        self.finished_during_detection_generation = -1
        super().__init__()

    def _task_status(self, raw):
        try:
            message = parse_message(raw)
            data = message.get("data", {}) if isinstance(message.get("data", {}), dict) else {}
            state = str(data.get("task_state", ""))
            current = self.current_execution or {}
            expected = self.expected_detection or {}
            if (
                state == "STATE_FINISH"
                and expected
                and current
                and expected.get("generation") == current.get("generation")
            ):
                self.last_task_status = data
                self.finished_during_detection_generation = current.get("generation", -1)
                self._event("TASK_FINISH_HELD_FOR_YOLO", {
                    "task_name": current.get("name"),
                    "task_number": current.get("number"),
                    "detection_id": expected.get("detection_id"),
                })
                self._publish_status()
                return
        except Exception:
            pass
        super()._task_status(raw)

    def _execute_decision(self, decision):
        action = str(decision.get("action", "")).lower() if isinstance(decision, dict) else str(decision).lower()
        number = int(decision.get("task_number", 0) or 0) if isinstance(decision, dict) else 0
        current = copy.deepcopy(self.current_execution) if self.current_execution else {}
        held_finish = current and current.get("generation") == self.finished_during_detection_generation
        if action == "numbered_vehicle" and held_finish and current.get("number") == number:
            self.finished_during_detection_generation = -1
            self._event("TRUCK_CONFIRMED_AFTER_TASK_FINISH", {"task_number": number})
            self._start_numbered_task(
                number,
                kind=current.get("kind", "auto"),
                requested_name=current.get("name", ""),
                platform_sequence=current.get("platform_sequence", 0),
            )
            self.vehicle_confirmed_generation = (self.current_execution or {}).get("generation", -1)
            return
        if action in ("numbered_vehicle", "numbered_empty", "numbered_detection_error"):
            self.finished_during_detection_generation = -1
        super()._execute_decision(decision)


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    ProductionInspectionOrchestratorV2()
    rospy.spin()

