#!/usr/bin/env python3
"""Production safety refinements for the real numbered orchestrator."""

import rospy

from inspection_interfaces.protocol import parse_message
from real_inspection_orchestrator import RealInspectionOrchestrator


class ProductionInspectionOrchestrator(RealInspectionOrchestrator):
    def _execute_decision(self, decision):
        action = str(decision.get("action", "")).lower() if isinstance(decision, dict) else str(decision).lower()
        number = int(decision.get("task_number", 0) or 0) if isinstance(decision, dict) else 0
        previous_number = (self.current_execution or {}).get("number")
        super()._execute_decision(decision)
        if action == "numbered_vehicle" and previous_number != number:
            self.vehicle_confirmed_generation = (self.current_execution or {}).get("generation", -1)

    def _task_status(self, raw):
        try:
            message = parse_message(raw)
            data = message.get("data", {}) if isinstance(message.get("data", {}), dict) else {}
            state = str(data.get("task_state", ""))
            if state in ("STATE_FAIL", "STATE_FAILED", "STATE_CANCEL", "STATE_CANCELED"):
                reported = self._reported_task_name(data)
                if not reported or not self._reported_matches_current(reported):
                    return
        except Exception:
            pass
        super()._task_status(raw)

    def _handle_obstacle(self, obstacle):
        # Missing obstacle fields mean "no new sample", not "obstacle cleared".
        if obstacle is None:
            return
        super()._handle_obstacle(obstacle)


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    ProductionInspectionOrchestrator()
    rospy.spin()

