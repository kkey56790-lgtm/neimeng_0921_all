#!/usr/bin/env python3
"""Full-auto orchestrator restricted to tasks of the robot's current map."""

import copy
import os
import sys

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

import rospy

from inspection_interfaces.protocol import parse_message
from production_inspection_orchestrator_final import ProductionInspectionOrchestratorFinal


class MapScopedFullAutoOrchestrator(ProductionInspectionOrchestratorFinal):
    def _catalog(self, raw):
        message = parse_message(raw)
        forwarded = copy.deepcopy(message)
        data = forwarded.get("data", {})
        current_tasks = data.get("tasks", []) if isinstance(data, dict) else []
        # robot_bridge has already filtered data.tasks by current map. Replacing
        # all_tasks prevents numbered matching from selecting another map.
        data["all_tasks"] = list(current_tasks)
        super()._catalog(forwarded)


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    MapScopedFullAutoOrchestrator()
    rospy.spin()
