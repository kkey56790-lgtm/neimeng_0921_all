#!/usr/bin/env python3
"""Current-map numbered task switching without owning PTZ hardware."""

import rospy

from map_scoped_full_auto_orchestrator import MapScopedFullAutoOrchestrator


class TaskSwitchOnlyOrchestrator(MapScopedFullAutoOrchestrator):
    def _begin_point(self):
        self._cancel_timer()
        self.detection_started_at = None
        self.point_ptz_ready = True
        self.state = "POINT_REACHED_TASK_SWITCH_ONLY"
        self._event("POINT_ARRIVED_TASK_SWITCH_ONLY")


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    TaskSwitchOnlyOrchestrator()
    rospy.spin()
