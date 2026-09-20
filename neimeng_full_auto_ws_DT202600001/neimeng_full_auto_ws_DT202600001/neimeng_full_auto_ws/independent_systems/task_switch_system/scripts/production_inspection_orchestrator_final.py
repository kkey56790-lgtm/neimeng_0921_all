#!/usr/bin/env python3
"""Final real-inspection entry point with charging-phase validation."""

import rospy

from production_inspection_orchestrator_v4 import ProductionInspectionOrchestratorV4


class ProductionInspectionOrchestratorFinal(ProductionInspectionOrchestratorV4):
    def _maybe_finish_recharge(self):
        # Battery reaching 80% is meaningful only after the recharge route itself
        # has completed and the state machine has entered the charging phase.
        if not self.charging:
            return
        super()._maybe_finish_recharge()


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    ProductionInspectionOrchestratorFinal()
    rospy.spin()

