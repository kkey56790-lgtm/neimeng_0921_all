#!/usr/bin/env python3
"""Catkin-safe entry point for the complete production orchestrator."""

import os
import sys

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

import rospy
from production_inspection_orchestrator_final import ProductionInspectionOrchestratorFinal


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    ProductionInspectionOrchestratorFinal()
    rospy.spin()
