#!/usr/bin/env python3
"""Catkin-safe entry point for preset-protected PTZ control."""

import os
import sys

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

import rospy
from protected_ptz_controller import ProtectedPTZController


if __name__ == "__main__":
    rospy.init_node("ptz_controller")
    ProtectedPTZController()
    rospy.spin()
