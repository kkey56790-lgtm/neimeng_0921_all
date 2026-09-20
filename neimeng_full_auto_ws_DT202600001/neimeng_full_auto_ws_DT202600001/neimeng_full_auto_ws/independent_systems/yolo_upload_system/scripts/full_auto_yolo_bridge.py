#!/usr/bin/env python3
"""Catkin-safe, start-point-only YOLO bridge for full-auto real tests."""

import os
import sys
import time

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

import rospy

from gated_yolo_bridge import GatedYoloBridge
from yolo_bridge import zmq


class FullAutoYoloBridge(GatedYoloBridge):
    def _receive(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.setsockopt(zmq.RCVTIMEO, 500)
        socket.connect(self.address)
        rospy.loginfo("连接RKNN YOLO（仅起始点采集）: %s", self.address)
        try:
            while self.running and not rospy.is_shutdown():
                try:
                    frame = socket.recv_json()
                except zmq.Again:
                    self._publish_health()
                    continue
                now = time.monotonic()
                with self.lock:
                    self.last_receive = now
                    collecting = self.active is not None
                    if collecting:
                        self.latest = frame
                        self._collect_frame(frame)
                    else:
                        self.latest = None
                self._publish_health()
        finally:
            socket.close(linger=0)
            context.term()


if __name__ == "__main__":
    rospy.init_node("yolo_bridge")
    FullAutoYoloBridge()
    rospy.spin()
