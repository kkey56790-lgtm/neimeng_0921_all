#!/usr/bin/env python3
"""YOLO bridge that exposes detections only during validated start-point windows."""

import time

import rospy
from std_msgs.msg import String

from gated_yolo_bridge import GatedYoloBridge
from inspection_interfaces.protocol import dumps, make_message
from yolo_bridge import zmq


class StartPointYoloBridge(GatedYoloBridge):
    """Keep ZMQ healthy, but never publish/collect object frames outside a start point."""


    def _receive(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.setsockopt(zmq.RCVTIMEO, 500)
        socket.connect(self.address)
        rospy.loginfo("连接YOLO ZMQ发布端（仅起始点采集）: %s", self.address)
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
                        # Do not retain object data outside an authorized start-point window.
                        self.latest = None

                if collecting and now - self.last_frame_publish >= self.frame_publish_interval:
                    self.last_frame_publish = now
                    message = make_message(
                        "detection.frame", source="start_point_yolo_bridge", data=frame)
                    self.frame_pub.publish(String(data=dumps(message)))
                self._publish_health()
        finally:
            socket.close(linger=0)
            context.term()


if __name__ == "__main__":
    rospy.init_node("yolo_bridge")
    StartPointYoloBridge()
    rospy.spin()
