#!/usr/bin/env python3
"""Manually trigger one gated YOLO collection for standalone verification."""

import argparse
import threading

import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, new_id, parse_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--point", default="起始点01")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--mode", choices=("truck", "inspection"), default="truck")
    args, _ = parser.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("manual_detection_trigger", anonymous=True)
    pub = rospy.Publisher(Topics.DETECTION_CONTROL, String, queue_size=2)
    done = threading.Event()
    detection_id = new_id("manual-detection")

    def result(raw):
        message = parse_message(raw)
        data = message.get("data", {}) if isinstance(message.get("data", {}), dict) else {}
        if data.get("detection_id") in ("", detection_id) and message.get("point_name") == args.point:
            rospy.loginfo("YOLO结果: %s", dumps(message))
            done.set()

    rospy.Subscriber(Topics.DETECTION_RESULT, String, result, queue_size=5)
    rospy.sleep(0.6)
    number_text = "".join(ch for ch in args.point if ch.isdigit())
    number = int(number_text or 0)
    start = make_message(
        "detection.control", source="manual_detection_trigger", action="start",
        task_id="manual", point_seq=number, point_name=args.point,
        data={
            "detection_id": detection_id,
            "task_number": number,
            "collection_mode": args.mode,
        },
    )
    pub.publish(String(data=dumps(start)))
    rospy.sleep(max(0.1, args.seconds))
    stop = make_message(
        "detection.control", source="manual_detection_trigger", action="stop",
        task_id="manual", point_seq=number, point_name=args.point,
        data={"detection_id": detection_id, "collection_mode": args.mode},
    )
    pub.publish(String(data=dumps(stop)))
    done.wait(3.0)


if __name__ == "__main__":
    main()
