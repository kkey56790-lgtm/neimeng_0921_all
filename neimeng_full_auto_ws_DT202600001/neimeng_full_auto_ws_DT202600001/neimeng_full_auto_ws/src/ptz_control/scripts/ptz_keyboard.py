#!/usr/bin/env python3
import json
import sys
import termios
import tty
import time

import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message


def result_callback(raw):
    try:
        value = json.loads(raw.data)
        print("\r\n[云台回执] action={} status={} state={} error={}".format(
            value.get("action", ""), value.get("status", ""),
            value.get("data", {}).get("device_state", ""),
            value.get("data", {}).get("error", ""),
        ))
    except Exception:
        pass


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rospy.init_node("ptz_keyboard")
    publisher = rospy.Publisher(Topics.PTZ_COMMAND, String, queue_size=10)
    rospy.Subscriber(Topics.PTZ_RESULT, String, result_callback, queue_size=20)
    print("W/S/A/D移动 空格停止 1/2/3/4预置点 R归位 M手动 U自动 Q退出")
    print("等待 ptz_controller 订阅连接...")
    deadline = time.monotonic() + 5.0
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.1)
    if publisher.get_num_connections() == 0:
        print("未发现 /inspection/ptz_command 订阅者，请先启动 ptz_test.launch")
    else:
        print("已连接云台控制节点")
    publisher.publish(String(data=dumps(make_message("ptz.command", action="mode", data={"mode": "manual"}))))
    rospy.sleep(0.2)
    try:
        while not rospy.is_shutdown():
            key = get_key().lower()
            if key in {"w", "s", "a", "d"}:
                direction = {"w": "up", "s": "down", "a": "left", "d": "right"}[key]
                message = make_message("ptz.command", action="move", mode="manual", data={"direction": direction, "start": True, "speed": 5})
            elif key == " ":
                message = make_message("ptz.command", action="stop", mode="manual")
            elif key in {"1", "2", "3", "4"}:
                message = make_message("ptz.command", action="goto_preset", mode="manual", data={"preset": key})
            elif key == "r":
                message = make_message("ptz.command", action="home", mode="manual", data={"preset": "1"})
            elif key == "m":
                message = make_message("ptz.command", action="mode", data={"mode": "manual"})
            elif key == "u":
                message = make_message("ptz.command", action="mode", data={"mode": "auto"})
            elif key == "q":
                break
            else:
                continue
            publisher.publish(String(data=dumps(message)))
    finally:
        publisher.publish(String(data=dumps(make_message("ptz.command", action="stop", mode="manual"))))
        rospy.sleep(0.1)


if __name__ == "__main__":
    main()
