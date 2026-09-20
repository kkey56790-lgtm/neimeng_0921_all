#!/usr/bin/env python3
"""Poll robot state and request remote WAV playback through HTTP V2.4."""

import math
import time

import requests
import rospy
from std_msgs.msg import String


STATE_AUDIO = {
    "inspecting": "01_inspecting.wav",
    "stopped": "02_stopped.wav",
    "reversing": "03_reversing.wav",
    "turn_left": "04_turn_left.wav",
    "turn_right": "05_turn_right.wav",
    "avoiding": "06_avoiding_obstacle.wav",
}

STATE_LABEL = {
    "inspecting": "正在巡检或前进",
    "stopped": "机器人停车",
    "reversing": "机器人倒车",
    "turn_left": "机器人左转",
    "turn_right": "机器人右转",
    "avoiding": "机器人避障",
    "unknown": "状态未知",
}


def finite_float(value, default=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def classify_state(base_data, task_data, linear_threshold, angular_threshold,
                   left_turn_positive):
    base_valid = isinstance(base_data, dict)
    task_valid = isinstance(task_data, dict)
    if not base_valid and not task_valid:
        return "unknown"
    base_data = base_data if base_valid else {}
    task_data = task_data if task_valid else {}
    robot = base_data.get("robot") or {}
    nav = base_data.get("nav") or {}
    obstacle = bool(nav.get("obstacle", False) or task_data.get("obstacle", False))
    vx = finite_float(robot.get("vx", 0.0))
    vz = finite_float(robot.get("vz", 0.0))
    task_state = str(task_data.get("task_state", "")).upper()

    if obstacle:
        return "avoiding"
    if base_valid and vx < -linear_threshold:
        return "reversing"
    if base_valid and abs(vz) > angular_threshold:
        positive_means_left = vz > 0.0
        is_left = positive_means_left if left_turn_positive else not positive_means_left
        return "turn_left" if is_left else "turn_right"
    if task_state == "STATE_DOING" or (base_valid and vx > linear_threshold):
        return "inspecting"
    if base_valid and abs(vx) <= linear_threshold and abs(vz) <= angular_threshold:
        return "stopped"
    return "unknown"


class RobotVoiceAnnouncer:
    def __init__(self):
        robot_ip = str(rospy.get_param("~robot_ip", "192.168.2.100")).strip()
        robot_port = int(rospy.get_param("~robot_port", 7999))
        self.base_url = "http://{}:{}".format(robot_ip, robot_port)
        self.http_timeout = float(rospy.get_param("~http_timeout", 0.6))
        self.poll_hz = max(0.2, float(rospy.get_param("~poll_hz", 5.0)))
        self.linear_threshold = abs(float(rospy.get_param("~linear_threshold", 0.03)))
        self.angular_threshold = abs(float(rospy.get_param("~angular_threshold", 0.08)))
        self.left_turn_positive = bool(rospy.get_param("~left_turn_positive", True))
        self.verify_remote_files = bool(rospy.get_param("~verify_remote_files", True))
        self.repeat_seconds = {
            "inspecting": float(rospy.get_param("~repeat_inspecting", 15.0)),
            "stopped": float(rospy.get_param("~repeat_stopped", 15.0)),
            "reversing": float(rospy.get_param("~repeat_reversing", 6.0)),
            "turn_left": float(rospy.get_param("~repeat_turning", 6.0)),
            "turn_right": float(rospy.get_param("~repeat_turning", 6.0)),
            "avoiding": float(rospy.get_param("~repeat_avoiding", 6.0)),
        }
        self.session = requests.Session()
        self.last_state = "unknown"
        self.last_play_time = 0.0
        self.last_http_warning = 0.0
        self.state_pub = rospy.Publisher("~state", String, queue_size=5, latch=True)
        if self.verify_remote_files:
            self.verify_voice_files()

    @staticmethod
    def success_payload(response, operation):
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict) or not payload.get("result", False):
            detail = payload.get("data", payload) if isinstance(payload, dict) else payload
            raise RuntimeError("{} 失败: {}".format(operation, detail))
        return payload

    def verify_voice_files(self):
        response = self.session.get(self.base_url + "/system_manager/get_wav_list",
                                    timeout=self.http_timeout)
        payload = self.success_payload(response, "获取语音列表")
        names = {str(item.get("name")) for item in payload.get("data", [])
                 if isinstance(item, dict) and item.get("name")}
        missing = sorted(set(STATE_AUDIO.values()) - names)
        if missing:
            raise RuntimeError("机器人缺少语音文件: {}".format(", ".join(missing)))
        rospy.loginfo("已通过 HTTP 确认六个语音文件存在")

    def get_data(self, endpoint):
        try:
            response = self.session.get(self.base_url + endpoint, timeout=self.http_timeout)
            payload = self.success_payload(response, endpoint)
            data = payload.get("data")
            return data if isinstance(data, dict) else None
        except (requests.RequestException, ValueError, RuntimeError) as exc:
            now = time.monotonic()
            if now - self.last_http_warning >= 5.0:
                rospy.logwarn("机器人 HTTP 接口不可用 %s: %s", endpoint, exc)
                self.last_http_warning = now
            return None

    def play(self, state):
        filename = STATE_AUDIO[state]
        try:
            response = self.session.post(
                self.base_url + "/system_manager/play_wav_data",
                json={"file_name": filename}, timeout=self.http_timeout)
            self.success_payload(response, "播放 {}".format(filename))
            rospy.loginfo("已请求机器人播放: %s", filename)
            return True
        except (requests.RequestException, ValueError, RuntimeError) as exc:
            rospy.logwarn("机器人语音播放失败 %s: %s", filename, exc)
            return False

    def run(self):
        rate = rospy.Rate(self.poll_hz)
        while not rospy.is_shutdown():
            base_data = self.get_data("/sensor_data/base_data")
            task_data = self.get_data("/task_manager/get_task_status")
            state = classify_state(base_data, task_data, self.linear_threshold,
                                   self.angular_threshold, self.left_turn_positive)
            self.state_pub.publish(String(data=state))
            now = time.monotonic()
            changed = state != self.last_state
            repeat_due = (state in self.repeat_seconds
                          and now - self.last_play_time >= self.repeat_seconds[state])
            if changed:
                rospy.loginfo("语音状态: %s", STATE_LABEL[state])
                self.last_state = state
            if state in STATE_AUDIO and (changed or repeat_due):
                self.play(state)
                self.last_play_time = now
            rate.sleep()


def main():
    rospy.init_node("robot_voice_announcer")
    try:
        RobotVoiceAnnouncer().run()
    except Exception as exc:
        rospy.logfatal("语音播报节点启动失败: %s", exc)
        raise


if __name__ == "__main__":
    main()
