#!/usr/bin/env python3
"""Bridge vehicle MQTT voice commands and OSD state to the robot HTTP V2.4 API."""

import json
import math
import queue
import threading
import time
from collections import OrderedDict

import paho.mqtt.client as mqtt
import requests
import rospy
from std_msgs.msg import String


STATE_AUDIO = {
    "inspecting": "正在巡检中请勿靠近.wav",
    "stopped": "机器人停车请避让.wav",
    "reversing": "倒车请注意.wav",
    "turn_left": "左转.wav",
    "turn_right": "右转.wav",
    "avoiding": "机器人避障中.wav",
}
PROTECTED_REMOTE_FILES = {"turn_left.wav", "turn_right.wav", "stop_car.wav"}
SUPPORTED_METHODS = {"voice_package_update", "voice_package_list", "voice_switch"}


def finite_float(value, default=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def classify_osd(data, linear_threshold, angular_threshold, left_turn_positive):
    if not isinstance(data, dict):
        return "unknown"
    robot = data.get("robot") or {}
    nav = data.get("nav") or {}
    navigation = data.get("navigation") or {}
    task = data.get("currentTask") or {}
    obstacle = bool(nav.get("obstacle", False) or task.get("obstacle", False)
                    or str(navigation.get("state", "")).upper() == "STATE_AVOID")
    vx = finite_float(robot.get("vx", 0.0))
    vz = finite_float(robot.get("vz", 0.0))
    task_state = str(task.get("task_state", "")).upper()
    orchestrator_state = str(data.get("state", "")).upper()

    if obstacle:
        return "avoiding"
    if vx < -linear_threshold:
        return "reversing"
    if abs(vz) > angular_threshold:
        positive_means_left = vz > 0.0
        is_left = positive_means_left if left_turn_positive else not positive_means_left
        return "turn_left" if is_left else "turn_right"
    if task_state == "STATE_DOING" or orchestrator_state not in ("", "IDLE") or vx > linear_threshold:
        return "inspecting"
    if abs(vx) <= linear_threshold and abs(vz) <= angular_threshold:
        return "stopped"
    return "unknown"


class HttpVoiceApi:
    def __init__(self, base_url, timeout):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout if timeout and timeout > 0 else None
        self.session = requests.Session()
        self.lock = threading.Lock()

    @staticmethod
    def require_success(response, operation):
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict) or not payload.get("result", False):
            detail = payload.get("data", payload) if isinstance(payload, dict) else payload
            raise RuntimeError("{}失败: {}".format(operation, detail))
        return payload

    def get_list(self):
        with self.lock:
            response = self.session.get(self.base_url + "/system_manager/get_wav_list",
                                        timeout=self.timeout)
        data = self.require_success(response, "获取语音列表").get("data", [])
        return [item for item in data if isinstance(item, dict) and item.get("name")]

    def delete(self, filename):
        with self.lock:
            response = self.session.post(
                self.base_url + "/system_manager/delete_wav_data",
                json={"file_name": filename}, timeout=self.timeout)
        self.require_success(response, "删除语音")

    def create(self, filename, text, voice_type):
        name_without_suffix = filename[:-4] if filename.lower().endswith(".wav") else filename
        with self.lock:
            response = self.session.post(
                self.base_url + "/system_manager/create_wav_data",
                json={"text": text, "name": name_without_suffix, "type": voice_type},
                timeout=self.timeout)
        self.require_success(response, "创建语音")

    def play(self, filename):
        with self.lock:
            response = self.session.post(
                self.base_url + "/system_manager/play_wav_data",
                json={"file_name": filename}, timeout=self.timeout)
        self.require_success(response, "播放语音")


class MqttVoiceBridge:
    def __init__(self):
        self.robot_code = str(rospy.get_param("~robot_code", "")).strip()
        if not self.robot_code:
            raise RuntimeError("robot_code 不能为空")
        self.robot_name = str(rospy.get_param("~robot_name", "")).strip()
        self.broker_host = str(rospy.get_param("~broker_host"))
        self.broker_port = int(rospy.get_param("~broker_port", 1883))
        self.keepalive = int(rospy.get_param("~keepalive", 30))
        self.qos = int(rospy.get_param("~qos", 1))
        self.username = str(rospy.get_param("~mqtt_username", ""))
        self.password = str(rospy.get_param("~mqtt_password", ""))
        if not self.username or not self.password:
            raise RuntimeError("MQTT 用户名或密码未配置")

        http_ip = str(rospy.get_param("~robot_http_ip", "192.168.2.100"))
        http_port = int(rospy.get_param("~robot_http_port", 7999))
        self.api = HttpVoiceApi("http://{}:{}".format(http_ip, http_port),
                                float(rospy.get_param("~http_timeout", 0.0)))
        self.enable_voice_commands = bool(rospy.get_param("~enable_voice_commands", True))
        self.enable_state_announcer = bool(rospy.get_param("~enable_state_announcer", True))
        self.linear_threshold = abs(float(rospy.get_param("~linear_threshold", 0.03)))
        self.angular_threshold = abs(float(rospy.get_param("~angular_threshold", 0.08)))
        self.left_turn_positive = bool(rospy.get_param("~left_turn_positive", True))
        self.repeat_seconds = {
            "inspecting": float(rospy.get_param("~repeat_inspecting", 15.0)),
            "stopped": float(rospy.get_param("~repeat_stopped", 15.0)),
            "reversing": float(rospy.get_param("~repeat_reversing", 6.0)),
            "turn_left": float(rospy.get_param("~repeat_turning", 6.0)),
            "turn_right": float(rospy.get_param("~repeat_turning", 6.0)),
            "avoiding": float(rospy.get_param("~repeat_avoiding", 6.0)),
        }

        prefix = "thing/robot/{}/".format(self.robot_code)
        self.services_topic = prefix + "services"
        self.reply_topic = prefix + "services_reply"
        self.osd_topic = prefix + "osd"
        self.jobs = queue.Queue(maxsize=100)
        self.latest_osd = None
        self.osd_event = threading.Event()
        self.reply_cache = OrderedDict()
        self.cache_lock = threading.Lock()
        self.last_state = "unknown"
        self.last_play_time = 0.0
        self.state_pub = rospy.Publisher("~state", String, queue_size=5, latch=True)

        client_id = "voice-bridge-{}-{}".format(self.robot_code, int(time.time()))
        self.client = mqtt.Client(client_id=client_id, clean_session=True)
        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    def on_connect(self, client, _userdata, _flags, rc, _properties=None):
        if rc != 0:
            rospy.logerr("MQTT 连接失败，返回码 %s", rc)
            return
        rospy.loginfo("MQTT 已连接 %s:%s", self.broker_host, self.broker_port)
        if self.enable_voice_commands:
            client.subscribe(self.services_topic, qos=self.qos)
        if self.enable_state_announcer:
            client.subscribe(self.osd_topic, qos=self.qos)

    @staticmethod
    def on_disconnect(_client, _userdata, rc, _properties=None):
        if rc:
            rospy.logwarn("MQTT 非正常断开，返回码 %s，将自动重连", rc)

    def on_message(self, _client, _userdata, message):
        envelope = {}
        try:
            envelope = json.loads(message.payload.decode("utf-8"))
            if not isinstance(envelope, dict):
                raise ValueError("消息不是 JSON 对象")
            if message.topic == self.osd_topic:
                self.latest_osd = envelope
                self.osd_event.set()
                return
            method = str(envelope.get("method", ""))
            if method not in SUPPORTED_METHODS:
                raise ValueError("不支持的 method: {}".format(method))
            tid = str(envelope.get("tid", "")).strip()
            if not tid:
                raise ValueError("缺少 tid")
            with self.cache_lock:
                cached = self.reply_cache.get(tid)
            if cached is not None:
                self.client.publish(self.reply_topic, cached, qos=self.qos)
                return
            self.jobs.put_nowait(envelope)
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError, queue.Full) as exc:
            rospy.logwarn("忽略无效 MQTT 消息 %s: %s", message.topic, exc)
            if message.topic != self.osd_topic:
                request = envelope if isinstance(envelope, dict) else {}
                reply = self.make_reply(str(request.get("tid", "")),
                                        str(request.get("method", "")),
                                        {"result": 1, "message": str(exc) or "命令队列已满"})
                self.client.publish(self.reply_topic, reply, qos=self.qos)
                rospy.loginfo("指令回执: %s", reply)

    def osd_worker(self):
        while not rospy.is_shutdown():
            if self.osd_event.wait(0.5):
                self.osd_event.clear()
                try:
                    self.handle_osd(self.latest_osd)
                except Exception as exc:
                    rospy.logwarn("状态处理失败: %s", exc)

    def handle_osd(self, envelope):
        data = envelope.get("data")
        if not isinstance(data, dict):
            return
        reported_code = str(data.get("robotCode", data.get("robot_code", "")))
        if reported_code and reported_code != self.robot_code:
            return
        state = classify_osd(data, self.linear_threshold, self.angular_threshold,
                             self.left_turn_positive)
        self.state_pub.publish(String(data=state))
        now = time.monotonic()
        changed = state != self.last_state
        repeat_due = (state in self.repeat_seconds
                      and now - self.last_play_time >= self.repeat_seconds[state])
        if changed:
            rospy.loginfo("MQTT OSD 语音状态: %s", state)
            self.last_state = state
        if state in STATE_AUDIO and (changed or repeat_due):
            try:
                self.api.play(STATE_AUDIO[state])
                rospy.loginfo("已播放 %s", STATE_AUDIO[state])
            except Exception as exc:
                rospy.logwarn("状态语音播放失败: %s", exc)
            self.last_play_time = now

    def make_reply(self, tid, method, data):
        envelope = {"tid": tid, "method": method,
                    "timestamp": int(time.time() * 1000), "data": data}
        return json.dumps(envelope, ensure_ascii=False, separators=(",", ":"))

    def handle_update(self, data):
        voices = data.get("voices")
        if not isinstance(voices, list) or not voices:
            raise ValueError("voices 必须是非空数组")
        current_names = {str(item.get("name")) for item in self.api.get_list()}
        created = []
        skipped = []
        for voice in voices:
            if not isinstance(voice, dict):
                raise ValueError("voices 元素必须是对象")
            filename = str(voice.get("name", "")).strip()
            text = str(voice.get("text", "")).strip()
            voice_type = str(voice.get("type", "Chinese")).strip()
            if (not filename.lower().endswith(".wav") or not text
                    or "/" in filename or "\\" in filename or "\x00" in filename):
                raise ValueError("语音 name 必须以 .wav 结尾且 text 不能为空")
            if filename in current_names:
                if filename in PROTECTED_REMOTE_FILES:
                    raise RuntimeError("固件保护语音不能覆盖: {}".format(filename))
                self.api.delete(filename)
            self.api.create(filename, text, voice_type)
            created.append(filename)
            current_names.add(filename)
        verified_names = {str(item.get("name")) for item in self.api.get_list()}
        missing = set(created) - verified_names
        if missing:
            raise RuntimeError("生成后列表缺少: {}".format(", ".join(sorted(missing))))
        return {"result": 0, "message": "success", "voices": created,
                "skipped": skipped}

    def handle_command(self, envelope):
        tid = str(envelope["tid"])
        method = str(envelope["method"])
        data = envelope.get("data") if isinstance(envelope.get("data"), dict) else {}
        target_code = str(data.get("robotCode", self.robot_code))
        if target_code != self.robot_code:
            raise ValueError("robotCode 与当前设备不一致")
        if method == "voice_package_update":
            reply_data = self.handle_update(data)
        elif method == "voice_package_list":
            voices = self.api.get_list()
            reply_data = {"result": 0, "message": "success",
                          "robotCode": self.robot_code, "robotName": self.robot_name,
                          "count": len(voices), "voices": voices}
        elif method == "voice_switch":
            filename = str(data.get("fileName", "")).strip()
            if not filename:
                raise ValueError("fileName 不能为空")
            self.api.play(filename)
            reply_data = {"result": 0, "message": "success"}
        else:
            raise ValueError("不支持的 method")
        return self.make_reply(tid, method, reply_data)

    def worker(self):
        while not rospy.is_shutdown():
            try:
                envelope = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            tid = str(envelope.get("tid", ""))
            method = str(envelope.get("method", ""))
            try:
                reply = self.handle_command(envelope)
            except Exception as exc:
                rospy.logerr("MQTT 语音命令失败 %s: %s", method, exc)
                reply = self.make_reply(tid, method, {"result": 1, "message": str(exc)})
            with self.cache_lock:
                self.reply_cache[tid] = reply
                while len(self.reply_cache) > 100:
                    self.reply_cache.popitem(last=False)
            self.client.publish(self.reply_topic, reply, qos=self.qos)
            rospy.loginfo("指令回执: %s", reply)
            self.jobs.task_done()

    def run(self):
        threading.Thread(target=self.worker, name="mqtt_voice_worker", daemon=True).start()
        threading.Thread(target=self.osd_worker, name="mqtt_osd_worker", daemon=True).start()
        self.client.connect_async(self.broker_host, self.broker_port, self.keepalive)
        self.client.loop_start()
        rospy.loginfo("订阅 services=%s osd=%s", self.services_topic, self.osd_topic)
        try:
            rospy.spin()
        finally:
            self.client.loop_stop()
            self.client.disconnect()


def main():
    rospy.init_node("mqtt_voice_bridge")
    try:
        MqttVoiceBridge().run()
    except Exception as exc:
        rospy.logfatal("MQTT 语音桥启动失败: %s", exc)
        raise


if __name__ == "__main__":
    main()
