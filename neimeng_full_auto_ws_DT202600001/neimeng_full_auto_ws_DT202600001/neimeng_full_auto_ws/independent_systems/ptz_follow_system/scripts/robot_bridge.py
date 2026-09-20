#!/usr/bin/env python3
import importlib.util
import math
import os
import threading
import time
from urllib.parse import quote

import requests
import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, parse_message


def _load_current_map_tasks():
    """Load the pure helper source, never Catkin's executable relay wrapper."""
    path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "catalog_rules.py")
    spec = importlib.util.spec_from_file_location(
        "robot_base_bridge_catalog_rules_source", path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load catalog rules from " + path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.current_map_tasks


current_map_tasks = _load_current_map_tasks()


class RobotApi:
    def __init__(self, base_url, timeout):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.session = requests.Session()

    def get(self, path):
        response = self.session.get(self.base_url + path, timeout=self.timeout)
        response.raise_for_status()
        return response.json()

    def post(self, path, body=None):
        response = self.session.post(
            self.base_url + path,
            json=body if body is not None else {},
            timeout=self.timeout,
        )
        response.raise_for_status()
        return response.json()

    def task_status(self):
        payload = self.get("/task_manager/get_task_status")
        data = payload.get("data", {}) if payload.get("result") else {}
        return data if isinstance(data, dict) else {}

    def start_task(self, name, loop_time=1):
        return self.post(
            "/task_manager/start_task_queue",
            {"name": name, "loop_time": int(loop_time)},
        )

    def pause_toggle(self):
        return self.post("/task_manager/pause_task_queue")

    def stop_task(self):
        return self.post("/task_manager/stop_task_queue")

    def move(self, speed_x, speed_z):
        return self.post("/cmd/move", {"speed_x": float(speed_x), "speed_z": float(speed_z)})

    def switch_map(self, map_name):
        return self.get("/map_manage/load_map?map_name=" + quote(str(map_name), safe=""))

    def voice_list(self):
        return self.get("/system_manager/get_wav_list")

    def play_voice(self, file_name):
        return self.post("/system_manager/play_wav_data", {"file_name": str(file_name)})

    def create_voice(self, text, name, language):
        return self.post(
            "/system_manager/create_wav_data",
            {"text": str(text), "name": str(name), "type": str(language)},
        )

    def delete_voice(self, file_name):
        return self.post("/system_manager/delete_wav_data", {"file_name": str(file_name)})

    def voice_task(self, file_name, command="start", loop=False, interval=5):
        voice_name = str(file_name)
        if voice_name.lower().endswith(".wav"):
            voice_name = voice_name[:-4]
        return self.post(
            "/task_manager/action",
            {
                "task_loop": bool(loop), "task_time": float(interval),
                "task_wav_file": voice_name, "task_cmd": str(command),
                "turn_wav": False, "stop_wav": False,
                "task_type": "ACTION_WAV_CONTROL",
            },
        )


class RobotBridge:
    def __init__(self):
        self.robot_code = str(rospy.get_param("~robot_code", "DT000000000"))
        host = rospy.get_param("~host", "192.168.2.100")
        port = int(rospy.get_param("~port", 7999))
        self.api = RobotApi(
            "http://{}:{}".format(host, port),
            float(rospy.get_param("~http_timeout", 3.0)),
        )
        self.pose_endpoint = rospy.get_param("~pose_endpoint", "/sensor_data/base_data")
        self.points_endpoint = rospy.get_param("~points_endpoint", "/pose_manage/get_pose")
        self.map_endpoint = rospy.get_param("~map_endpoint", "/map_manage/current_map_info")
        self.frequency = float(rospy.get_param("~frequency", 5.0))
        self.reload_interval = float(rospy.get_param("~reload_interval", 10.0))
        self.approach_radius = float(rospy.get_param("~approach_radius", 1.2))
        self.arrival_radius = float(rospy.get_param("~arrival_radius", 0.45))
        self.arrival_hold = float(rospy.get_param("~arrival_hold", 0.8))
        self.leave_radius = float(rospy.get_param("~leave_radius", 0.70))
        # true: 根据真实底盘位姿自动产生到点事件；false: 仅发布真实数据，事件由测试器注入。
        self.publish_point_events = bool(rospy.get_param("~publish_point_events", True))
        self.start_point_name = str(rospy.get_param("~start_point_name", "起始点")).strip()
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.5))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 1.0))
        self.allow_map_switch_while_busy = bool(rospy.get_param("~allow_map_switch_while_busy", False))
        self.map_switch_timeout = float(rospy.get_param("~map_switch_timeout", 12.0))
        self.manual_move_timeout = float(rospy.get_param("~manual_move_timeout", 0.8))
        self.start_point_keywords = [
            str(value).replace(" ", "").lower()
            for value in rospy.get_param(
                "~start_point_keywords",
                ["起始点", "起始", "充电点", "待命点", "原点"],
            )
            if str(value).strip()
        ]

        self.point_pub = rospy.Publisher(Topics.POINT_EVENT, String, queue_size=20)
        self.task_pub = rospy.Publisher(Topics.TASK_STATUS, String, queue_size=20)
        self.catalog_pub = rospy.Publisher(Topics.ROBOT_CATALOG, String, queue_size=2, latch=True)
        self.telemetry_pub = rospy.Publisher(Topics.ROBOT_TELEMETRY, String, queue_size=10)
        self.robot_result_pub = rospy.Publisher(Topics.ROBOT_RESULT, String, queue_size=20)
        self.health_pub = rospy.Publisher(Topics.HEALTH, String, queue_size=10)
        rospy.Subscriber(Topics.TASK_COMMAND, String, self._task_command, queue_size=20)
        rospy.Subscriber(Topics.ROBOT_COMMAND, String, self._robot_command, queue_size=30)

        self.points = []
        self.last_reload = 0.0
        self.prepared = set()
        self.arrival_candidate = None
        self.arrival_since = None
        self.active_point = None
        self.last_task_status = {}
        self.current_map = {}
        self.last_catalog_publish = 0.0
        self.api_lock = threading.Lock()
        self.move_timer = None
        self.move_generation = 0
        self.voice_loop_stop = None
        self.voice_loop_thread = None
        rospy.on_shutdown(self._shutdown_stop)

    @staticmethod
    def _float(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _extract_points(self, payload):
        found = []

        def walk(node):
            if isinstance(node, dict):
                name = node.get("name")
                if isinstance(name, str) and name.strip():
                    for pose in (node.get("pose"), node.get("position"), node.get("point"), node):
                        if not isinstance(pose, dict):
                            continue
                        x, y = self._float(pose.get("x")), self._float(pose.get("y"))
                        if x is not None and y is not None:
                            found.append({
                                "name": name.strip(), "x": x, "y": y,
                                "map": str(node.get("map", "")).strip(),
                            })
                            break
                for value in node.values():
                    walk(value)
            elif isinstance(node, list):
                for value in node:
                    walk(value)

        walk(payload)
        return list({item["name"]: item for item in found}.values())

    def _extract_pose(self, payload):
        candidates = []

        def walk(node, path=""):
            if isinstance(node, dict):
                x, y = self._float(node.get("x")), self._float(node.get("y"))
                if x is not None and y is not None:
                    lower = path.lower()
                    score = 100 if "robot_pose" in lower or "robotpose" in lower else 0
                    score += 50 if "pose" in lower else 0
                    score -= 100 if "speed" in lower or "velocity" in lower else 0
                    yaw = self._float(node.get("yaw", node.get("theta", 0.0))) or 0.0
                    candidates.append((score, (x, y, yaw)))
                for key, value in node.items():
                    walk(value, path + "/" + str(key))
            elif isinstance(node, list):
                for index, value in enumerate(node):
                    walk(value, path + "/" + str(index))

        walk(payload)
        return max(candidates, key=lambda item: item[0])[1] if candidates else None

    @staticmethod
    def _extract_sensor_state(payload):
        aliases = {
            "localization": ("local_state", "localization", "localization_state", "location_state"),
            "radar": ("radar_state", "laser_state", "lidar_state", "scan_state", "laser"),
            "imu": ("imu_state", "imu"),
            "battery": ("battery", "battery_soc", "soc", "power"),
            "obstacle": ("obstacle", "obstacle_state"),
            "emergency_stop": ("emergency_stop", "estop", "emergency"),
        }
        found = {}
        def walk(value):
            if isinstance(value, dict):
                for key, child in value.items():
                    lower = str(key).lower()
                    for target, names in aliases.items():
                        if target not in found and lower in names and isinstance(child, (str, int, float, bool)):
                            found[target] = child
                    walk(child)
            elif isinstance(value, list):
                for child in value:
                    walk(child)
        walk(payload)
        return found

    @staticmethod
    def _extract_map_name(payload):
        """兼容 current_map_info 返回dict、list、字符串以及 name/map_name 字段。"""
        candidates = []
        def walk(value, score=0):
            if isinstance(value, dict):
                for key, child in value.items():
                    lower = str(key).lower()
                    if lower in ("map_name", "current_map", "current_map_name") and isinstance(child, (str, int, float)):
                        candidates.append((score + 100, str(child).strip()))
                    elif lower == "name" and isinstance(child, (str, int, float)):
                        candidates.append((score + 50, str(child).strip()))
                    walk(child, score + (10 if "map" in lower else 0))
            elif isinstance(value, list):
                for child in value:
                    walk(child, score)
            elif isinstance(value, str) and value.strip() and score > 0:
                candidates.append((score, value.strip()))
        walk(payload)
        values = [item for item in candidates if item[1]]
        return max(values, key=lambda item: item[0])[1] if values else ""

    def _reload_points(self):
        if self.points and time.monotonic() - self.last_reload < self.reload_interval:
            return
        points_payload = self.api.get(self.points_endpoint)
        points = self._extract_points(points_payload)
        try:
            map_payload = self.api.get(self.map_endpoint)
            map_name = self._extract_map_name(map_payload)
            filtered = [p for p in points if not p["map"] or p["map"] == map_name]
            self.points = filtered or points
        except Exception:
            self.points = points
        self.last_reload = time.monotonic()
        rospy.loginfo("底盘巡航点已加载: %d", len(self.points))

    def _publish_point(self, event, point, distance):
        message = make_message(
            "point." + event,
            source="robot_bridge",
            event=event,
            point_name=point["name"],
            distance=round(float(distance), 3),
        )
        self.point_pub.publish(String(data=dumps(message)))

    def _nearest(self, x, y):
        if not self.points:
            return None, float("inf")
        point = min(self.points, key=lambda p: math.hypot(x - p["x"], y - p["y"]))
        return point, math.hypot(x - point["x"], y - point["y"])

    def _update_point_state(self, point, distance):
        if point is None:
            return
        name = point["name"]
        if name not in self.prepared and distance <= self.approach_radius:
            self.prepared.add(name)
            self._publish_point("approach", point, distance)

        if self.active_point:
            if self.active_point == name and distance > self.leave_radius:
                self._publish_point("leave", point, distance)
                self.active_point = None
                self.arrival_candidate = None
                self.arrival_since = None
            return

        if distance > self.arrival_radius:
            self.arrival_candidate = None
            self.arrival_since = None
            return
        if self.arrival_candidate != name:
            self.arrival_candidate = name
            self.arrival_since = time.monotonic()
            return
        if time.monotonic() - self.arrival_since >= self.arrival_hold:
            self.active_point = name
            self._publish_point("arrived", point, distance)
            normalized_name = name.replace(" ", "").lower()
            is_start_point = (
                name == self.start_point_name
                or any(keyword in normalized_name for keyword in self.start_point_keywords)
            )
            if is_start_point:
                self.prepared.clear()

    def _publish_task_status(self, status, command_id="", command_status=""):
        message = make_message(
            "task.status",
            source="robot_bridge",
            command_id=command_id,
            command_status=command_status,
            data=status,
        )
        self.task_pub.publish(String(data=dumps(message)))

    @staticmethod
    def _response_data(payload, default):
        if not isinstance(payload, dict) or not payload.get("result", False):
            return default
        data = payload.get("data", default)
        return data if isinstance(data, type(default)) else default

    def _publish_catalog(self):
        """按 当前地图 -> 当前地图任务 -> 轨道/巡航点 的顺序发布目录。"""
        current_map_payload = self.api.get(self.map_endpoint)
        current_map_value = current_map_payload.get("data", {}) if isinstance(current_map_payload, dict) else {}
        if isinstance(current_map_value, list):
            current_map = current_map_value[0] if current_map_value and isinstance(current_map_value[0], dict) else {"name": self._extract_map_name(current_map_payload)}
        elif isinstance(current_map_value, dict):
            current_map = current_map_value
        else:
            current_map = {"name": self._extract_map_name(current_map_payload)}
        current_map_name = self._extract_map_name(current_map_payload)
        self.current_map = current_map if isinstance(current_map, dict) else {}
        maps = self._response_data(self.api.get("/map_manage/get_map_info"), [])
        all_tasks = self._response_data(self.api.get("/task_manager/get_task_list"), [])
        tracks = self._response_data(self.api.get("/track_manage/get_track_graph"), [])
        raw_points = self._response_data(self.api.get(self.points_endpoint), [])
        try:
            voices = self._response_data(self.api.voice_list(), [])
        except Exception as exc:
            rospy.logwarn_throttle(30.0, "语音列表读取失败: %s", exc)
            voices = []
        try:
            task_report = self._response_data(self.api.get("/task_manager/get_task_report"), [])
        except Exception as exc:
            rospy.logwarn_throttle(30.0, "任务报告读取失败: %s", exc)
            task_report = []

        tasks = current_map_tasks(all_tasks, current_map)
        if current_map_name and all_tasks and not tasks:
            rospy.logwarn_throttle(
                30.0, "current map %s has no matching tasks; catalog remains empty for safety",
                current_map_name,
            )
        message = make_message(
            "robot.catalog", source="robot_bridge",
            data={
                "current_map": current_map,
                "maps": maps,
                "tasks": tasks,
                "all_tasks": all_tasks,
                "tracks": tracks,
                "points": raw_points,
                "voices": voices,
                "task_report": task_report,
            },
        )
        self.catalog_pub.publish(String(data=dumps(message)))
        self.last_catalog_publish = time.monotonic()

    def _task_command(self, raw):
        try:
            message = parse_message(raw)
            action = str(message.get("action", message.get("method", ""))).lower()
            data = message.get("data", {})
            command_id = message.get("command_id", message.get("tid", message.get("message_id", "")))
            with self.api_lock:
                status = self.api.task_status()
                state = str(status.get("task_state", ""))
                if action in ("start", "task.start"):
                    result = self.api.start_task(data.get("task_name", data.get("name", "")), data.get("loop_time", 1))
                elif action in ("pause", "task.pause"):
                    result = {"result": True, "data": "already paused"} if state == "STATE_PAUSE" else self.api.pause_toggle()
                elif action in ("resume", "continue", "task.resume"):
                    result = {"result": True, "data": "already running"} if state == "STATE_DOING" else self.api.pause_toggle()
                elif action in ("stop", "task.stop"):
                    result = self.api.stop_task()
                else:
                    raise ValueError("unsupported task action: " + action)
            ok = bool(result and result.get("result", False))
            self._publish_task_status(
                {"request_action": action, "response": result}, command_id,
                "success" if ok else "failed",
            )
        except Exception as exc:
            rospy.logerr("底盘任务命令失败: %s", exc)
            self._publish_task_status({"error": str(exc)}, command_id if 'command_id' in locals() else "", "failed")

    def _publish_robot_result(self, action, command_id, status, data=None):
        message = make_message(
            "robot.result", source="robot_bridge", action=action,
            command_id=command_id, status=status, data=data or {},
        )
        self.robot_result_pub.publish(String(data=dumps(message)))

    def _cancel_move_timer(self):
        self.move_generation += 1
        if self.move_timer is not None:
            self.move_timer.cancel()
            self.move_timer = None

    def _schedule_move_stop(self):
        self._cancel_move_timer()
        if self.manual_move_timeout <= 0:
            return
        generation = self.move_generation
        self.move_timer = threading.Timer(self.manual_move_timeout, self._watchdog_stop, args=(generation,))
        self.move_timer.daemon = True
        self.move_timer.start()

    def _watchdog_stop(self, generation):
        try:
            if generation != self.move_generation:
                return
            with self.api_lock:
                if generation != self.move_generation:
                    return
                self.api.move(0.0, 0.0)
            rospy.logwarn("车体手控续发超时，已自动发送零速")
        except Exception as exc:
            rospy.logwarn("车体自动停止失败: %s", exc)

    def _shutdown_stop(self):
        self._cancel_move_timer()
        self._stop_voice_loop()
        try:
            with self.api_lock:
                self.api.move(0.0, 0.0)
        except Exception:
            pass

    def _stop_voice_loop(self):
        if self.voice_loop_stop is not None:
            self.voice_loop_stop.set()
            self.voice_loop_stop = None

    def _start_voice_loop(self, file_name, interval):
        self._stop_voice_loop()
        stop_event = threading.Event()
        self.voice_loop_stop = stop_event
        wait_seconds = max(0.5, float(interval))

        def run():
            while not stop_event.is_set() and not rospy.is_shutdown():
                try:
                    with self.api_lock:
                        response = self.api.play_voice(file_name)
                    if not bool(response.get("result", False)):
                        rospy.logwarn("循环语音播放失败: %s", response)
                except Exception as exc:
                    rospy.logwarn("循环语音播放异常: %s", exc)
                stop_event.wait(wait_seconds)

        self.voice_loop_thread = threading.Thread(target=run, daemon=True)
        self.voice_loop_thread.start()

    def _robot_command(self, raw):
        """执行 UI/MQTT 共用的低层车体指令，并只以真实 HTTP 响应作为成功依据。"""
        action = ""
        command_id = ""
        try:
            message = parse_message(raw)
            action = str(message.get("action", message.get("method", ""))).lower()
            if action.startswith("robot."):
                action = action[6:]
            command_id = str(message.get("command_id", message.get("tid", message.get("message_id", ""))))
            data = message.get("data", {})
            with self.api_lock:
                if action == "move":
                    speed_x = max(-self.max_linear_speed, min(self.max_linear_speed, float(data.get("speed_x", 0.0))))
                    speed_z = max(-self.max_angular_speed, min(self.max_angular_speed, float(data.get("speed_z", 0.0))))
                    result = self.api.move(speed_x, speed_z)
                    self._schedule_move_stop()
                elif action == "stop":
                    self._cancel_move_timer()
                    result = self.api.move(0.0, 0.0)
                elif action in ("map.switch", "switch_map"):
                    map_name = str(data.get("map_name", data.get("name", ""))).strip()
                    if not map_name:
                        raise ValueError("map_name is required")
                    status = self.api.task_status()
                    if not self.allow_map_switch_while_busy and str(status.get("task_state", "")) in ("STATE_DOING", "STATE_PAUSE"):
                        raise ValueError("task is active; stop it before switching map")
                    result = self.api.switch_map(map_name)
                    if bool(result.get("result", False)):
                        deadline = time.monotonic() + max(0.0, self.map_switch_timeout)
                        actual_map = ""
                        while not rospy.is_shutdown() and time.monotonic() <= deadline:
                            actual_map = self._extract_map_name(self.api.get(self.map_endpoint))
                            if actual_map == map_name:
                                break
                            time.sleep(0.5)
                        if actual_map != map_name:
                            raise RuntimeError("map switch not confirmed: requested={}, actual={}".format(map_name, actual_map or "unknown"))
                        self.points = []
                        self.last_reload = 0.0
                        self._reload_points()
                        self._publish_catalog()
                elif action in ("catalog.refresh", "refresh"):
                    self.last_catalog_publish = 0.0
                    self._publish_catalog()
                    result = {"result": True, "data": "catalog refreshed"}
                elif action in ("voice.play", "play_voice"):
                    file_name = str(data.get("file_name", data.get("name", ""))).strip()
                    if not file_name:
                        raise ValueError("file_name is required")
                    result = self.api.play_voice(file_name)
                elif action in ("voice.create", "create_voice"):
                    text = str(data.get("text", "")).strip()
                    name = str(data.get("name", "")).strip()
                    language = str(data.get("type", data.get("language", "Chinese"))).strip()
                    if not text or not name:
                        raise ValueError("voice text and name are required")
                    result = self.api.create_voice(text, name, language)
                    if bool(result.get("result", False)):
                        self._publish_catalog()
                elif action in ("voice.delete", "delete_voice"):
                    file_name = str(data.get("file_name", data.get("name", ""))).strip()
                    if not file_name:
                        raise ValueError("file_name is required")
                    result = self.api.delete_voice(file_name)
                    if bool(result.get("result", False)):
                        if self.voice_loop_stop is not None:
                            self._stop_voice_loop()
                        self._publish_catalog()
                elif action in ("voice.task", "voice.stop"):
                    file_name = str(data.get("file_name", data.get("name", ""))).strip()
                    if not file_name:
                        raise ValueError("file_name is required")
                    if action == "voice.stop" or str(data.get("task_cmd", "start")).lower() == "stop":
                        self._stop_voice_loop()
                        result = {"result": True, "data": "voice loop stopped"}
                    else:
                        self._start_voice_loop(file_name, data.get("task_time", 5))
                        result = {"result": True, "data": "voice loop started", "file_name": file_name}
                elif action in ("obstacle_avoid", "emergency_stop"):
                    # 一个命令内同时停止任务与底盘速度，最终回执以两个真实HTTP调用为准。
                    self._cancel_move_timer()
                    task_result = self.api.stop_task()
                    move_result = self.api.move(0.0, 0.0)
                    result = {
                        "result": bool(task_result.get("result", False)) and bool(move_result.get("result", False)),
                        "data": {"task_stop": task_result, "motion_stop": move_result},
                    }
                else:
                    raise ValueError("unsupported robot action: " + action)
            ok = bool(result and result.get("result", False))
            self._publish_robot_result(action, command_id, "success" if ok else "failed", {"response": result})
        except Exception as exc:
            rospy.logerr("车体命令失败: %s", exc)
            # 手控异常时尽力发零速，避免网络短暂异常后保持运动。
            if action == "move":
                try:
                    with self.api_lock:
                        self.api.move(0.0, 0.0)
                except Exception:
                    pass
            self._publish_robot_result(action, command_id, "failed", {"error": str(exc)})

    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            try:
                with self.api_lock:
                    self._reload_points()
                    pose_payload = self.api.get(self.pose_endpoint)
                    pose = self._extract_pose(pose_payload)
                    sensors = self._extract_sensor_state(pose_payload)
                    status = self.api.task_status()
                    if time.monotonic() - self.last_catalog_publish >= self.reload_interval:
                        self._publish_catalog()
                self.last_task_status = status
                self._publish_task_status(status)
                body = pose_payload.get("data", pose_payload) if isinstance(pose_payload, dict) else {}
                telemetry = dict(body) if isinstance(body, dict) else {}
                telemetry["robotCode"] = self.robot_code
                telemetry.setdefault("status", "0")
                telemetry.setdefault("heartbeat", {"online": True})
                if pose and not isinstance(telemetry.get("location"), dict):
                    telemetry["location"] = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
                if sensors and not isinstance(telemetry.get("sensor"), dict):
                    telemetry["sensor"] = sensors
                if status:
                    telemetry["task"] = status
                if self.current_map and not isinstance(telemetry.get("map"), dict):
                    telemetry["map"] = self.current_map
                self.telemetry_pub.publish(String(data=dumps(make_message(
                    "robot.telemetry", source="robot_bridge", robot_code=self.robot_code,
                    data=telemetry,
                ))))
                if pose:
                    point, distance = self._nearest(pose[0], pose[1])
                    if self.publish_point_events:
                        self._update_point_state(point, distance)
                    health = make_message(
                        "health.robot", source="robot_bridge", state="online",
                        data={
                            "pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
                            "points": len(self.points),
                            "point_event_mode": "real_pose" if self.publish_point_events else "external_test_signal",
                            "sensors": sensors,
                        },
                    )
                    self.health_pub.publish(String(data=dumps(health)))
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "底盘读取失败: %s", exc)
                health = make_message("health.robot", source="robot_bridge", state="offline", data={"error": str(exc)})
                self.health_pub.publish(String(data=dumps(health)))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_bridge")
    RobotBridge().run()
