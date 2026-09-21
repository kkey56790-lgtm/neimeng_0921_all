#!/usr/bin/env python3
"""启动 jieguo 任务截图和文字上传节点。

本文件是独立 ROS 节点，不修改也不导入现有任务编排或 YOLO 节点。它只读取：

* /inspection/status：当前任务名称和任务 ID；
* /inspection/point_event：真实巡航点 arrived 事件；
* YOLO_ZMQ_ADDRESS：现有 YOLO 逐帧结构化检测结果。

当前入口使用 jieguo_uploader.py：启动补传，每张截图附带任务、识别项、时间、点位、机器人编码，
获取中台凭证后PUT图片并通知上传结果，再清理本地文件。
下方保留的原点位类和工具函数供旧实时上传程序兼容导入。
"""

import argparse
import ast
import base64
import collections
import copy
import json
import os
import re
import signal
import sys
import threading
import time
import uuid
from urllib.parse import quote

try:
    import yaml
except ImportError:  # pragma: no cover - 仅在部署环境缺依赖时触发
    yaml = None

try:
    import rospy
    from std_msgs.msg import String
except ImportError:  # 允许 --self-test 和静态检查在非 ROS 环境运行
    rospy = None
    String = None

try:
    import zmq
except ImportError:  # pragma: no cover - 仅在部署环境缺依赖时触发
    zmq = None

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover - 仅在部署环境缺依赖时触发
    mqtt = None

try:
    import cv2
except ImportError:  # pragma: no cover - 仅在部署环境缺依赖时触发
    cv2 = None


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.dirname(SCRIPT_DIR)
WORKSPACE_DIR = os.path.dirname(os.path.dirname(MODULE_DIR))
BRINGUP_CONFIG_DIR = os.path.join(WORKSPACE_DIR, "src", "neimeng_bringup", "config")

DEFAULT_MQTT_CONFIG = os.path.join(BRINGUP_CONFIG_DIR, "mqtt.yaml")
DEFAULT_MQTT_CREDENTIALS = os.path.join(BRINGUP_CONFIG_DIR, "mqtt_credentials.yaml")
DEFAULT_DEVICE_CONFIG = os.path.join(BRINGUP_CONFIG_DIR, "device.yaml")
DEFAULT_YOLO_CONFIG = os.path.join(MODULE_DIR, "config", "full_auto_yolo.yaml")
DEFAULT_YOLO_ENV = os.path.join(MODULE_DIR, "config", "yolo.env")
DEFAULT_DETECTOR_FILE = os.path.join(MODULE_DIR, "detector", "rtsp_truck.py")
DEFAULT_RESULTS_ROOT = os.path.join(WORKSPACE_DIR, "jieguo")

# 中台默认连接参数。现场如需临时切换，可使用 ROBOT_MQTT_* 环境变量，
# 或 --mqtt-host/--mqtt-port 参数覆盖。
DEFAULT_MQTT_HOST = "222.187.130.102"
DEFAULT_MQTT_PORT = 1883
DEFAULT_MQTT_USERNAME = "autocar"
DEFAULT_MQTT_PASSWORD = "123456"

FALLBACK_CLASSES = [
    "chocks", "tyre", "extinguisher", "plate", "tag",
    "light", "support", "screw", "tank", "truck",
]
FALLBACK_CLASS_NAMES_CN = [
    "挡掩", "轮胎", "灭火器", "车牌", "检修牌",
    "车灯", "支护", "轮毂螺丝", "油箱", "卡车",
]

STATUS_TOPIC = "/inspection/status"
POINT_EVENT_TOPIC = "/inspection/point_event"
INACTIVE_TASK_STATES = {"", "IDLE", "FINISHED", "STOPPED"}
MANIFEST_NAME = "task_manifest.json"


def now_ms():
    return int(time.time() * 1000)


def json_load(raw):
    if hasattr(raw, "data"):
        raw = raw.data
    value = json.loads(raw) if isinstance(raw, str) else raw
    if not isinstance(value, dict):
        raise ValueError("消息根节点必须是 JSON object")
    return value


def read_yaml(path):
    if yaml is None:
        raise RuntimeError("缺少 PyYAML，请安装 python3-yaml")
    if not path or not os.path.isfile(path):
        return {}
    with open(path, "r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream) or {}
    if not isinstance(value, dict):
        raise ValueError("YAML 根节点必须是 object: " + path)
    return value


def resolve_local_file(requested, filename):
    """优先用指定路径；复制成单文件运行时再搜索脚本旁边的配置。"""
    candidates = [requested]
    for base in (os.getcwd(), SCRIPT_DIR, os.path.dirname(SCRIPT_DIR)):
        candidates.append(os.path.join(base, filename))
        candidates.append(os.path.join(base, "config", filename))
    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            return os.path.abspath(candidate)
    return str(requested or "")


def infer_robot_code(*values):
    """从显式值、环境变量、脚本路径或当前目录中识别 DT 设备编号。"""
    pattern = re.compile(r"(?<![A-Za-z0-9])DT\d{6,}(?!\d)", re.IGNORECASE)
    for value in values:
        text = str(value or "").strip()
        if not text:
            continue
        match = pattern.search(text)
        if match:
            return match.group(0).upper()
    return ""


def read_env_file(path):
    values = {}
    if not path or not os.path.isfile(path):
        return values
    with open(path, "r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip().strip("'\"")
    return values


def safe_filename(value, fallback="item", maximum=80):
    text = str(value or "").strip()
    for character in '<>:"/\\|?*\0\r\n\t':
        text = text.replace(character, "_")
    text = text.strip(" ._")
    return (text or fallback)[:maximum]


def atomic_write_bytes(path, data):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp-" + uuid.uuid4().hex
    with open(temporary, "wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def atomic_write_json(path, value):
    atomic_write_bytes(
        path,
        json.dumps(value, ensure_ascii=False, indent=2).encode("utf-8"),
    )


def build_rtsp_url(explicit_url, env_file):
    if explicit_url:
        return str(explicit_url)
    values = read_env_file(env_file)

    def setting(name, default):
        return str(os.environ.get(name) or values.get(name) or default)

    username = quote(setting("RTSP_USERNAME", "admin"), safe="")
    password = quote(setting("RTSP_PASSWORD", "okwy1688"), safe="")
    host = setting("RTSP_HOST", "192.168.2.64")
    port = int(setting("RTSP_PORT", "554"))
    channel = setting("RTSP_CHANNEL", "102")
    return "rtsp://{}:{}@{}:{}/Streaming/Channels/{}".format(
        username, password, host, port, channel)


def payload_with_image(payload, image_path):
    result = copy.deepcopy(payload)
    if image_path and os.path.isfile(image_path):
        with open(image_path, "rb") as stream:
            encoded = base64.b64encode(stream.read()).decode("ascii")
        result["data"]["imageName"] = os.path.basename(image_path)
        result["data"]["imageMimeType"] = "image/jpeg"
        result["data"]["imageBase64"] = encoded
    else:
        result["data"]["imageName"] = ""
        result["data"]["imageMimeType"] = ""
        result["data"]["imageBase64"] = ""
    return result


def _literal_lists_from_python(path):
    """只用 AST 读取类别常量，避免导入带 NPU/RTSP 副作用的检测程序。"""
    with open(path, "r", encoding="utf-8") as stream:
        tree = ast.parse(stream.read(), filename=path)
    result = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name):
            continue
        try:
            value = ast.literal_eval(node.value)
        except (ValueError, TypeError):
            continue
        if isinstance(value, list) and all(isinstance(item, str) for item in value):
            result[target.id] = value
    return result


def resolve_class_profile(requested, env_file):
    profile = str(requested or "auto").strip().lower()
    if profile != "auto":
        return profile
    file_env = read_env_file(env_file)
    profile = str(
        os.environ.get("YOLO_CLASS_PROFILE")
        or file_env.get("YOLO_CLASS_PROFILE")
        or ""
    ).strip().lower()
    if profile:
        return profile
    backend = str(
        os.environ.get("YOLO_BACKEND") or file_env.get("YOLO_BACKEND") or "rknn"
    ).strip().lower()
    return "coco80" if backend in ("coco", "coco80", "yolov8") else "truck10"


def load_detection_items(detector_file, profile="auto", env_file=DEFAULT_YOLO_ENV,
                         explicit_items=""):
    """返回 [{id, name, display_name}, ...]，保证零检测时也有完整项目表。"""
    if explicit_items:
        names = [item.strip() for item in explicit_items.split(",") if item.strip()]
        if not names:
            raise ValueError("--detection-items 不能为空")
        return [
            {"id": index, "name": name, "display_name": name}
            for index, name in enumerate(names)
        ]

    constants = (
        _literal_lists_from_python(detector_file)
        if detector_file and os.path.isfile(detector_file)
        else {
            "CLASSES": FALLBACK_CLASSES,
            "CLASS_NAMES_CN": FALLBACK_CLASS_NAMES_CN,
        }
    )
    resolved = resolve_class_profile(profile, env_file)
    if resolved == "inspection11":
        names = constants.get("INSPECTION11_CLASSES", [])
        display_names = constants.get("INSPECTION11_CLASS_NAMES_CN", names)
    elif resolved in ("coco", "coco80", "yolov8"):
        names = constants.get("COCO_CLASSES", [])
        display_names = names
    else:
        names = constants.get("CLASSES", [])
        display_names = constants.get("CLASS_NAMES_CN", names)
    if not names:
        raise ValueError("无法从检测程序读取类别列表: " + detector_file)
    if len(display_names) != len(names):
        display_names = names
    return [
        {"id": index, "name": name, "display_name": display_names[index]}
        for index, name in enumerate(names)
    ]


def build_event_payload(robot_code, task_id, task_name, point_name, items, counts,
                        frame_count, collection_state, timestamp_ms=None):
    """按文件上传协议生成请求，并在content中保留检测文字格式。"""
    timestamp_ms = int(timestamp_ms or now_ms())
    detection_items = [
        {
            "name": item["display_name"],
            "count": int(counts.get(item["name"], 0)),
        }
        for item in items
    ]
    content_object = collections.OrderedDict([
        ("taskName", str(task_name)),
        ("detectionItems", detection_items),
        ("pointName", str(point_name)),
    ])
    content = json.dumps(
        content_object, ensure_ascii=False, separators=(",", ":"))
    data = collections.OrderedDict([
        ("robotCode", str(robot_code)),
        ("stationCode", str(task_id or task_name)),
        ("stationName", str(task_name)),
        ("objectId", "{}-{}".format(timestamp_ms, robot_code)),
        ("content", content),
        ("expireMinutes", 30),
        ("taskName", str(task_name)),
        ("detectionItems", detection_items),
        ("pointName", str(point_name)),
        ("frameCount", int(frame_count)),
        ("detectionState", str(collection_state)),
    ])
    return collections.OrderedDict([
        ("tid", "file-{}".format(uuid.uuid4().hex)),
        ("method", "file_upload_request"),
        ("timestamp", timestamp_ms),
        ("data", data),
    ])


class MqttOutbox:
    """QoS 1 顺序发送队列；仅在 PUBACK 后清理本地结果。"""

    def __init__(self, config, credentials, robot_code, dry_run=False):
        self.config = config
        self.robot_code = robot_code
        self.dry_run = bool(dry_run)
        self.topic = str(
            config.get("topics", {}).get(
                "file", "thing/robot/{robot_code}/file")
        ).format(robot_code=robot_code)
        self.maximum = max(1, int(config.get("max_outbox", 10000)))
        self.items = collections.deque()
        self.condition = threading.Condition()
        self.connected = False
        self.stopping = False
        self.client = None

        if self.dry_run:
            return
        if mqtt is None:
            raise RuntimeError("缺少 paho-mqtt，请安装 python3-paho-mqtt")

        client_id = str(config.get("client_id", "{robot_code}-inspection")).format(
            robot_code=robot_code)
        try:
            self.client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION1,
                client_id=client_id,
                protocol=mqtt.MQTTv311,
            )
        except (AttributeError, TypeError):
            self.client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)

        authentication = credentials.get("authentication", {})
        username_env = str(
            authentication.get("username_env", "ROBOT_MQTT_USERNAME")).strip()
        password_env = str(
            authentication.get("password_env", "ROBOT_MQTT_PASSWORD")).strip()
        username = os.environ.get(username_env, "") if username_env else ""
        password = os.environ.get(password_env, "") if password_env else ""
        username = username or str(
            authentication.get("username", DEFAULT_MQTT_USERNAME))
        password = password or str(
            authentication.get("password", DEFAULT_MQTT_PASSWORD))
        if username:
            self.client.username_pw_set(username, password)

        tls = config.get("tls", {})
        if bool(tls.get("enabled", False)):
            ca_file = str(tls.get("ca_file", "")).strip() or None
            self.client.tls_set(ca_certs=ca_file)

        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        if hasattr(self, "on_message"):
            self.client.on_message = self.on_message
        if hasattr(self, "on_subscribe"):
            self.client.on_subscribe = self.on_subscribe
        if hasattr(self, "_on_connect_fail"):
            self.client.on_connect_fail = self._on_connect_fail
        broker = config.get("broker", {})
        self.client.connect_async(
            str(broker.get("host", DEFAULT_MQTT_HOST)),
            int(broker.get("port", DEFAULT_MQTT_PORT)),
            int(broker.get("keepalive", 30)),
        )
        self.client.loop_start()

    def _on_connect(self, _client, _userdata, _flags, rc, _properties=None):
        with self.condition:
            self.connected = int(rc) == 0
            self.condition.notify_all()
        if rospy is not None:
            if self.connected:
                rospy.loginfo("MQTT已连接，检测结果主题: %s", self.topic)
            else:
                rospy.logerr("MQTT连接失败 rc=%s", rc)

    def _on_disconnect(self, _client, _userdata, rc, _properties=None):
        with self.condition:
            self.connected = False
            self.condition.notify_all()
        if rospy is not None and int(rc) != 0:
            rospy.logwarn("MQTT异常断开 rc=%s，将继续重连", rc)

    def enqueue(self, payload, cleanup_paths=None, cleanup_dir=""):
        text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        if self.dry_run:
            print(text, flush=True)
            return True
        item = {
            "text": text,
            "cleanup_paths": list(cleanup_paths or []),
            "cleanup_dir": str(cleanup_dir or ""),
        }
        with self.condition:
            if len(self.items) >= self.maximum:
                if rospy is not None:
                    rospy.logerr("MQTT待发队列已满，拒绝丢弃新的点位结果")
                return False
            self.items.append(item)
            self.condition.notify_all()
        return True

    @staticmethod
    def _cleanup_confirmed(item):
        for path in item.get("cleanup_paths", []):
            try:
                if path and os.path.isfile(path):
                    os.remove(path)
            except OSError as exc:
                if rospy is not None:
                    rospy.logwarn("MQTT已确认，但删除本地文件失败 %s: %s", path, exc)
        directory = item.get("cleanup_dir", "")
        if not directory or not os.path.isdir(directory):
            return
        try:
            remaining = [
                name for name in os.listdir(directory)
                if name != MANIFEST_NAME
            ]
            if not remaining:
                manifest = os.path.join(directory, MANIFEST_NAME)
                if os.path.isfile(manifest):
                    os.remove(manifest)
                os.rmdir(directory)
        except OSError as exc:
            if rospy is not None:
                rospy.logwarn("清理已上传任务目录失败 %s: %s", directory, exc)

    def run(self):
        while True:
            with self.condition:
                while not self.stopping and (not self.items or not self.connected):
                    self.condition.wait(timeout=1.0)
                if self.stopping:
                    return
                item = self.items[0]
                text = item["text"]
            try:
                info = self.client.publish(self.topic, text, qos=1, retain=False)
                if info.rc != mqtt.MQTT_ERR_SUCCESS:
                    raise RuntimeError("publish rc={}".format(info.rc))
                info.wait_for_publish(timeout=10.0)
                if not info.is_published():
                    raise RuntimeError("等待 MQTT PUBACK 超时")
                with self.condition:
                    if self.items and self.items[0] is item:
                        self.items.popleft()
                self._cleanup_confirmed(item)
                if rospy is not None:
                    rospy.loginfo("MQTT PUBACK已收到，本地点位图片和结果已清理")
            except Exception as exc:
                if rospy is not None:
                    rospy.logwarn_throttle(5.0, "MQTT结果发送失败，将重试: %s", exc)
                time.sleep(1.0)

    def stop(self):
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
        if self.client is not None:
            self.client.loop_stop()
            self.client.disconnect()


class PointCollection:
    def __init__(self, task_id, task_name, point_name, items, duration):
        self.task_id = str(task_id)
        self.task_name = str(task_name)
        self.point_name = str(point_name)
        self.deadline = time.monotonic() + max(0.05, float(duration))
        self.frame_count = 0
        self.counts = {item["name"]: 0 for item in items}
        self.best_score = 0
        self.best_objects = []
        self.best_image = None


class YoloPointMqttUploader:
    def __init__(self, args):
        mqtt_config_path = resolve_local_file(args.mqtt_config, "mqtt.yaml")
        mqtt_credentials_path = resolve_local_file(
            args.mqtt_credentials, "mqtt_credentials.yaml")
        device_config_path = resolve_local_file(args.device_config, "device.yaml")
        yolo_config_path = resolve_local_file(args.yolo_config, "full_auto_yolo.yaml")
        yolo_env_path = resolve_local_file(args.yolo_env, "yolo.env")
        detector_file = resolve_local_file(args.detector_file, "rtsp_truck.py")

        mqtt_config = read_yaml(mqtt_config_path)
        mqtt_credentials = read_yaml(mqtt_credentials_path)
        device_config = read_yaml(device_config_path)
        yolo_config = read_yaml(yolo_config_path)
        authentication = mqtt_credentials.setdefault("authentication", {})
        authentication["username"] = str(args.mqtt_username)
        authentication["password"] = str(args.mqtt_password)
        broker = mqtt_config.setdefault("broker", {})
        mqtt_host = str(args.mqtt_host or os.environ.get("ROBOT_MQTT_HOST") or "").strip()
        mqtt_port = args.mqtt_port or os.environ.get("ROBOT_MQTT_PORT")
        if mqtt_host:
            broker["host"] = mqtt_host
        if mqtt_port:
            broker["port"] = int(mqtt_port)
        if args.file_topic:
            mqtt_config.setdefault("topics", {})["file"] = args.file_topic

        self.robot_code = infer_robot_code(
            args.robot_code,
            os.environ.get("ROBOT_CODE"),
            device_config.get("robot_code"),
            os.path.abspath(__file__),
            os.getcwd(),
        )
        if not self.robot_code:
            raise ValueError(
                "无法确定robot_code；请使用 --robot-code DT202600001，"
                "或设置ROBOT_CODE环境变量/提供device.yaml")

        self.items = load_detection_items(
            detector_file,
            profile=args.class_profile,
            env_file=yolo_env_path,
            explicit_items=args.detection_items,
        )
        self.min_confidence = float(
            args.min_confidence
            if args.min_confidence is not None
            else yolo_config.get("min_confidence", 0.35)
        )
        self.collection_seconds = float(
            args.collection_seconds
            if args.collection_seconds is not None
            else yolo_config.get("collect_seconds", 6.0)
        )
        self.zmq_address = str(
            args.zmq_address
            or os.environ.get("YOLO_ZMQ_ADDRESS")
            or yolo_config.get("zmq_address")
            or "tcp://127.0.0.1:5566"
        )
        self.point_keywords = [
            value.strip() for value in args.point_keywords.split(",") if value.strip()
        ]
        self.results_root = os.path.abspath(args.results_root)
        self.screenshots_enabled = not bool(args.disable_screenshots)
        self.image_max_width = max(320, int(args.image_max_width))
        self.jpeg_quality = min(100, max(30, int(args.jpeg_quality)))
        self.rtsp_url = build_rtsp_url(args.rtsp_url, yolo_env_path)
        os.makedirs(self.results_root, exist_ok=True)

        self.lock = threading.RLock()
        self.current_task_id = ""
        self.current_task_name = ""
        self.current_task_identity = ""
        self.task_active = False
        self.uploaded_points = set()
        self.active_collection = None
        self.collection_timer = None
        self.task_dir = ""
        self.latest_rtsp_frame = None
        self.stopping = False

        self.outbox = getattr(self, "outbox_class", MqttOutbox)(
            mqtt_config, mqtt_credentials, self.robot_code, dry_run=args.dry_run)
        self.outbox_thread = threading.Thread(target=self.outbox.run, daemon=True)
        self.outbox_thread.start()
        self._recover_ready_tasks()

        self.zmq_thread = threading.Thread(target=self._receive_yolo, daemon=True)
        self.zmq_thread.start()
        self.rtsp_thread = threading.Thread(target=self._receive_rtsp, daemon=True)
        self.rtsp_thread.start()

        rospy.Subscriber(STATUS_TOPIC, String, self._status_callback, queue_size=20)
        rospy.Subscriber(POINT_EVENT_TOPIC, String, self._point_callback, queue_size=50)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "独立YOLO任务归档上传已启动: robot=%s classes=%d window=%.2fs zmq=%s root=%s",
            self.robot_code, len(self.items), self.collection_seconds,
            self.zmq_address, self.results_root,
        )

    def _manifest_path(self, directory=None):
        return os.path.join(directory or self.task_dir, MANIFEST_NAME)

    def _write_manifest(self, state):
        if not self.task_dir:
            return
        atomic_write_json(self._manifest_path(), {
            "taskId": self.current_task_id,
            "taskName": self.current_task_name,
            "taskIdentity": self.current_task_identity,
            "state": str(state),
            "updatedMs": now_ms(),
        })

    def _open_task_directory(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        directory_name = "{}_{}_{}".format(
            safe_filename(self.current_task_name, "task"),
            safe_filename(self.current_task_id or "no-id", "no-id", 48),
            stamp,
        )
        self.task_dir = os.path.join(self.results_root, directory_name)
        os.makedirs(self.task_dir, exist_ok=True)
        self._write_manifest("COLLECTING")
        rospy.loginfo("已建立当前任务识别目录: %s", self.task_dir)

    def _enqueue_result_file(self, result_path):
        try:
            with open(result_path, "r", encoding="utf-8") as stream:
                record = json.load(stream)
            payload = record.get("payload")
            if not isinstance(payload, dict):
                raise ValueError("result payload不是object")
            directory = os.path.dirname(result_path)
            image_name = os.path.basename(str(record.get("imageFile", "")))
            image_path = os.path.join(directory, image_name) if image_name else ""
            upload_payload = payload_with_image(payload, image_path)
            cleanup_paths = [result_path]
            if image_path:
                cleanup_paths.append(image_path)
            return self.outbox.enqueue(
                upload_payload,
                cleanup_paths=cleanup_paths,
                cleanup_dir=directory,
            )
        except Exception as exc:
            rospy.logerr("读取待上传结果失败 %s: %s", result_path, exc)
            return False

    def _enqueue_task_directory(self, directory):
        queued = 0
        for name in sorted(os.listdir(directory)):
            if not name.endswith(".result.json"):
                continue
            if self._enqueue_result_file(os.path.join(directory, name)):
                queued += 1
        if queued == 0:
            try:
                manifest = os.path.join(directory, MANIFEST_NAME)
                if os.path.isfile(manifest):
                    os.remove(manifest)
                if not os.listdir(directory):
                    os.rmdir(directory)
            except OSError:
                pass
        return queued

    def _recover_ready_tasks(self):
        """重启后继续上传已结束但尚未收到 PUBACK 的任务结果。"""
        for name in sorted(os.listdir(self.results_root)):
            directory = os.path.join(self.results_root, name)
            manifest_path = os.path.join(directory, MANIFEST_NAME)
            if not os.path.isdir(directory) or not os.path.isfile(manifest_path):
                continue
            try:
                with open(manifest_path, "r", encoding="utf-8") as stream:
                    manifest = json.load(stream)
                if str(manifest.get("state", "")).upper() != "READY_TO_UPLOAD":
                    continue
                count = self._enqueue_task_directory(directory)
                if count:
                    rospy.loginfo("恢复待上传任务目录: %s，结果数=%d", directory, count)
            except Exception as exc:
                rospy.logwarn("恢复任务目录失败 %s: %s", directory, exc)

    def _close_current_task_locked(self, reason):
        if not self.task_dir:
            return
        if self.active_collection is not None:
            self._finish_collection_locked("task_finished")
        self._write_manifest("READY_TO_UPLOAD")
        directory = self.task_dir
        self.task_dir = ""
        queued = self._enqueue_task_directory(directory)
        rospy.loginfo(
            "任务结束，开始上传归档结果: task=%s reason=%s results=%d",
            self.current_task_name, reason, queued,
        )

    def _begin_task(self, task_id, task_name, state):
        task_id = str(task_id or "").strip()
        task_name = str(task_name or "").strip()
        state = str(state or "").upper()
        active = state not in INACTIVE_TASK_STATES
        if not active:
            if self.task_active:
                self._close_current_task_locked(state or "inactive")
            self.task_active = False
            return
        if not task_name:
            return
        identity = task_id or task_name
        if not self.task_active or identity != self.current_task_identity:
            if self.task_active:
                self._close_current_task_locked("task_changed")
            self.uploaded_points.clear()
            self.current_task_identity = identity
            self.current_task_id = task_id
            self.current_task_name = task_name
            self._open_task_directory()
            rospy.loginfo("检测到新任务，重置点位归档记录: %s", task_name)
        else:
            self.current_task_id = task_id
            self.current_task_name = task_name
        self.task_active = True

    def _status_callback(self, raw):
        try:
            message = json_load(raw)
            with self.lock:
                self._begin_task(
                    message.get("task_id"), message.get("task_name"), message.get("state"))
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "巡检状态解析失败: %s", exc)

    def _point_is_selected(self, point_name):
        return not self.point_keywords or any(
            keyword in point_name for keyword in self.point_keywords)

    def _point_callback(self, raw):
        try:
            message = json_load(raw)
            if str(message.get("event", "")).lower() != "arrived":
                return
            point_name = str(message.get("point_name", "")).strip()
            if not point_name or not self._point_is_selected(point_name):
                return
            with self.lock:
                if not self.task_active or not self.current_task_name:
                    rospy.logwarn("收到到点事件但当前任务未激活，暂不上报: %s", point_name)
                    return
                point_key = (self.current_task_identity, point_name)
                if point_key in self.uploaded_points:
                    return
                if self.active_collection is not None:
                    self._finish_collection_locked("next_point_arrived")
                self.uploaded_points.add(point_key)
                self.active_collection = PointCollection(
                    self.current_task_id, self.current_task_name, point_name,
                    self.items, self.collection_seconds,
                )
                self.collection_timer = threading.Timer(
                    self.collection_seconds, self._finish_collection)
                self.collection_timer.daemon = True
                self.collection_timer.start()
                rospy.loginfo("开始汇总巡航点检测: %s", point_name)
        except Exception as exc:
            rospy.logwarn("点位事件解析失败: %s", exc)

    def _receive_rtsp(self):
        if not self.screenshots_enabled:
            return
        if cv2 is None:
            rospy.logerr("缺少 OpenCV，文字结果仍会保存上传，但无法保存识别截图")
            return
        capture = None
        while not self.stopping and not rospy.is_shutdown():
            try:
                if capture is None or not capture.isOpened():
                    if capture is not None:
                        capture.release()
                    capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    if not capture.isOpened():
                        capture.release()
                        capture = None
                        rospy.logwarn_throttle(10.0, "截图RTSP连接失败，将继续重试")
                        time.sleep(2.0)
                        continue
                ok, frame = capture.read()
                if not ok or frame is None:
                    capture.release()
                    capture = None
                    time.sleep(0.5)
                    continue
                with self.lock:
                    self.latest_rtsp_frame = frame
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "截图RTSP读取失败: %s", exc)
                if capture is not None:
                    capture.release()
                    capture = None
                time.sleep(1.0)
        if capture is not None:
            capture.release()

    def _receive_yolo(self):
        if zmq is None:
            rospy.logerr("缺少 pyzmq；仍会按点位上传全0结果")
            return
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.setsockopt(zmq.RCVTIMEO, 500)
        socket.connect(self.zmq_address)
        try:
            while not self.stopping and not rospy.is_shutdown():
                try:
                    frame = socket.recv_json()
                except zmq.Again:
                    continue
                except Exception as exc:
                    rospy.logwarn_throttle(5.0, "YOLO ZMQ接收失败: %s", exc)
                    continue
                self._collect_frame(frame)
        finally:
            socket.close(linger=0)
            context.term()

    def _collect_frame(self, frame):
        objects = frame.get("objects", []) if isinstance(frame, dict) else []
        if not isinstance(objects, list):
            objects = []
        with self.lock:
            collection = self.active_collection
            if collection is None:
                return
            frame_counts = {item["name"]: 0 for item in self.items}
            matched_objects = []
            by_id = {item["id"]: item["name"] for item in self.items}
            by_name = {}
            for item in self.items:
                by_name[item["name"].strip().lower()] = item["name"]
                by_name[item["display_name"].strip().lower()] = item["name"]
            for obj in objects:
                if not isinstance(obj, dict):
                    continue
                try:
                    confidence = float(obj.get("confidence", 0.0))
                except (TypeError, ValueError):
                    continue
                if confidence < self.min_confidence:
                    continue
                canonical_name = None
                supplied_names = []
                for key in ("class_name", "class_name_cn"):
                    candidate = str(obj.get(key, "")).strip().lower()
                    if candidate:
                        supplied_names.append(candidate)
                    if candidate in by_name:
                        canonical_name = by_name[candidate]
                        break
                # 逐帧数据没有类别名时才按 class_id 回退。这样即使启动参数选错
                # class profile，也不会把 COCO 的 person(id=0)误算成自定义挡掩(id=0)。
                if canonical_name is None and not supplied_names:
                    try:
                        canonical_name = by_id.get(int(obj.get("class_id", -1)))
                    except (TypeError, ValueError):
                        pass
                if canonical_name is not None:
                    frame_counts[canonical_name] += 1
                    matched = dict(obj)
                    matched["canonical_name"] = canonical_name
                    matched_objects.append(matched)
            collection.frame_count += 1
            for name, count in frame_counts.items():
                collection.counts[name] = max(collection.counts[name], count)
            score = sum(frame_counts.values())
            if score > 0 and score >= collection.best_score:
                collection.best_score = score
                collection.best_objects = matched_objects
                if self.latest_rtsp_frame is not None:
                    collection.best_image = self.latest_rtsp_frame.copy()

    def _save_screenshot(self, collection, stem):
        if cv2 is None or collection.best_image is None or not collection.best_objects:
            return ""
        image = collection.best_image.copy()
        height, width = image.shape[:2]
        for obj in collection.best_objects:
            bbox = obj.get("bbox", [])
            if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
                continue
            try:
                x1, y1, x2, y2 = [int(float(value)) for value in bbox[:4]]
            except (TypeError, ValueError):
                continue
            x1, x2 = sorted((max(0, min(width - 1, x1)), max(0, min(width - 1, x2))))
            y1, y2 = sorted((max(0, min(height - 1, y1)), max(0, min(height - 1, y2))))
            confidence = float(obj.get("confidence", 0.0))
            label = "{} {:.2f}".format(obj.get("canonical_name", "object"), confidence)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.putText(
                image, label, (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2, cv2.LINE_AA,
            )
        if width > self.image_max_width:
            scale = float(self.image_max_width) / float(width)
            image = cv2.resize(
                image, (self.image_max_width, max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            rospy.logwarn("巡航点截图JPEG编码失败: %s", collection.point_name)
            return ""
        image_path = os.path.join(self.task_dir, stem + ".jpg")
        atomic_write_bytes(image_path, encoded.tobytes())
        return image_path

    def _persist_collection(self, collection, state, reason):
        if not self.task_dir:
            rospy.logerr("任务目录不存在，无法保存点位结果: %s", collection.point_name)
            return False
        stamp = now_ms()
        stem = "{}_{}".format(safe_filename(collection.point_name, "point"), stamp)
        image_path = self._save_screenshot(collection, stem)
        payload = build_event_payload(
            self.robot_code,
            collection.task_id,
            collection.task_name,
            collection.point_name,
            self.items,
            collection.counts,
            collection.frame_count,
            state,
            timestamp_ms=stamp,
        )
        result_path = os.path.join(self.task_dir, stem + ".result.json")
        atomic_write_json(result_path, {
            "payload": payload,
            "imageFile": os.path.basename(image_path) if image_path else "",
            "savedMs": stamp,
            "finishReason": reason,
        })
        rospy.loginfo(
            "巡航点识别结果已保存: task=%s point=%s frames=%d image=%s",
            collection.task_name, collection.point_name, collection.frame_count,
            "yes" if image_path else "no",
        )
        return True

    def _finish_collection(self):
        with self.lock:
            self._finish_collection_locked("window_finished")

    def _finish_collection_locked(self, reason):
        collection = self.active_collection
        if collection is None:
            return
        if self.collection_timer is not None:
            self.collection_timer.cancel()
            self.collection_timer = None
        self.active_collection = None
        state = "OK" if collection.frame_count > 0 else "NO_YOLO_FRAME"
        self._persist_collection(collection, state, reason)

    def shutdown(self):
        with self.lock:
            self.stopping = True
            if self.active_collection is not None:
                self._finish_collection_locked("shutdown")
            if self.task_dir:
                self._write_manifest("COLLECTING")
        self.outbox.stop()
        self.zmq_thread.join(timeout=1.0)
        self.rtsp_thread.join(timeout=1.0)
        self.outbox_thread.join(timeout=1.0)


def create_parser():
    parser = argparse.ArgumentParser(description="按任务归档并上传YOLO截图和检测计数")
    parser.add_argument("--mqtt-config", default=DEFAULT_MQTT_CONFIG)
    parser.add_argument("--mqtt-credentials", default=DEFAULT_MQTT_CREDENTIALS)
    parser.add_argument("--mqtt-host", default="")
    parser.add_argument("--mqtt-port", type=int, default=None)
    parser.add_argument("--mqtt-username", default=DEFAULT_MQTT_USERNAME)
    parser.add_argument("--mqtt-password", default=DEFAULT_MQTT_PASSWORD)
    parser.add_argument("--file-topic", default="")
    parser.add_argument("--device-config", default=DEFAULT_DEVICE_CONFIG)
    parser.add_argument("--yolo-config", default=DEFAULT_YOLO_CONFIG)
    parser.add_argument("--yolo-env", default=DEFAULT_YOLO_ENV)
    parser.add_argument("--detector-file", default=DEFAULT_DETECTOR_FILE)
    parser.add_argument("--robot-code", default="")
    parser.add_argument("--zmq-address", default="")
    parser.add_argument("--rtsp-url", default="", help="留空时读取yolo.env中的RTSP配置")
    parser.add_argument("--results-root", default=DEFAULT_RESULTS_ROOT)
    parser.add_argument("--image-max-width", type=int, default=1280)
    parser.add_argument("--jpeg-quality", type=int, default=82)
    parser.add_argument(
        "--disable-screenshots", action="store_true",
        help="只保存文字结果，不额外连接RTSP保存截图",
    )
    parser.add_argument(
        "--class-profile", default="auto",
        choices=("auto", "inspection11", "truck10", "rknn", "custom", "coco", "coco80", "yolov8"),
        help="auto读取yolo.env；标准COCO YOLOv8请使用coco80",
    )
    parser.add_argument(
        "--detection-items", default="",
        help="逗号分隔的检测项目名；填写后覆盖检测程序中的类别表",
    )
    parser.add_argument("--min-confidence", type=float, default=None)
    parser.add_argument("--collection-seconds", type=float, default=None)
    parser.add_argument(
        "--point-keywords", default="",
        help="逗号分隔的点名关键词；留空表示每个arrived巡航点都上报",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="不连接MQTT；任务结束时打印待上传JSON且保留本地文件",
    )
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--snapshot-interval", type=float, default=3.0)
    return parser


def self_test(args):
    items = load_detection_items(
        args.detector_file, args.class_profile, args.yolo_env, args.detection_items)
    counts = {item["name"]: 0 for item in items}
    if items:
        counts[items[0]["name"]] = 2
    payload = build_event_payload(
        "DT-TEST", "TASK-TEST", "测试任务", "巡航点01",
        items, counts, 12, "OK", timestamp_ms=1700000000000,
    )
    assert payload["method"] == "file_upload_request"
    assert payload["data"]["taskName"] == "测试任务"
    assert payload["data"]["pointName"] == "巡航点01"
    assert len(payload["data"]["detectionItems"]) == len(items)
    assert payload["data"]["detectionItems"][0]["count"] == 2
    assert json.loads(payload["data"]["content"])["pointName"] == "巡航点01"
    assert infer_robot_code("/home/mile/mqtt_ws_DT202600001/mqtt_ws") == "DT202600001"
    fallback = load_detection_items(
        "/path/that/does/not/exist.py", "truck10", "/missing/yolo.env")
    assert len(fallback) == 10
    assert fallback[0]["display_name"] == "挡掩"
    print("SELF_TEST_OK classes={}".format(len(items)))


def main():
    parser = create_parser()
    raw_args = rospy.myargv(argv=sys.argv)[1:] if rospy is not None else sys.argv[1:]
    args = parser.parse_args(raw_args)
    if args.self_test:
        self_test(args)
        return 0
    if rospy is None or String is None:
        parser.error("运行节点需要 ROS Noetic 的 rospy 和 std_msgs")
    rospy.init_node("mqtt_yolo_point_uploader")
    from jieguo_uploader import SnapshotJieguoUploader
    SnapshotJieguoUploader(args)
    rospy.spin()
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda _signum, _frame: sys.exit(0))
    sys.exit(main())
