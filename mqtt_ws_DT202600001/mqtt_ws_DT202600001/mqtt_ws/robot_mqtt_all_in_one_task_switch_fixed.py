#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DT202600001 机器人 MQTT 综合服务（任务切换修复版）。

运行：python3 robot_mqtt_all_in_one_task_switch_fixed.py
先停止原 robot_mqtt_all_in_one.py 进程，再启动本文件；不要同时运行两个版本。
这是完整独立副本，不导入、不修改原文件，沿用原配置和上报功能。
停止接口按本项目 RobotApi.stop_task 使用 POST；停止和启动均核对底盘状态。
启动成功回执 stage=started 仅表示已启动，不表示整项巡检已完成。
若其他 ROS 调度器也在控制任务，需保证同一时间只有一个任务控制方。


整合功能：
1. MQTT 连接成功（包括断线重连）后自动向 thing/robot/insert 注册；
2. 每秒采集基础数据、当前地图和当前任务并上报 OSD；
3. 每 60 秒上报任务列表和地图列表；
4. 订阅 task_upload / task_stop 指令，安全切换或停止机器人任务；
5. 将控制结果发布到 services_reply。

依赖：requests、paho-mqtt。
"""

import ast
import hashlib
import json
import queue
import signal
import threading
import time
import uuid

import paho.mqtt.client as mqtt
import requests


# =============================================================================
# 机器人及 MQTT 配置
# =============================================================================

ROBOT_IP = "192.168.2.100"
ROBOT_HTTP_PORT = 7999
ROBOT_CODE = "DT202600001"
ROBOT_NAME = "内蒙古巡检车-DT202600001"

MQTT_HOST = "222.187.130.102"
MQTT_PORT = 1883
MQTT_USERNAME = "autocar"
MQTT_PASSWORD = "123456"
MQTT_KEEPALIVE = 60

REGISTER_TOPIC = "thing/robot/insert"
OSD_TOPIC = "thing/robot/{}/osd".format(ROBOT_CODE)
SERVICE_TOPIC = "thing/robot/{}/services".format(ROBOT_CODE)
REPLY_TOPIC = "thing/robot/{}/services_reply".format(ROBOT_CODE)


# =============================================================================
# HTTP 接口及运行参数
# =============================================================================

HTTP_BASE = "http://{}:{}".format(ROBOT_IP, ROBOT_HTTP_PORT)
BASE_DATA_URL = HTTP_BASE + "/sensor_data/base_data"
CURRENT_MAP_URL = HTTP_BASE + "/map_manage/current_map_info"
MAP_LIST_URL = HTTP_BASE + "/map_manage/get_map_info"
TASK_LIST_URL = HTTP_BASE + "/task_manager/get_task_list"
TASK_STATUS_URL = HTTP_BASE + "/task_manager/get_task_status"
TASK_START_URL = HTTP_BASE + "/task_manager/start_task_queue"
TASK_STOP_URL = HTTP_BASE + "/task_manager/stop_task_queue"

OSD_INTERVAL = 1.0
CATALOG_REPORT_INTERVAL = 60.0
OSD_HTTP_TIMEOUT = 3.0
HTTP_TIMEOUT = 5.0

TASK_LOOP_TIME = 1
TASK_START_RETRY = 3
TASK_RETRY_DELAY = 1.0
TASK_RELEASE_TIMEOUT = 15.0
TASK_START_CONFIRM_TIMEOUT = 10.0
TASK_RELEASE_CHECK_INTERVAL = 0.5

_http_local = threading.local()
_http_sessions = []
_http_sessions_lock = threading.Lock()


# =============================================================================
# 通用工具
# =============================================================================

def now_ms():
    """返回 13 位毫秒时间戳。"""
    return int(time.time() * 1000)


def create_tid():
    return str(uuid.uuid4())


def generate_task_code(task_name):
    raw = str(task_name).encode("utf-8")
    digest = hashlib.sha256(raw).hexdigest().upper()
    return "TASK-" + digest[:16]


def _safe_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


# =============================================================================
# HTTP 访问
# =============================================================================

def _get_http_session():
    """每个工作线程使用独立 Session，避免并发访问同一 Session。"""
    session = getattr(_http_local, "session", None)
    if session is None:
        session = requests.Session()
        _http_local.session = session
        with _http_sessions_lock:
            _http_sessions.append(session)
    return session


def close_http_sessions():
    with _http_sessions_lock:
        sessions = list(_http_sessions)
        _http_sessions[:] = []
    for session in sessions:
        try:
            session.close()
        except Exception:
            pass


def http_get(url, name, show=False, timeout=HTTP_TIMEOUT):
    try:
        response = _get_http_session().get(url, timeout=timeout)
        response.raise_for_status()
        result = response.json()
    except Exception as exc:
        print("[HTTP ERROR] {}: {}".format(name, exc))
        return None

    if not isinstance(result, dict):
        print("[HTTP ERROR] {}: 返回不是 JSON Object".format(name))
        return None

    if show:
        print("\n[HTTP] {}".format(name))
        print(json.dumps(result, ensure_ascii=False, indent=2))

    return result


def http_post(url, body, name, show=True, timeout=HTTP_TIMEOUT):
    try:
        response = _get_http_session().post(url, json=body, timeout=timeout)
        response.raise_for_status()
        result = response.json()
    except Exception as exc:
        print("[HTTP ERROR] {}: {}".format(name, exc))
        return None

    if not isinstance(result, dict):
        print("[HTTP ERROR] {}: 返回不是 JSON Object".format(name))
        return None

    if show:
        print("\n[HTTP] {}".format(name))
        print(json.dumps(result, ensure_ascii=False, indent=2))

    return result


def http_get_data(url, name, timeout=HTTP_TIMEOUT):
    """兼容 osd_mqtt_test.py：校验 result 后返回 data。"""
    result = http_get(url, name, timeout=timeout)
    if result is None:
        return None
    if result.get("result") is not True:
        print("[HTTP ERROR] {}: result=false".format(name))
        print(json.dumps(result, ensure_ascii=False))
        return None
    return result.get("data")


def _get_response_data(url, name, expected_type, fallback, timeout=HTTP_TIMEOUT):
    result = http_get(url, name, timeout=timeout)
    if result is None:
        return fallback

    if result.get("result") is not True:
        print("[HTTP ERROR] {}: result=false".format(name))
        print(json.dumps(result, ensure_ascii=False))
        return fallback

    data = result.get("data")
    if not isinstance(data, expected_type):
        print("[HTTP ERROR] {}: data 类型不正确".format(name))
        return fallback

    return data


def get_base_data(timeout=HTTP_TIMEOUT):
    return _get_response_data(
        BASE_DATA_URL,
        "BASE_DATA",
        dict,
        None,
        timeout=timeout,
    )


def get_current_map(timeout=HTTP_TIMEOUT):
    return _get_response_data(
        CURRENT_MAP_URL,
        "CURRENT_MAP",
        dict,
        {},
        timeout=timeout,
    )


def get_task_status(timeout=HTTP_TIMEOUT):
    return _get_response_data(
        TASK_STATUS_URL,
        "GET_TASK_STATUS",
        dict,
        {},
        timeout=timeout,
    )


def get_current_task(timeout=HTTP_TIMEOUT):
    """兼容 osd_mqtt_test.py 中原有的函数名。"""
    return get_task_status(timeout=timeout)


def task_record_name(task):
    # 与项目 catalog_rules.py 中的底盘任务名称字段保持兼容。
    for key in ("name", "task_name", "taskName", "main_task_name", "mainTaskName"):
        value = task.get(key)
        if isinstance(value, str) and value.strip():
            return value  # 启动时保留底盘给出的原始名称。
    return ""


def normalize_task_code(value):
    """接受协议字符串及中台单选数组；多选不得默默取第一项。"""
    if value is None:
        return ""
    if isinstance(value, str):
        value = value.strip()
        if value.startswith("["):
            if len(value) > 4096:
                raise ValueError("taskCode 数组文本过长")
            try:
                value = json.loads(value)
            except (ValueError, TypeError):
                try:
                    # 兼容 "['TASK-xxx']"；只解析字面量，不执行代码。
                    value = ast.literal_eval(value)
                except (ValueError, SyntaxError, RecursionError):
                    raise ValueError("taskCode 数组文本格式错误")
    if isinstance(value, list):
        if len(value) != 1:
            raise ValueError("taskCode 数组必须且只能包含一个任务编码")
        value = value[0]
        if not isinstance(value, str) or not value.strip():
            raise ValueError("taskCode 数组元素必须是非空字符串")
    if not isinstance(value, str):
        raise ValueError("taskCode 必须是字符串或仅含一个字符串的数组")
    return value.strip()


class TaskCatalogError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def read_task_catalog(timeout=HTTP_TIMEOUT):
    response = http_get(TASK_LIST_URL, "GET_TASK_LIST", timeout=timeout)
    if response is None:
        raise TaskCatalogError("TASK_LIST_READ_FAILED",
                               "任务列表接口读取失败: " + TASK_LIST_URL + "；请检查终端 HTTP ERROR 日志")
    if response.get("result") is not True:
        raise TaskCatalogError("TASK_LIST_REJECTED",
                               "任务列表接口未返回 result=true: " + json.dumps(response, ensure_ascii=False)[:800])
    raw = response.get("data")
    if not isinstance(raw, list):
        raise TaskCatalogError("TASK_LIST_FORMAT_ERROR",
                               "任务列表 data 应为数组，实际为 {}；请提供 GET_TASK_LIST 原始返回".format(type(raw).__name__))
    tasks = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict) or not task_record_name(item):
            raise TaskCatalogError("TASK_LIST_FORMAT_ERROR",
                                   "任务列表第 {} 项缺少有效任务名称；请提供接口原始返回".format(index + 1))
        task = dict(item)
        task["name"] = task_record_name(item)
        tasks.append(task)
    return tasks


def get_task_list(timeout=HTTP_TIMEOUT):
    # 不把接口错误伪装成空列表上报，保留中台上一次有效目录。
    return read_task_catalog(timeout=timeout)


def get_map_list(timeout=HTTP_TIMEOUT):
    return _get_response_data(
        MAP_LIST_URL,
        "GET_MAP_INFO",
        list,
        [],
        timeout=timeout,
    )


# =============================================================================
# 数据转换及消息构造
# =============================================================================

def format_current_map(current_map):
    if not isinstance(current_map, dict):
        return {}
    result = dict(current_map)
    result["mapName"] = str(current_map.get("name", ""))
    return result


def convert_current_map(current_map):
    """兼容 robot_task_mqtt.py 中原有的函数名。"""
    return format_current_map(current_map)


def extract_map_names(task):
    result = []
    subtasks = task.get("tasks", [])
    if not isinstance(subtasks, list):
        return result

    for subtask in subtasks:
        if not isinstance(subtask, dict):
            continue
        for key in ("map", "map_name"):
            map_name = subtask.get(key)
            if map_name:
                map_name = str(map_name)
                if map_name not in result:
                    result.append(map_name)
    return result


def convert_task_list(raw_tasks):
    result = []
    for index, task in enumerate(raw_tasks, start=1):
        if not isinstance(task, dict):
            continue

        name = str(task.get("name", ""))
        subtasks = task.get("tasks", [])
        if not isinstance(subtasks, list):
            subtasks = []

        task_code = task.get("taskCode", "") or generate_task_code(name)
        map_names = extract_map_names(task)
        result.append(
            {
                "taskCode": task_code,
                "mainTaskName": name,
                "taskName": name,
                "index": index,
                "mapName": map_names[0] if map_names else "",
                "mapNames": map_names,
                "subTaskCount": len(subtasks),
            }
        )
    return result


def convert_map_list(raw_maps):
    result = []
    for item in raw_maps:
        if not isinstance(item, dict):
            continue
        new_item = dict(item)
        new_item["mapName"] = str(item.get("name", ""))
        result.append(new_item)
    return result


def build_registration_message():
    return {
        "tid": create_tid(),
        "method": "insert",
        "timestamp": now_ms(),
        "data": {
            "robotCode": ROBOT_CODE,
            "robotName": ROBOT_NAME,
        },
    }


def build_osd_message(base_data, current_map, current_task):
    data = dict(base_data)
    data["map"] = format_current_map(current_map)
    data["currentTask"] = current_task
    timestamp = now_ms()
    return {
        "tid": str(timestamp),
        "method": "osd",
        "timestamp": timestamp,
        "data": data,
    }


def build_task_message():
    raw_tasks = get_task_list()
    current_task = get_task_status()
    return {
        "tid": create_tid(),
        "method": "task_list",
        "timestamp": now_ms(),
        "data": {
            "robotCode": ROBOT_CODE,
            "robotName": ROBOT_NAME,
            "taskList": convert_task_list(raw_tasks),
            "currentTask": current_task,
        },
    }


def build_map_message():
    raw_maps = get_map_list()
    current_map = get_current_map()
    return {
        "tid": create_tid(),
        "method": "map_list",
        "timestamp": now_ms(),
        "data": {
            "robotCode": ROBOT_CODE,
            "robotName": ROBOT_NAME,
            "mapList": convert_map_list(raw_maps),
            "map": convert_current_map(current_map),
        },
    }


# =============================================================================
# 任务查找、停止及切换
# =============================================================================

def find_task_by_name(name):
    for task in get_task_list():
        if isinstance(task, dict) and str(task.get("name", "")) == str(name):
            return task
    return None


def find_task_by_code(task_code):
    for task in get_task_list():
        if not isinstance(task, dict):
            continue
        name = str(task.get("name", ""))
        local_code = task.get("taskCode", "") or generate_task_code(name)
        if str(local_code) == str(task_code):
            return task
    return None


class TaskControlError(RuntimeError):
    """可回传中台的任务控制失败原因。"""


TERMINAL_TASK_STATES = frozenset((
    "STATE_CANCEL", "STATE_CANCELED", "STATE_CANCELLED",
    "STATE_FINISH", "STATE_FINISHED", "STATE_FAIL", "STATE_FAILED",
    "STATE_IDLE", "STATE_STOP", "STATE_STOPPED",
))


def _task_state(status):
    return str(status.get("task_state", "")).strip().upper()


def _check_cancelled(stop_event):
    if stop_event is not None and stop_event.is_set():
        raise TaskControlError("服务正在退出，取消任务切换")


def _control_wait(seconds, stop_event):
    if stop_event is None:
        time.sleep(seconds)
    else:
        stop_event.wait(seconds)
    _check_cancelled(stop_event)


def stop_robot_task():
    print("\n[TASK] POST 停止当前任务...")
    result = http_post(TASK_STOP_URL, {}, "STOP_TASK_QUEUE", show=True)
    return result is not None and result.get("result") is True


def wait_task_release(stop_event=None):
    deadline = time.monotonic() + TASK_RELEASE_TIMEOUT
    while time.monotonic() < deadline:
        _check_cancelled(stop_event)
        status = get_task_status(timeout=min(HTTP_TIMEOUT, max(0.1, deadline - time.monotonic())))
        state = _task_state(status)
        print("[WAIT STOP] state={} main={} queue={}".format(
            state, status.get("main_task_name", ""), status.get("queue_size", "?")))
        # 空字典/空状态可能是 HTTP 失败，绝不能当作已释放。
        if state in TERMINAL_TASK_STATES:
            _check_cancelled(stop_event)
            return True
        _control_wait(TASK_RELEASE_CHECK_INTERVAL, stop_event)
    print("[ERROR] 等待旧任务停止超时，禁止启动新任务")
    return False


def start_robot_task(task_name):
    body = {"name": task_name, "loop_time": TASK_LOOP_TIME}
    return http_post(TASK_START_URL, body, "START_TASK_QUEUE", show=True)


def wait_task_started(task_name, previous_status, stop_event=None):
    deadline = time.monotonic() + TASK_START_CONFIRM_TIMEOUT
    while time.monotonic() < deadline:
        _check_cancelled(stop_event)
        status = get_task_status(timeout=min(HTTP_TIMEOUT, max(0.1, deadline - time.monotonic())))
        state = _task_state(status)
        name = str(status.get("main_task_name", ""))
        print("[WAIT START] target={} main={} state={}".format(task_name, name, state))
        if name == task_name:
            if state in ("STATE_DOING", "STATE_PAUSE"):
                _check_cancelled(stop_event)
                return True
            # 极短任务可能在第一次轮询前完成；排除同名旧任务的完成快照。
            fresh = (previous_status.get("main_task_name") != task_name or
                     (status.get("task_time") and
                      status.get("task_time") != previous_status.get("task_time")))
            if fresh and state in ("STATE_FINISH", "STATE_FINISHED"):
                return True
            if fresh and state in ("STATE_FAIL", "STATE_FAILED"):
                raise TaskControlError("目标任务执行失败: {}".format(status.get("error_info", "")))
        _control_wait(TASK_RELEASE_CHECK_INTERVAL, stop_event)
    return False


def safe_switch_task(task_name, stop_event=None):
    print("\n[TASK SWITCH] 目标任务:", task_name)
    _check_cancelled(stop_event)
    if not stop_robot_task():
        raise TaskControlError("POST 停止旧任务失败；未下发新任务，请查看 STOP_TASK_QUEUE 日志")
    if not wait_task_release(stop_event):
        raise TaskControlError("等待旧任务释放超时或状态读取失败；未下发新任务")
    previous_status = get_task_status()
    for attempt in range(1, TASK_START_RETRY + 1):
        _check_cancelled(stop_event)
        print("[TASK] 启动尝试 {}/{}".format(attempt, TASK_START_RETRY))
        result = start_robot_task(task_name)
        if result is None or result.get("result") is True:
            # HTTP 超时也可能已被底盘执行；先核实，不盲目重发启动请求。
            if wait_task_started(task_name, previous_status, stop_event):
                print("[TASK] 已确认目标任务启动:", task_name)
                return True
            raise TaskControlError("未确认目标任务启动，结果未知；请检查底盘状态，未重复下发")
        error_data = str(result.get("data", result))
        if "is doing" not in error_data.lower() or attempt == TASK_START_RETRY:
            raise TaskControlError("底盘拒绝启动任务: " + error_data)
        # 仅在底盘明确拒绝（busy）时重试，且每次都必须确认停止。
        if not stop_robot_task() or not wait_task_release(stop_event):
            raise TaskControlError("底盘 busy，重新停止或等待释放失败")
        _control_wait(TASK_RETRY_DELAY, stop_event)
    return False


# =============================================================================
# 统一 MQTT 服务
# =============================================================================

class RobotMQTTService:
    def __init__(self):
        self.connected = threading.Event()
        self.stop_event = threading.Event()
        self.control_queue = queue.Queue()
        self.control_thread = threading.Thread(
            target=self._control_worker,
            name="robot-task-control",
            daemon=True,
        )
        self.catalog_thread = threading.Thread(
            target=self._catalog_worker,
            name="robot-catalog-report",
            daemon=True,
        )

        try:
            self.client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id="{}_all_in_one".format(ROBOT_CODE),
                protocol=mqtt.MQTTv311,
            )
        except (AttributeError, TypeError):
            self.client = mqtt.Client(
                client_id="{}_all_in_one".format(ROBOT_CODE),
                protocol=mqtt.MQTTv311,
            )

        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    @staticmethod
    def _reason_code_value(reason_code):
        try:
            return int(reason_code)
        except (TypeError, ValueError):
            return reason_code

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        code = self._reason_code_value(reason_code)
        if code != 0:
            self.connected.clear()
            print("[MQTT ERROR] 连接失败:", reason_code)
            return

        self.connected.set()
        subscribe_result = client.subscribe(SERVICE_TOPIC, qos=1)
        print("\n======================================")
        print("MQTT Broker 连接成功")
        print("Broker    : {}:{}".format(MQTT_HOST, MQTT_PORT))
        print("RobotCode :", ROBOT_CODE)
        print("订阅       :", SERVICE_TOPIC)
        print("上报       :", OSD_TOPIC)
        print("支持       : task_upload / task_stop")
        print("订阅结果   :", subscribe_result)
        print("======================================")

        # on_connect 会在每次重连后再次执行，因此注册信息也会重新发送。
        self.publish_registration()

    def on_disconnect(self, client, userdata, *args):
        self.connected.clear()
        if not self.stop_event.is_set():
            print("[MQTT] Broker 连接断开，等待自动重连...")

    def on_message(self, client, userdata, msg):
        try:
            message = json.loads(msg.payload.decode("utf-8"))
            if not isinstance(message, dict):
                raise ValueError("消息不是 JSON Object")
        except Exception as exc:
            print("[MQTT ERROR] 无法解析控制消息:", exc)
            return

        method = str(message.get("method", ""))
        if method not in ("task_upload", "task_stop"):
            return

        data = message.get("data", {})
        if not isinstance(data, dict):
            data = {}
        self.control_queue.put((str(message.get("tid", "")), method, data))

    def connect(self):
        print("正在连接 MQTT Broker: {}:{}".format(MQTT_HOST, MQTT_PORT))
        self.client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=MQTT_KEEPALIVE)
        self.client.loop_start()

    def publish_json(self, topic, message):
        # 所有上报统一携带本机编码，覆盖底盘可能返回的其他 robotCode。
        message = dict(message)
        data = dict(message.get("data", {}))
        data["robotCode"] = ROBOT_CODE
        message["data"] = data
        payload = json.dumps(message, ensure_ascii=False, separators=(",", ":"))
        return self.client.publish(topic, payload, qos=1)

    def publish_registration(self):
        message = build_registration_message()
        info = self.publish_json(REGISTER_TOPIC, message)
        print(
            "[REGISTER] topic={} tid={} robotCode={} rc={}".format(
                REGISTER_TOPIC,
                message["tid"],
                ROBOT_CODE,
                info.rc,
            )
        )

    def publish_osd(self):
        base_data = get_base_data(timeout=OSD_HTTP_TIMEOUT)
        current_map = get_current_map(timeout=OSD_HTTP_TIMEOUT)
        current_task = get_task_status(timeout=OSD_HTTP_TIMEOUT)

        if base_data is None:
            print("[WARN] 基础数据读取失败，本周期不上传 OSD")
            return
        if not self.connected.is_set():
            return

        message = build_osd_message(base_data, current_map, current_task)
        info = self.publish_json(OSD_TOPIC, message)
        bms = base_data.get("bms", {})
        pose = base_data.get("pose", {})
        if not isinstance(bms, dict):
            bms = {}
        if not isinstance(pose, dict):
            pose = {}
        print(
            "[OSD] SOC={}% X={:.2f} Y={:.2f} 地图={} 主任务={} 当前任务={} 状态={} rc={}".format(
                bms.get("soc", 0),
                _safe_float(pose.get("x", 0)),
                _safe_float(pose.get("y", 0)),
                current_map.get("name", ""),
                current_task.get("main_task_name", ""),
                current_task.get("task_name", ""),
                current_task.get("task_state", ""),
                info.rc,
            )
        )

    def publish_task_list(self):
        message = build_task_message()
        info = self.publish_json(OSD_TOPIC, message)
        tasks = message["data"]["taskList"]
        current = message["data"]["currentTask"]
        print(
            "[TASK_LIST] 数量={} 当前任务={} 状态={} rc={}".format(
                len(tasks),
                current.get("main_task_name", ""),
                current.get("task_state", ""),
                info.rc,
            )
        )

    def publish_map_list(self):
        message = build_map_message()
        info = self.publish_json(OSD_TOPIC, message)
        maps = message["data"]["mapList"]
        current = message["data"]["map"]
        print(
            "[MAP_LIST] 数量={} 当前地图={} rc={}".format(
                len(maps), current.get("mapName", ""), info.rc
            )
        )

    def send_reply(self, tid, method, result, **details):
        message = {
            "tid": tid,
            "method": method,
            "timestamp": now_ms(),
            "data": dict(details, result=result),
        }
        info = self.publish_json(REPLY_TOPIC, message)
        print("[REPLY] method={} result={} rc={}".format(method, result, info.rc))

    def _control_worker(self):
        while not self.stop_event.is_set():
            try:
                command = self.control_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if command is None:
                self.control_queue.task_done()
                break

            tid, method, data = command
            try:
                if method == "task_upload":
                    self.handle_task_upload(tid, data)
                elif method == "task_stop":
                    self.handle_task_stop(tid, data)
            except Exception as exc:
                print("[TASK ERROR] method={}: {}".format(method, exc))
                self.send_reply(tid, method, -1, msg=str(exc))
            finally:
                self.control_queue.task_done()

    def _catalog_worker(self):
        """独立上报任务/地图列表，避免慢接口阻塞 1 Hz OSD。"""
        while not self.stop_event.is_set():
            if not self.connected.wait(timeout=0.5):
                continue
            if self.stop_event.is_set():
                break

            cycle_start = time.monotonic()
            try:
                self.publish_task_list()
            except Exception as exc:
                print("[TASK_LIST ERROR]", exc)
            try:
                self.publish_map_list()
            except Exception as exc:
                print("[MAP_LIST ERROR]", exc)

            elapsed = time.monotonic() - cycle_start
            self.stop_event.wait(max(0.0, CATALOG_REPORT_INTERVAL - elapsed))

    def handle_task_upload(self, tid, data):
        robot_code = str(data.get("robotCode") or "").strip()
        main_task_name = str(data.get("mainTaskName") or "").strip()
        try:
            task_code = normalize_task_code(data.get("taskCode"))
        except ValueError as exc:
            self.send_reply(tid, "task_upload", -1,
                            errorCode="TASK_CODE_FORMAT_ERROR", msg=str(exc))
            return

        print("\n======================================")
        print("[TASK_UPLOAD]")
        print("robotCode:", robot_code)
        print("任务名称:", main_task_name)
        print("taskCode:", task_code)

        if robot_code and robot_code != ROBOT_CODE:
            print("robotCode 不匹配")
            self.send_reply(tid, "task_upload", -1, msg="robotCode 不匹配")
            return

        if not main_task_name and not task_code:
            self.send_reply(tid, "task_upload", -1, errorCode="TASK_SELECTOR_MISSING",
                            msg="data 中必须提供 mainTaskName 或 taskCode")
            return
        try:
            tasks = read_task_catalog()
        except TaskCatalogError as exc:
            print("[TASK CATALOG ERROR]", exc)
            self.send_reply(tid, "task_upload", -1, errorCode=exc.code, msg=str(exc))
            return

        # 一条指令只读取一次列表，名称和编码使用同一份目录匹配。
        names = [task for task in tasks if main_task_name and
                 task["name"].strip() == main_task_name]
        codes = [task for task in tasks if task_code and
                 str(task.get("taskCode") or generate_task_code(task["name"])).strip() == task_code]
        matches = names or codes
        if len(matches) > 1 or (names and codes and names != codes):
            self.send_reply(tid, "task_upload", -1, errorCode="TASK_AMBIGUOUS",
                            msg="任务名称不唯一或名称与编码指向不同任务，请使用最新 task_list")
            return
        if not matches:
            available = [{"mainTaskName": task["name"],
                          "taskCode": task.get("taskCode") or generate_task_code(task["name"])}
                         for task in tasks]
            print("[TASK NOT FOUND] requested name={!r} code={!r}; available={}".format(
                main_task_name, task_code, json.dumps(available, ensure_ascii=False)))
            self.send_reply(tid, "task_upload", -1,
                            errorCode="TASK_LIST_EMPTY" if not tasks else "TASK_NOT_FOUND",
                            msg=("底盘任务列表为空，请先在底盘创建任务或检查当前地图" if not tasks else
                                 "任务名称/编码未匹配，请使用最新 task_list 中的 mainTaskName 和 taskCode"),
                            requestedMainTaskName=main_task_name, requestedTaskCode=task_code,
                            availableTasks=available[:20], availableTaskCount=len(available))
            return
        target_task = matches[0]

        real_name = str(target_task.get("name", ""))
        current = get_task_status()
        print("找到任务:", real_name)
        print("当前运行任务:", current.get("main_task_name", ""))
        print("当前任务状态:", current.get("task_state", ""))

        success = safe_switch_task(real_name, self.stop_event)
        self.send_reply(tid, "task_upload", 0 if success else -1,
                        mainTaskName=real_name, taskCode=task_code,
                        stage="started" if success else "failed")

    def handle_task_stop(self, tid, data):
        robot_code = str(data.get("robotCode") or "").strip()
        print("\n======================================")
        print("[TASK_STOP]")

        if robot_code and robot_code != ROBOT_CODE:
            print("robotCode 不匹配")
            self.send_reply(tid, "task_stop", -1)
            return

        _check_cancelled(self.stop_event)
        if not stop_robot_task():
            raise TaskControlError("POST 停止任务失败，请查看 STOP_TASK_QUEUE 日志")
        if not wait_task_release(self.stop_event):
            raise TaskControlError("等待停止确认超时或状态读取失败")
        print("任务停止成功（已核对底盘状态）")
        self.send_reply(tid, "task_stop", 0)

    def stop(self):
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        self.connected.clear()
        try:
            self.control_queue.put_nowait(None)
        except queue.Full:
            pass

    def close(self):
        self.stop()
        try:
            self.client.disconnect()
        except Exception:
            pass
        try:
            self.client.loop_stop()
        except Exception:
            pass
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        if self.catalog_thread.is_alive():
            self.catalog_thread.join(timeout=2.0)
        close_http_sessions()

    def run(self):
        self.control_thread.start()
        self.catalog_thread.start()
        self.connect()

        print("\n机器人 MQTT 综合服务运行中")
        print("OSD 每 {} 秒上传，任务/地图列表每 {} 秒上传".format(
            OSD_INTERVAL, CATALOG_REPORT_INTERVAL
        ))

        next_osd = 0.0
        try:
            while not self.stop_event.is_set():
                if not self.connected.is_set():
                    self.stop_event.wait(0.2)
                    continue

                now = time.monotonic()
                if now >= next_osd:
                    try:
                        self.publish_osd()
                    except Exception as exc:
                        print("[OSD ERROR]", exc)
                    next_osd = now + OSD_INTERVAL

                self.stop_event.wait(0.05)
        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，正在退出...")
        finally:
            self.close()
            print("程序已退出")


def main():
    service = RobotMQTTService()

    def handle_signal(signum, _frame):
        print("\n[SYSTEM] 收到信号 {}，正在停止...".format(signum))
        service.stop()

    signal.signal(signal.SIGINT, handle_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, handle_signal)

    service.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
