#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DT202600001 机器人 MQTT 综合服务。

整合功能：
1. MQTT 连接成功（包括断线重连）后自动向 thing/robot/insert 注册；
2. 每秒采集基础数据、当前地图和当前任务并上报 OSD；
3. 每 60 秒上报任务列表和地图列表；
4. 订阅 task_upload / task_stop 指令，安全切换或停止机器人任务；
5. 将控制结果发布到 services_reply。

依赖：requests、paho-mqtt。
"""

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
TASK_RELEASE_TIMEOUT = 5.0
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


def get_task_list(timeout=HTTP_TIMEOUT):
    return _get_response_data(
        TASK_LIST_URL,
        "GET_TASK_LIST",
        list,
        [],
        timeout=timeout,
    )


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


def stop_robot_task():
    print("\n[TASK] 停止当前任务...")
    result = http_get(TASK_STOP_URL, "STOP_TASK_QUEUE", show=True)
    return result is not None and result.get("result") is True


def wait_task_release(stop_event=None):
    deadline = time.monotonic() + TASK_RELEASE_TIMEOUT
    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return False

        status = get_task_status()
        state = str(status.get("task_state", ""))
        main_name = str(status.get("main_task_name", ""))
        task_name = str(status.get("task_name", ""))
        print("[WAIT] state={} main={} task={}".format(state, main_name, task_name))

        if state in ("STATE_CANCEL", "STATE_FINISH", "STATE_FAIL", ""):
            if stop_event is None:
                time.sleep(0.8)
            else:
                stop_event.wait(0.8)
            return True

        if stop_event is None:
            time.sleep(TASK_RELEASE_CHECK_INTERVAL)
        else:
            stop_event.wait(TASK_RELEASE_CHECK_INTERVAL)

    print("[WARN] 等待旧任务释放超时")
    return False


def start_robot_task(task_name):
    body = {"name": task_name, "loop_time": TASK_LOOP_TIME}
    return http_post(TASK_START_URL, body, "START_TASK_QUEUE", show=True)


def safe_switch_task(task_name, stop_event=None):
    print("\n======================================")
    print("[TASK SWITCH]")
    print("目标任务:", task_name)

    stop_ok = stop_robot_task()
    if stop_ok:
        print("[TASK] stop_task_queue 成功")
    else:
        print("[WARN] stop_task_queue 未返回成功，继续尝试释放任务")

    wait_task_release(stop_event)

    for attempt in range(1, TASK_START_RETRY + 1):
        if stop_event is not None and stop_event.is_set():
            return False

        print("\n[TASK] 启动尝试 {}/{}".format(attempt, TASK_START_RETRY))
        result = start_robot_task(task_name)
        if result is None:
            if stop_event is None:
                time.sleep(TASK_RETRY_DELAY)
            else:
                stop_event.wait(TASK_RETRY_DELAY)
            continue

        if result.get("result") is True:
            print("[TASK] 新任务启动成功:", task_name)
            return True

        error_data = str(result.get("data", ""))
        print("[TASK] 启动失败:", error_data)
        if "is doing" not in error_data.lower():
            return False

        print("[TASK] 底盘仍处于 busy 状态，再次执行 STOP")
        stop_robot_task()
        if stop_event is None:
            time.sleep(TASK_RETRY_DELAY)
        else:
            stop_event.wait(TASK_RETRY_DELAY)
        wait_task_release(stop_event)

    print("[TASK] 多次尝试仍无法启动")
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

    def send_reply(self, tid, method, result):
        message = {
            "tid": tid,
            "method": method,
            "timestamp": now_ms(),
            "data": {"result": result},
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
                self.send_reply(tid, method, -1)
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
        robot_code = str(data.get("robotCode", ""))
        main_task_name = str(data.get("mainTaskName", ""))
        task_code = str(data.get("taskCode", ""))

        print("\n======================================")
        print("[TASK_UPLOAD]")
        print("robotCode:", robot_code)
        print("任务名称:", main_task_name)
        print("taskCode:", task_code)

        if robot_code and robot_code != ROBOT_CODE:
            print("robotCode 不匹配")
            self.send_reply(tid, "task_upload", -1)
            return

        target_task = find_task_by_name(main_task_name) if main_task_name else None
        if target_task is None and task_code:
            target_task = find_task_by_code(task_code)
        if target_task is None:
            print("本地不存在目标任务")
            self.send_reply(tid, "task_upload", -1)
            return

        real_name = str(target_task.get("name", ""))
        current = get_task_status()
        print("找到任务:", real_name)
        print("当前运行任务:", current.get("main_task_name", ""))
        print("当前任务状态:", current.get("task_state", ""))

        success = safe_switch_task(real_name, self.stop_event)
        self.send_reply(tid, "task_upload", 0 if success else -1)

    def handle_task_stop(self, tid, data):
        robot_code = str(data.get("robotCode", ""))
        print("\n======================================")
        print("[TASK_STOP]")

        if robot_code and robot_code != ROBOT_CODE:
            print("robotCode 不匹配")
            self.send_reply(tid, "task_stop", -1)
            return

        success = stop_robot_task()
        if success:
            wait_task_release(self.stop_event)
            print("任务停止成功")
        else:
            print("任务停止失败")
        self.send_reply(tid, "task_stop", 0 if success else -1)

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
