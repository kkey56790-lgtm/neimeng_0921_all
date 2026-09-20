#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
robot_running_mqtt.py

作用：
1. 连接 MQTT 中台，监听：
   thing/robot/{robotCode}/services
2. 处理：
   - set_mode
   - running_control
3. running_control 收到 start=true 后，不是在 MQTT 回调里只发一次 HTTP，
   而是由独立线程以固定频率持续 POST：
       http://机器人IP:7999/cmd/move
   从而保证连续运动。
4. 收到 start=false、MQTT 断线、程序退出、指令超时后立即发送 0 速度停车。
5. 每条中台指令都通过：
   thing/robot/{robotCode}/services_reply
   返回 result=0 / -1。
6. 可选：以约 1Hz 读取 /sensor_data/base_data 并通过 osd Topic 上报。

依赖：
    pip3 install requests paho-mqtt

运行：
    python3 robot_running_mqtt.py
"""

import json
import signal
import threading
import time
import uuid
from typing import Optional, Tuple

import requests
import paho.mqtt.client as mqtt


# ============================================================
# 1. 配置区
# ============================================================

# ---------------- 机器人 HTTP ----------------
ROBOT_IP = "192.168.2.100"
ROBOT_HTTP_PORT = 7999
ROBOT_HTTP_BASE = f"http://{ROBOT_IP}:{ROBOT_HTTP_PORT}"

HTTP_MOVE_URL = f"{ROBOT_HTTP_BASE}/cmd/move"
HTTP_BASE_DATA_URL = f"{ROBOT_HTTP_BASE}/sensor_data/base_data"

HTTP_CONNECT_TIMEOUT = 0.5
HTTP_READ_TIMEOUT = 0.8

# ---------------- MQTT 中台 ----------------
# 用户提供的是 tcp://222.187.130.102:1883
# paho-mqtt connect() 中只填写主机地址，不写 tcp://
MQTT_HOST = "222.187.130.102"
MQTT_PORT = 1883
MQTT_USERNAME = "autocar"
MQTT_PASSWORD = "123456"

# 按你的实际车辆编号修改
ROBOT_CODE = "DT202600001"
ROBOT_NAME = "内蒙古巡检车-DT202600001"

TOPIC_SERVICE = f"thing/robot/{ROBOT_CODE}/services"
TOPIC_SERVICE_REPLY = f"thing/robot/{ROBOT_CODE}/services_reply"
TOPIC_OSD = f"thing/robot/{ROBOT_CODE}/osd"

# ---------------- 运动参数 ----------------
# HTTP 协议：
# speed_x：线速度 m/s
# speed_z：角速度 rad/s
LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.20

# 运动状态下，以该频率持续向车体 HTTP 发布速度
MOVE_PUBLISH_HZ = 10.0

# 安全看门狗：
# 中台若持续发送 start=true，则每次都会刷新时间。
# 超过该时间没有收到新的 running_control 指令，自动停车。
#
# 如果你希望“一条 start=true 后一直走，直到收到 start=false”，
# 将其设为 0 即可关闭超时停车。
COMMAND_WATCHDOG_SEC = 2.5

# MQTT 协议要求先进入 MANUAL。
# True：没有收到 set_mode=MANUAL 前拒绝行走。
# False：允许直接 running_control。
REQUIRE_MANUAL_MODE = True

# 是否按约 1Hz 上报基本 OSD
ENABLE_OSD_UPLOAD = True
OSD_HZ = 1.0

# MQTT QoS
MQTT_QOS = 1


# ============================================================
# 2. 全局状态
# ============================================================

running = True

state_lock = threading.Lock()

manual_mode = False

motion_active = False
motion_direction: Optional[str] = None
last_running_command_time = 0.0

# 复用 HTTP TCP 连接，避免每次 POST 都重新建立连接
http_session = requests.Session()

mqtt_client = None


# ============================================================
# 3. 工具函数
# ============================================================

def now_ms() -> int:
    return int(time.time() * 1000)


def json_dumps(data) -> str:
    return json.dumps(
        data,
        ensure_ascii=False,
        separators=(",", ":")
    )


def direction_to_velocity(direction: str) -> Optional[Tuple[float, float]]:
    """
    MQTT 协议方向：
      forward  -> 前进
      backward -> 后退
      left     -> 左转
      right    -> 右转

    HTTP /cmd/move：
      speed_x = 线速度 m/s
      speed_z = 角速度 rad/s
    """
    direction = str(direction).strip().lower()

    table = {
        "forward": (LINEAR_SPEED, 0.0),
        "backward": (-LINEAR_SPEED, 0.0),
        "left": (0.0, ANGULAR_SPEED),
        "right": (0.0, -ANGULAR_SPEED),
    }
    return table.get(direction)


def http_post_move(speed_x: float, speed_z: float, quiet: bool = False) -> bool:
    """
    POST /cmd/move
    Body:
    {
        "speed_x": ...,
        "speed_z": ...
    }
    """
    body = {
        "speed_x": float(speed_x),
        "speed_z": float(speed_z),
    }

    try:
        resp = http_session.post(
            HTTP_MOVE_URL,
            json=body,
            timeout=(HTTP_CONNECT_TIMEOUT, HTTP_READ_TIMEOUT),
        )

        if resp.status_code != 200:
            if not quiet:
                print(
                    f"[HTTP][MOVE] HTTP状态码异常: {resp.status_code}, "
                    f"body={resp.text[:300]}"
                )
            return False

        try:
            data = resp.json()
        except Exception:
            if not quiet:
                print(f"[HTTP][MOVE] 非JSON返回: {resp.text[:300]}")
            return False

        result = bool(data.get("result", False))

        if not result and not quiet:
            print(
                "[HTTP][MOVE] 车体返回失败: "
                + json.dumps(data, ensure_ascii=False)
            )

        return result

    except requests.RequestException as exc:
        if not quiet:
            print(f"[HTTP][MOVE] 请求失败: {exc}")
        return False


def force_stop(reason: str = "") -> None:
    """
    将内部运动状态清空，并立即向底盘发送 0 速度。
    连发两次停车命令，提高停车命令送达概率。
    """
    global motion_active, motion_direction

    with state_lock:
        motion_active = False
        motion_direction = None

    if reason:
        print(f"[STOP] {reason}")

    for _ in range(2):
        http_post_move(0.0, 0.0, quiet=False)
        time.sleep(0.03)


def publish_reply(tid, method: str, result: int) -> None:
    """
    MQTT 协议 services_reply：
    {
      "tid": 原tid,
      "method": 原method,
      "timestamp": 13位毫秒时间戳,
      "data": {
        "result": 0
      }
    }
    """
    global mqtt_client

    if mqtt_client is None:
        return

    reply = {
        "tid": str(tid) if tid is not None else str(uuid.uuid4()),
        "method": method,
        "timestamp": now_ms(),
        "data": {
            "result": int(result)
        }
    }

    try:
        info = mqtt_client.publish(
            TOPIC_SERVICE_REPLY,
            payload=json_dumps(reply),
            qos=MQTT_QOS,
            retain=False,
        )
        print(
            f"[REPLY] method={method} result={result} "
            f"mid={getattr(info, 'mid', '-')}"
        )
    except Exception as exc:
        print(f"[REPLY] 发布失败: {exc}")


# ============================================================
# 4. MQTT 指令处理
# ============================================================

def handle_set_mode(tid, data: dict) -> None:
    """
    MQTT:
    method=set_mode
    data:
    {
        "robotCode": "DT202600001",
        "mode": "MANUAL"
    }

    注意：
    用户提供的 HTTP V2.3 文档没有给出“手动/自动模式切换”的 HTTP 接口，
    因此这里只把 set_mode 作为本 MQTT 网关自己的运动授权状态，
    不虚构 HTTP API。
    """
    global manual_mode

    robot_code = str(data.get("robotCode", "")).strip()
    mode = str(data.get("mode", "")).strip().upper()

    if robot_code and robot_code != ROBOT_CODE:
        print(f"[SET_MODE] robotCode不匹配: {robot_code}")
        publish_reply(tid, "set_mode", -1)
        return

    if mode == "MANUAL":
        with state_lock:
            manual_mode = True

        print("[SET_MODE] 已进入 MANUAL")
        publish_reply(tid, "set_mode", 0)
        return

    # 非 MANUAL：停止运动，并退出手动授权
    force_stop(f"收到模式切换: {mode or 'UNKNOWN'}")

    with state_lock:
        manual_mode = False

    print(f"[SET_MODE] 当前不是 MANUAL: {mode}")
    publish_reply(tid, "set_mode", 0)


def handle_running_control(tid, data: dict) -> None:
    """
    MQTT:
    method=running_control
    data:
    {
        "robotCode": "DT202600001",
        "direction": "forward",
        "start": true
    }
    """
    global motion_active
    global motion_direction
    global last_running_command_time

    robot_code = str(data.get("robotCode", "")).strip()
    direction = str(data.get("direction", "")).strip().lower()
    start = data.get("start", None)

    # robotCode 校验
    if robot_code and robot_code != ROBOT_CODE:
        print(
            f"[RUNNING_CONTROL] robotCode不匹配: "
            f"recv={robot_code}, local={ROBOT_CODE}"
        )
        publish_reply(tid, "running_control", -1)
        return

    # start 必须是真正的 JSON bool
    if not isinstance(start, bool):
        print(f"[RUNNING_CONTROL] start字段不是bool: {start!r}")
        publish_reply(tid, "running_control", -1)
        return

    # stop 指令优先处理：
    # 不要求 direction 一定合法，因为中台有时可能只关心 start=false。
    if start is False:
        force_stop(
            f"收到停止指令 direction={direction or '-'}"
        )
        publish_reply(tid, "running_control", 0)
        return

    # start=true 时检查方向
    velocity = direction_to_velocity(direction)
    if velocity is None:
        print(f"[RUNNING_CONTROL] 非法direction: {direction!r}")
        force_stop("收到非法方向，为安全起见停车")
        publish_reply(tid, "running_control", -1)
        return

    # 是否要求 MANUAL
    with state_lock:
        current_manual = manual_mode

    if REQUIRE_MANUAL_MODE and not current_manual:
        print(
            "[RUNNING_CONTROL] 当前未进入MANUAL，拒绝运动。"
            "请先下发 method=set_mode, mode=MANUAL"
        )
        force_stop("未进入MANUAL")
        publish_reply(tid, "running_control", -1)
        return

    # 更新运动状态。
    # HTTP 连续发送由 motion_worker() 完成，不阻塞 MQTT callback。
    with state_lock:
        motion_direction = direction
        motion_active = True
        last_running_command_time = time.monotonic()

    speed_x, speed_z = velocity

    # 收到命令立即先发一次，降低首包运动延迟
    ok = http_post_move(speed_x, speed_z, quiet=False)

    print(
        f"[RUNNING_CONTROL] START direction={direction}, "
        f"speed_x={speed_x:.3f}, speed_z={speed_z:.3f}, "
        f"http={'OK' if ok else 'FAIL'}"
    )

    # “正确收到并触发动作”才返回 0
    publish_reply(tid, "running_control", 0 if ok else -1)


def handle_service_message(payload: dict) -> None:
    tid = payload.get("tid")
    method = str(payload.get("method", "")).strip()
    data = payload.get("data", {})

    if not isinstance(data, dict):
        print("[MQTT] data不是Object")
        publish_reply(tid, method or "unknown", -1)
        return

    print(
        f"[SERVICE] tid={tid} method={method} "
        f"data={json.dumps(data, ensure_ascii=False)}"
    )

    if method == "set_mode":
        handle_set_mode(tid, data)

    elif method == "running_control":
        handle_running_control(tid, data)

    else:
        # 当前文件只实现车体行走控制相关 method。
        print(f"[SERVICE] 暂不处理method: {method}")
        publish_reply(tid, method or "unknown", -1)


# ============================================================
# 5. 连续运动线程
# ============================================================

def motion_worker() -> None:
    """
    关键线程：
    只要 motion_active=True，就持续以 MOVE_PUBLISH_HZ
    向 /cmd/move 重复发送当前速度。

    因此：
    - MQTT 回调不会因为 HTTP 连续发送而阻塞；
    - 连续收到 forward/start=true 时车体保持连续前进；
    - forward -> left 等方向切换会立即改变持续发布的速度；
    - start=false 会立即变为 0；
    - 指令流中断超过 watchdog 后自动停车。
    """
    global motion_active
    global motion_direction

    period = 1.0 / max(MOVE_PUBLISH_HZ, 1.0)
    next_tick = time.monotonic()
    consecutive_failures = 0

    while running:
        now = time.monotonic()

        with state_lock:
            active = motion_active
            direction = motion_direction
            last_cmd = last_running_command_time

        if active:
            # 看门狗
            if (
                COMMAND_WATCHDOG_SEC > 0
                and last_cmd > 0
                and (now - last_cmd) > COMMAND_WATCHDOG_SEC
            ):
                force_stop(
                    f"running_control 超时 "
                    f"{now - last_cmd:.2f}s，自动停车"
                )
                consecutive_failures = 0
                time.sleep(period)
                continue

            velocity = direction_to_velocity(direction or "")

            if velocity is None:
                force_stop("内部方向状态异常")
                consecutive_failures = 0
                time.sleep(period)
                continue

            speed_x, speed_z = velocity
            ok = http_post_move(speed_x, speed_z, quiet=True)

            if ok:
                consecutive_failures = 0
            else:
                consecutive_failures += 1

                # 连续 HTTP 失败时打印一次明显提示
                if consecutive_failures == 3:
                    print(
                        "[MOVE_LOOP] 连续3次HTTP发送失败，"
                        "请检查机器人 192.168.2.100:7999"
                    )

        next_tick += period
        sleep_time = next_tick - time.monotonic()

        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            # 若一次请求耗时过长，不追赶历史周期
            next_tick = time.monotonic()


# ============================================================
# 6. OSD 上报线程（约1Hz）
# ============================================================

def osd_worker() -> None:
    """
    根据 MQTT 协议，以约 1Hz 发布 method=osd。
    数据来源：HTTP /sensor_data/base_data

    为尽量保持 HTTP 原始字段结构，这里把 base_data 返回的 data 原样上报，
    并附加 robotCode、robotName。
    """
    period = 1.0 / max(OSD_HZ, 0.1)

    while running:
        begin = time.monotonic()

        if ENABLE_OSD_UPLOAD and mqtt_client is not None:
            try:
                resp = http_session.get(
                    HTTP_BASE_DATA_URL,
                    timeout=(HTTP_CONNECT_TIMEOUT, HTTP_READ_TIMEOUT),
                )

                if resp.status_code == 200:
                    body = resp.json()

                    if body.get("result") is True and isinstance(
                        body.get("data"), dict
                    ):
                        osd_data = dict(body["data"])
                        osd_data["robotCode"] = ROBOT_CODE
                        osd_data["robotName"] = ROBOT_NAME

                        envelope = {
                            "tid": str(uuid.uuid4()),
                            "method": "osd",
                            "timestamp": now_ms(),
                            "data": osd_data,
                        }

                        mqtt_client.publish(
                            TOPIC_OSD,
                            payload=json_dumps(envelope),
                            qos=0,
                            retain=False,
                        )

            except Exception as exc:
                print(f"[OSD] 获取/发布失败: {exc}")

        elapsed = time.monotonic() - begin
        time.sleep(max(0.05, period - elapsed))


# ============================================================
# 7. MQTT 回调
# ============================================================

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("=" * 70)
        print("[MQTT] 已连接中台")
        print(f"[MQTT] Broker : tcp://{MQTT_HOST}:{MQTT_PORT}")
        print(f"[MQTT] Robot  : {ROBOT_CODE}")
        print(f"[MQTT] SUB    : {TOPIC_SERVICE}")
        print(f"[MQTT] REPLY  : {TOPIC_SERVICE_REPLY}")
        print(f"[MQTT] OSD    : {TOPIC_OSD}")
        print("=" * 70)

        client.subscribe(TOPIC_SERVICE, qos=MQTT_QOS)
    else:
        print(f"[MQTT] 连接失败 rc={rc}")


def on_message(client, userdata, msg):
    try:
        text = msg.payload.decode("utf-8")
        payload = json.loads(text)
    except Exception as exc:
        print(f"[MQTT] JSON解析失败: {exc}")
        return

    if not isinstance(payload, dict):
        print("[MQTT] 根节点必须是JSON Object")
        return

    # 不在回调线程中执行可能稍耗时的 HTTP 操作，
    # 防止 MQTT 心跳和后续控制指令被阻塞。
    threading.Thread(
        target=handle_service_message,
        args=(payload,),
        daemon=True,
    ).start()


def on_disconnect(client, userdata, rc, properties=None):
    print(f"[MQTT] 已断开 rc={rc}")

    # 网络控制链路断开时主动停车
    if running:
        threading.Thread(
            target=force_stop,
            args=("MQTT连接断开",),
            daemon=True,
        ).start()


# ============================================================
# 8. MQTT Client
# ============================================================

def build_mqtt_client():
    client_id = f"{ROBOT_CODE}-running-gateway-{uuid.uuid4().hex[:8]}"

    # 兼容 paho-mqtt 1.x / 2.x
    try:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION1,
            client_id=client_id,
            clean_session=True,
        )
    except (AttributeError, TypeError):
        client = mqtt.Client(
            client_id=client_id,
            clean_session=True,
        )

    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    # 自动重连间隔
    try:
        client.reconnect_delay_set(min_delay=1, max_delay=10)
    except Exception:
        pass

    return client


# ============================================================
# 9. 程序退出
# ============================================================

def shutdown_handler(signum=None, frame=None):
    global running

    if not running:
        return

    print("\n[EXIT] 正在安全停车并退出...")
    running = False

    try:
        force_stop("程序退出")
    except Exception:
        pass

    try:
        if mqtt_client is not None:
            mqtt_client.disconnect()
    except Exception:
        pass


# ============================================================
# 10. main
# ============================================================

def main():
    global mqtt_client

    print("=" * 70)
    print("机器人 MQTT -> HTTP 连续行走控制网关")
    print("=" * 70)
    print(f"Robot HTTP : {ROBOT_HTTP_BASE}")
    print(f"MQTT       : tcp://{MQTT_HOST}:{MQTT_PORT}")
    print(f"RobotCode  : {ROBOT_CODE}")
    print(f"Move Hz    : {MOVE_PUBLISH_HZ}")
    print(f"Linear     : {LINEAR_SPEED} m/s")
    print(f"Angular    : {ANGULAR_SPEED} rad/s")
    print(f"Watchdog   : {COMMAND_WATCHDOG_SEC} s")
    print(f"NeedManual : {REQUIRE_MANUAL_MODE}")
    print("=" * 70)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    # 启动前先发停车，避免继承旧速度
    force_stop("程序启动初始化")

    # 连续运动线程
    threading.Thread(
        target=motion_worker,
        name="motion_worker",
        daemon=True,
    ).start()

    # OSD线程
    if ENABLE_OSD_UPLOAD:
        threading.Thread(
            target=osd_worker,
            name="osd_worker",
            daemon=True,
        ).start()

    mqtt_client = build_mqtt_client()

    while running:
        try:
            print("[MQTT] 正在连接...")
            mqtt_client.connect(
                MQTT_HOST,
                MQTT_PORT,
                keepalive=30,
            )

            # 阻塞并自动处理 MQTT 网络循环。
            # 断线后 loop_forever 会按配置尝试重连。
            mqtt_client.loop_forever(
                retry_first_connection=True
            )

        except TypeError:
            # 兼容旧版 paho，没有 retry_first_connection 参数
            try:
                mqtt_client.loop_forever()
            except Exception as exc:
                if running:
                    print(f"[MQTT] 网络循环异常: {exc}")
                    time.sleep(2)

        except Exception as exc:
            if running:
                print(f"[MQTT] 连接/网络异常: {exc}")
                time.sleep(2)

    print("[EXIT] 程序结束")


if __name__ == "__main__":
    main()
