#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模拟中台通过 MQTT 一次下发任务7、任务10，兼容 batch/transition 服务。

安装依赖：python3 -m pip install paho-mqtt
预览报文：python3 simulate_task_queue.py --dry-run
实际下发：python3 simulate_task_queue.py
指定任务：python3 simulate_task_queue.py --tasks 任务7 任务10 --wait 600
指定真实编码：python3 simulate_task_queue.py --task-codes CODE7 CODE10
继续监听原批：python3 simulate_task_queue.py --listen-only --tid 原tid
默认持续监听，Ctrl+C退出；--wait 300可设置有限监听时间。
只监听模式不下发任务，仅接收订阅后的回执，不补发已经错过的历史消息。

默认使用现有服务的 Broker 配置，也可用 MQTT_HOST/MQTT_PORT/MQTT_USERNAME/
MQTT_PASSWORD 环境变量覆盖。脚本不导入或启动机器人服务。
每次运行生成新的 tid；重试同一批可用 --tid 原tid，避免重复执行。
名称编码默认沿用服务的 SHA256 算法；若底盘有自定义 taskCode，必须用
--task-codes 按任务顺序传入真实编码。
本脚本模拟下发端，不模拟底盘；普通运行会实际发送任务指令。
只发送 task_upload，不发送 task_switch/task_stop。退出监听不会停止机器人。
PUBACK 只证明 Broker 收到消息，task_completed 才表示整批完成。
"""

import argparse
import hashlib
import json
import os
import sys
import threading
import time
import uuid


def task_code(name):
    return "TASK-" + hashlib.sha256(name.encode("utf-8")).hexdigest().upper()[:16]


def build_message(robot_code, names, codes=None, tid=None):
    if not names or len(names) > 200:
        raise ValueError("任务数量必须为 1～200")
    if any(not isinstance(name, str) or not name.strip() for name in names):
        raise ValueError("任务名称不能为空")
    names = [name.strip() for name in names]
    if codes is not None and (len(codes) != len(names) or any(not c.strip() for c in codes)):
        raise ValueError("--task-codes 必须与任务数量一致且不能为空")
    if not robot_code.strip() or (tid is not None and not tid.strip()):
        raise ValueError("robot-code 和 tid 不能为空")
    codes = [c.strip() for c in codes] if codes is not None else [task_code(n) for n in names]
    return {
        "tid": tid.strip() if tid is not None else "sim-" + str(uuid.uuid4()),
        "method": "task_upload",
        "timestamp": int(time.time() * 1000),
        "data": {"robotCode": robot_code.strip(), "tasks": [
            {"mainTaskName": name, "taskCode": code} for name, code in zip(names, codes)
        ]},
    }


def reason_value(reason):
    return int(getattr(reason, "value", reason))


def send_and_listen(args, message):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        raise RuntimeError("缺少依赖，请执行：python3 -m pip install paho-mqtt")
    topic = "thing/robot/{}/services".format(message["data"]["robotCode"])
    reply_topic = topic + "_reply"
    ready = threading.Event()
    completed = threading.Event()
    errors = []
    outcome = {"result": None}

    def on_connect(client, userdata, flags, reason_code, properties=None):
        if reason_value(reason_code) != 0:
            errors.append("MQTT 连接被拒绝: {}".format(reason_code))
            ready.set()
            return
        rc, _ = client.subscribe(reply_topic, qos=1)
        if rc != mqtt.MQTT_ERR_SUCCESS:
            errors.append("订阅回执失败: {}".format(rc))
            ready.set()

    def on_subscribe(client, userdata, mid, granted_qos, properties=None):
        if not granted_qos or any(reason_value(code) >= 128 for code in granted_qos):
            errors.append("Broker 拒绝订阅回执 Topic")
        ready.set()

    def on_message(client, userdata, msg):
        if msg.retain or msg.topic != reply_topic:
            return
        try:
            body = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeError):
            return
        if not isinstance(body, dict) or body.get("tid") != message["tid"]:
            return
        print("\n[回执] " + json.dumps(body, ensure_ascii=False, indent=2), flush=True)
        data = body.get("data")
        if not isinstance(data, dict):
            return
        if body.get("method") == "task_transition" and data.get("stage") == "status_unknown":
            print("导航状态尚未确认，继续监听；不重发任务、不发送停止指令。", flush=True)
            return
        if body.get("method") == "task_completed" and data.get("result") == 0:
            outcome["result"] = 0
            completed.set()
        elif data.get("result") == -1:
            # 明确的失败或取消结束监听；status_unknown已在上方单独处理。
            outcome["result"] = 1
            completed.set()

    client_id = "sim-task-queue-" + uuid.uuid4().hex[:12]
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id=client_id, protocol=mqtt.MQTTv311)
    except (AttributeError, TypeError):
        client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    if args.username:
        client.username_pw_set(args.username, args.password)
    client.connect_timeout = args.timeout
    started = False
    try:
        client.connect(args.host, args.port, keepalive=60)
        client.loop_start()
        started = True
        if not ready.wait(args.timeout):
            raise RuntimeError("等待连接/订阅确认超时，未下发任务")
        if errors:
            raise RuntimeError(errors[0])
        if args.listen_only:
            print("只监听已有批次，不下发任务；历史回执不会补发。", flush=True)
        else:
            # 只在此处发送一次；重连回调只恢复订阅，不重发业务命令。
            info = client.publish(topic, json.dumps(message, ensure_ascii=False), qos=1, retain=False)
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError("发送状态不确定，rc={}；重试请沿用 tid={}".format(info.rc, message["tid"]))
            info.wait_for_publish(timeout=args.timeout)
            if not info.is_published():
                raise RuntimeError("未收到 PUBACK；重试请沿用 tid=" + message["tid"])
            print("\nBroker 已确认收包，等待机器人处理；队列顺序：" +
                  " → ".join(item["mainTaskName"] for item in message["data"]["tasks"]), flush=True)
        if args.wait == 0:
            print("已退出，不等待执行回执。")
            return 0
        print("持续监听回执，Ctrl+C退出；退出不会停止机器人。" if args.wait == -1 else
              "监听回执最多 {} 秒；退出监听不会停止机器人。".format(args.wait), flush=True)
        # 短周期等待，兼容 Windows 下 Ctrl+C 中断。
        deadline = None if args.wait == -1 else time.monotonic() + args.wait
        while not completed.is_set():
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                break
            completed.wait(0.5 if remaining is None else min(0.5, remaining))
        if not completed.is_set():
            print("监听超时，尚未确认整批完成；不会重发或停止任务。")
            return 2
        if outcome["result"] == 0:
            print("整批任务执行完成。")
            return 0
        print("收到失败/取消回执，请查看上面的报文；未发送停止指令。")
        return 1
    finally:
        client.disconnect()
        if started:
            client.loop_stop()


def main(argv=None):
    parser = argparse.ArgumentParser(description="模拟下发任务队列，默认任务7 → 任务10")
    parser.add_argument("--tasks", nargs="+", default=["任务7", "任务10"])
    parser.add_argument("--task-codes", nargs="+", help="真实任务编码，顺序与 --tasks 一致")
    parser.add_argument("--robot-code", default="DT202600001")
    parser.add_argument("--host", default=os.getenv("MQTT_HOST", "222.187.130.102"))
    parser.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")))
    parser.add_argument("--username", default=os.getenv("MQTT_USERNAME", "autocar"))
    parser.add_argument("--password", default=os.getenv("MQTT_PASSWORD", "123456"))
    parser.add_argument("--tid", help="重试同一批时填入原tid；默认生成新tid")
    parser.add_argument("--timeout", type=float, default=10, help="连接及发送超时秒数")
    parser.add_argument("--wait", type=float, default=-1, help="监听秒数；默认-1持续监听，0为只发不等")
    parser.add_argument("--listen-only", action="store_true", help="只监听指定tid，不发送任务")
    parser.add_argument("--dry-run", action="store_true", help="只打印报文，不联网，无需paho-mqtt")
    args = parser.parse_args(argv)
    if args.timeout <= 0 or (args.wait < 0 and args.wait != -1) or not 1 <= args.port <= 65535:
        parser.error("timeout必须大于0，wait必须为-1或非负数，port必须为1～65535")
    if args.listen_only and (not args.tid or not args.tid.strip() or args.wait == 0):
        parser.error("--listen-only 必须指定原 --tid，且 --wait 不能为0")
    try:
        message = build_message(args.robot_code, args.tasks, args.task_codes, args.tid)
    except ValueError as exc:
        parser.error(str(exc))
    if args.listen_only:
        print("监听 Topic: thing/robot/{}/services_reply  tid={}".format(
            message["data"]["robotCode"], message["tid"]), flush=True)
    else:
        print("Topic: thing/robot/{}/services  QoS=1  retain=false".format(message["data"]["robotCode"]))
        print(json.dumps(message, ensure_ascii=False, indent=2), flush=True)
    if args.dry_run:
        print("\n预览模式，未连接 MQTT、未发送任务。")
        return 0
    try:
        return send_and_listen(args, message)
    except KeyboardInterrupt:
        print("\n已退出下发/监听程序；机器人任务不会因此停止。")
        return 130
    except Exception as exc:
        print("错误: {}；本批 tid={}".format(exc, message["tid"]), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
