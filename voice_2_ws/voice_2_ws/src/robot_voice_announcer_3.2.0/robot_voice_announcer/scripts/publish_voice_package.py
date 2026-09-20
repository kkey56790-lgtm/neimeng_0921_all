#!/usr/bin/env python3
"""Publish the six default voice definitions through the vehicle MQTT protocol."""

import argparse
import json
import os
import sys
import threading
import time
import uuid
from pathlib import Path

import paho.mqtt.client as mqtt
import rospkg
import yaml


VOICES = [
    {"name": "正在巡检中请勿靠近.wav", "text": "正在巡检中请勿靠近", "type": "Chinese"},
    {"name": "机器人停车请避让.wav", "text": "机器人停车请避让", "type": "Chinese"},
    {"name": "倒车请注意.wav", "text": "倒车请注意", "type": "Chinese"},
    {"name": "左转.wav", "text": "左转", "type": "Chinese"},
    {"name": "右转.wav", "text": "右转", "type": "Chinese"},
    {"name": "机器人避障中.wav", "text": "机器人避障中", "type": "Chinese"},
]


def load_credentials(path):
    with Path(path).expanduser().open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    username = str(data.get("mqtt_username", ""))
    password = str(data.get("mqtt_password", ""))
    if not username or not password:
        raise RuntimeError("凭据文件缺少 mqtt_username 或 mqtt_password")
    return username, password


def parse_args():
    parser = argparse.ArgumentParser(description="通过 MQTT 下发语音包更新、列表或播放命令。")
    parser.add_argument("--broker-host", default="222.187.130.102")
    parser.add_argument("--broker-port", type=int, default=1883)
    parser.add_argument("--robot-code", default="DT202600001")
    parser.add_argument("--credentials", default=None,
                        help="MQTT 凭据 YAML；默认读取本包 config 目录")
    parser.add_argument("--timeout", type=float, default=0.0,
                        help="等待回执秒数，默认 0 表示无限等待")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--list", action="store_true", help="获取机器人语音列表")
    action.add_argument("--play", metavar="FILE", help="播放指定远程 WAV")
    action.add_argument("--update", action="store_true", help="更新六个默认语音（默认动作）")
    return parser.parse_args()


def main():
    args = parse_args()
    credentials_path = args.credentials or os.path.join(
        rospkg.RosPack().get_path("robot_voice_announcer"),
        "config", "mqtt_credentials.example.yaml")
    try:
        username, password = load_credentials(credentials_path)
    except Exception as exc:
        print(json.dumps({"result": 1, "message": "读取 MQTT 凭据失败: {}".format(exc)}, ensure_ascii=False))
        return 2

    tid = str(uuid.uuid4())
    if args.list:
        method = "voice_package_list"
        data = {"robotCode": args.robot_code}
    elif args.play:
        method = "voice_switch"
        data = {"robotCode": args.robot_code, "fileName": args.play}
    else:
        method = "voice_package_update"
        data = {"robotCode": args.robot_code, "packageVersion": "1.0",
                "overwrite": True, "voices": VOICES}
    envelope = {"tid": tid, "method": method,
                "timestamp": int(time.time() * 1000), "data": data}

    service_topic = "thing/robot/{}/services".format(args.robot_code)
    reply_topic = "thing/robot/{}/services_reply".format(args.robot_code)
    done = threading.Event()
    result = {"reply": None, "error": None}

    def on_connect(client, _userdata, _flags, rc, _properties=None):
        if rc != 0:
            result["error"] = "MQTT 连接失败，返回码 {}".format(rc)
            done.set()
            return
        client.subscribe(reply_topic, qos=1)
    def on_subscribe(client, _userdata, _mid, _granted_qos, _properties=None):
        info = client.publish(service_topic,
                       json.dumps(envelope, ensure_ascii=False, separators=(",", ":")),
                       qos=1)
        print(json.dumps({"tid": tid, "stage": "publish",
                          "result": 0 if info.rc == 0 else 1,
                          "message": "已提交 MQTT 发送，等待机器人执行回执" if info.rc == 0 else "MQTT 发送失败"},
                         ensure_ascii=False), flush=True)
        if info.rc != 0:
            result["error"] = "MQTT 发送失败"
            done.set()

    def on_message(_client, _userdata, message):
        try:
            reply = json.loads(message.payload.decode("utf-8"))
            if str(reply.get("tid", "")) == tid:
                result["reply"] = reply
                done.set()
        except (UnicodeDecodeError, ValueError):
            pass

    client_id = "voice-publisher-{}-{}".format(args.robot_code, os.getpid())
    client = mqtt.Client(client_id=client_id, clean_session=True)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    try:
        client.connect(args.broker_host, args.broker_port, keepalive=30)
        client.loop_start()
        if not done.wait(args.timeout if args.timeout > 0 else None):
            print(json.dumps({"result": 1, "message": "等待设备回执超时，机器人执行结果未知"}, ensure_ascii=False))
            return 1
        if result["error"]:
            print(json.dumps({"result": 1, "message": result["error"]}, ensure_ascii=False))
            return 1
        print(json.dumps(result["reply"], ensure_ascii=False, indent=2))
        reply_data = result["reply"].get("data", {})
        if not isinstance(reply_data, dict) or reply_data.get("result") != 0:
            return 1
        return 0
    except Exception as exc:
        print(json.dumps({"result": 1, "message": str(exc)}, ensure_ascii=False))
        return 1
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    sys.exit(main())
