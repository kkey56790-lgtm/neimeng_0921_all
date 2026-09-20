#!/usr/bin/env python3
import json
import time
import uuid


class Topics:
    CONTROL = "/inspection/control"
    STATUS = "/inspection/status"
    EVENT = "/inspection/event"
    RESULT = "/inspection/result"
    HEALTH = "/inspection/health"
    POINT_EVENT = "/inspection/point_event"
    TASK_COMMAND = "/inspection/task_command"
    TASK_STATUS = "/inspection/task_status"
    ROBOT_CATALOG = "/inspection/robot_catalog"
    # 车体HTTP接口返回的真实状态快照，供UI和MQTT网关使用。
    ROBOT_TELEMETRY = "/inspection/robot_telemetry"
    ROBOT_COMMAND = "/inspection/robot_command"
    ROBOT_RESULT = "/inspection/robot_result"
    PTZ_COMMAND = "/inspection/ptz_command"
    PTZ_RESULT = "/inspection/ptz_result"
    DETECTION_CONTROL = "/inspection/detection_control"
    DETECTION_FRAME = "/inspection/detection_frame"
    DETECTION_RESULT = "/inspection/detection_result"


def now_ms():
    return int(time.time() * 1000)


def new_id(prefix="msg"):
    return "{}-{}".format(prefix, uuid.uuid4().hex[:16])


def make_message(message_type, data=None, **fields):
    message = {
        "version": "1.0",
        "message_id": fields.pop("message_id", new_id()),
        "timestamp_ms": fields.pop("timestamp_ms", now_ms()),
        "type": message_type,
        "data": data if isinstance(data, dict) else {},
    }
    message.update(fields)
    return message


def parse_message(raw):
    if hasattr(raw, "data"):
        raw = raw.data
    if isinstance(raw, dict):
        return raw
    if not isinstance(raw, str):
        raise ValueError("message must be JSON string or object")
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("message root must be an object")
    return value


def dumps(message):
    return json.dumps(message, ensure_ascii=False, separators=(",", ":"))
