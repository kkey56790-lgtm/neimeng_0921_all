#!/usr/bin/env python3
"""Real full-auto inspection monitor with a structured execution timeline."""

import datetime
import json
import re
import threading
from collections import deque

import rospy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QAbstractItemView, QApplication, QComboBox, QFormLayout, QGridLayout,
    QGroupBox, QHeaderView, QLabel, QPlainTextEdit, QPushButton, QTabWidget,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, new_id, parse_message


TASK_NUMBER_RE = re.compile(r"任务\s*0*(\d+)(?!\d)")

FLOW_LABELS = {
    "IDLE": "空闲",
    "RUNNING": "巡检运行",
    "PAUSED": "已暂停",
    "STARTING_NUMBERED_TASK": "正在启动编号任务",
    "NUMBERED_TASK_RUNNING": "编号任务运行中",
    "WAITING_YOLO": "起始点检测中",
    "YOLO_ERROR_WAITING_OPERATOR": "检测异常，等待人工处理",
    "OBSTACLE_WAIT": "检测到障碍，等待清除",
    "REVERSING": "障碍超时，正在倒车",
    "RETURNING_START_AFTER_REVERSE": "倒车完成，等待返回起始点",
    "RECHARGING": "正在执行回充路线",
    "CHARGING_TO_80": "充电中，等待达到目标电量",
    "ERROR": "流程异常",
}

EVENT_LABELS = {
    "NUMBERED_FLOW_STARTED": "自动巡检流程已启动",
    "NUMBERED_TASK_START_REQUESTED": "已请求启动编号任务",
    "NUMBERED_TASK_FINISHED": "编号任务已完成",
    "PLATFORM_TASK_QUEUED": "中台任务已进入队列",
    "PLATFORM_TASK_REPLACED_DURING_RECHARGE": "回充期间中台任务已更新",
    "DETECTION_STARTED": "已到起始点，YOLO开始采集",
    "TRUCK_CONFIRMED": "检测到车辆，继续当前编号任务",
    "NO_TRUCK_TASK_SKIPPED": "未检测到车辆，跳过并进入下一任务",
    "NO_TRUCK_TASK_CONTINUED": "未检测到车辆，保持并继续当前任务（实测保护）",
    "YOLO_ERROR_TASK_HELD": "YOLO检测异常，任务保持",
    "YOLO_SKIPPED_NON_START_POINT": "非起始点，不启动YOLO",
    "YOLO_SKIPPED_TASK_POINT_MISMATCH": "点位编号与任务编号不一致",
    "INSPECTION_DETECTION_STARTED": "非起始点开始采集非truck巡检项目",
    "NON_TRUCK_INSPECTION_RESULT": "非truck巡检结果已记录",
    "REAL_TASK_FAILED": "真实底盘任务失败",
    "REAL_TASK_CANCELLED": "真实底盘任务已取消",
    "LOW_BATTERY_RECHARGE_REQUESTED": "低电量，已请求回充",
    "RECHARGE_STARTED": "回充任务已启动",
    "RECHARGE_ROUTE_FINISHED": "已到充电位置",
    "RECHARGE_COMPLETED": "充电完成，恢复巡检",
    "OBSTACLE_TIMER_STARTED": "检测到障碍，任务已暂停",
    "OBSTACLE_CLEARED_TASK_RESUMED": "障碍清除，任务已继续",
    "REVERSE_REQUESTED": "障碍超时，已请求安全倒车",
    "REVERSE_COMPLETED": "安全倒车完成",
    "REVERSE_FAILED": "安全倒车失败",
    "REVERSE_RETURN_POINT_CONFIRMED": "已返回任务起始点",
    "TASK_FINISH_HELD_FOR_YOLO": "底盘任务完成，等待YOLO结论",
    "TRUCK_CONFIRMED_AFTER_TASK_FINISH": "任务完成后确认有车，重新执行",
    "PLATFORM_TASK_REJECTED": "中台任务被拒绝",
    "INSPECTION_START_REJECTED": "巡检启动被拒绝",
}


class FullAutoTestPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("真实自动巡检全流程监视")
        self.resize(1220, 780)
        self.lock = threading.RLock()
        self.snapshot = {
            "inspection": {}, "telemetry": {}, "task": {}, "detection": {},
            "ptz": {}, "health": {}, "catalog": [], "maps": [],
            "current_map": "",
        }
        self.logs = deque(maxlen=100)
        self.timeline = deque(maxlen=100)
        self.last_timeline_signatures = {}
        self.timeline_version = 0
        self.displayed_timeline_version = -1
        self.displayed_log_version = -1
        self.control_pub = rospy.Publisher(Topics.CONTROL, String, queue_size=20)

        rospy.Subscriber(Topics.STATUS, String, self._inspection, queue_size=1)
        rospy.Subscriber(Topics.EVENT, String, self._event, queue_size=50)
        rospy.Subscriber(Topics.RESULT, String, self._result, queue_size=30)
        rospy.Subscriber(Topics.DETECTION_RESULT, String, self._detection, queue_size=30)
        rospy.Subscriber(Topics.TASK_STATUS, String, self._task, queue_size=30)
        rospy.Subscriber(Topics.ROBOT_TELEMETRY, String, self._telemetry, queue_size=1)
        rospy.Subscriber(Topics.PTZ_RESULT, String, self._ptz, queue_size=30)
        rospy.Subscriber(Topics.HEALTH, String, self._health, queue_size=30)
        rospy.Subscriber(Topics.ROBOT_CATALOG, String, self._catalog, queue_size=1)

        self.auto_task = QComboBox()
        self.platform_task = QComboBox()
        self.labels = {name: QLabel("--") for name in (
            "flow", "task", "point", "battery", "obstacle", "yolo",
            "ptz", "queue", "nodes", "error",
        )}
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.timeline_table = QTableWidget(0, 7)
        self._build()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    def _build(self):
        root = QVBoxLayout(self)
        controls = QGroupBox("真实巡检控制")
        form = QFormLayout(controls)
        form.addRow("自动巡检任务0", self.auto_task)
        form.addRow("中台下发任务（当前地图）", self.platform_task)
        buttons = QGridLayout()
        actions = [
            ("启动自动巡检", self.start_auto),
            ("下发中台任务", self.start_platform),
            ("暂停", lambda: self.control("inspection.pause")),
            ("继续", lambda: self.control("inspection.resume")),
            ("停止", lambda: self.control("inspection.stop")),
        ]
        for index, (text, callback) in enumerate(actions):
            button = QPushButton(text)
            button.clicked.connect(callback)
            buttons.addWidget(button, index // 3, index % 3)
        form.addRow(buttons)
        root.addWidget(controls)

        status = QGroupBox("当前运行阶段（全部为真实设备/编排数据）")
        grid = QGridLayout(status)
        names = [
            ("流程状态", "flow"), ("真实任务", "task"),
            ("当前点位", "point"), ("电量", "battery"),
            ("障碍处理", "obstacle"), ("YOLO truck", "yolo"),
            ("云台", "ptz"), ("中台队列", "queue"),
            ("节点健康", "nodes"), ("异常", "error"),
        ]
        for index, (title, key) in enumerate(names):
            row = index // 2
            column = (index % 2) * 2
            grid.addWidget(QLabel(title), row, column)
            grid.addWidget(self.labels[key], row, column + 1)
        root.addWidget(status)

        self.timeline_table.setHorizontalHeaderLabels(
            ["时间", "阶段", "流程/状态", "地图", "任务", "点位", "真实详情"])
        self.timeline_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.timeline_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.timeline_table.setAlternatingRowColors(True)
        self.timeline_table.verticalHeader().setVisible(False)
        header = self.timeline_table.horizontalHeader()
        for column in range(6):
            header.setSectionResizeMode(column, QHeaderView.Interactive)
        header.setSectionResizeMode(6, QHeaderView.Stretch)
        for column, width in enumerate((95, 120, 190, 120, 160, 140)):
            self.timeline_table.setColumnWidth(column, width)

        self.details = QTabWidget()
        self.details.addTab(self.timeline_table, "全流程时间线（最新在上）")
        self.details.addTab(self.log_view, "原始事件日志")
        root.addWidget(self.details, 1)

    def control(self, action, data=None, command_id=""):
        message = make_message(
            "inspection.control", source="full_auto_test_panel", action=action,
            command_id=command_id or new_id("panel"), data=data or {},
        )
        self.control_pub.publish(String(data=dumps(message)))
        self._append_timeline("操作指令", message)

    def start_auto(self):
        name = self.auto_task.currentText().strip()
        if name:
            self.control("inspection.start", {"task_name": name, "loop_time": 1})

    def start_platform(self):
        name = self.platform_task.currentText().strip()
        if name:
            self.control(
                "inspection.platform_task", {"task_name": name}, new_id("platform"))

    @staticmethod
    def _scalar(node, keys):
        if isinstance(node, dict):
            for key, value in node.items():
                if str(key).lower() in keys and not isinstance(value, (dict, list)):
                    return value
            for value in node.values():
                found = FullAutoTestPanel._scalar(value, keys)
                if found not in (None, ""):
                    return found
        elif isinstance(node, list):
            for value in node:
                found = FullAutoTestPanel._scalar(value, keys)
                if found not in (None, ""):
                    return found
        return None

    @staticmethod
    def _task_name(message):
        data = message.get("data", {}) if isinstance(message.get("data"), dict) else {}
        flow = data.get("numbered_flow", {}) if isinstance(data.get("numbered_flow"), dict) else {}
        current = flow.get("current_execution") or data.get("current_execution") or {}
        decision = data.get("decision") or {}
        candidates = [
            message.get("task_name"),
            current.get("name") if isinstance(current, dict) else "",
            data.get("main_task_name"), data.get("task_name"), data.get("taskName"),
            data.get("name"),
            decision.get("task_name") if isinstance(decision, dict) else "",
        ]
        return next((str(value) for value in candidates if value not in (None, "")), "")

    @staticmethod
    def _point_name(message):
        data = message.get("data", {}) if isinstance(message.get("data"), dict) else {}
        return str(message.get("point_name") or data.get("point_name") or "")

    @staticmethod
    def _event_code(message):
        return str(
            message.get("event") or message.get("action") or
            message.get("command_status") or message.get("state") or
            message.get("status") or message.get("type") or ""
        )

    @staticmethod
    def _time_text(message):
        try:
            timestamp = int(message.get("timestamp_ms", 0)) / 1000.0
            if timestamp > 0:
                return datetime.datetime.fromtimestamp(timestamp).strftime("%H:%M:%S.%f")[:-3]
        except (TypeError, ValueError, OSError):
            pass
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    @staticmethod
    def _detail_text(message):
        data = message.get("data", {})
        if not isinstance(data, dict):
            data = {"value": data}
        cleaned = {
            key: value for key, value in data.items()
            if key not in ("preview_jpeg_b64", "image", "frame")
        }
        text = json.dumps(cleaned, ensure_ascii=False, default=str)
        return text if len(text) <= 320 else text[:317] + "..."

    def _append_timeline(self, kind, message, signature=None):
        if not isinstance(message, dict):
            return
        code = self._event_code(message)
        label = EVENT_LABELS.get(code, code or str(message.get("type", "")))
        with self.lock:
            if signature is not None:
                if self.last_timeline_signatures.get(kind) == signature:
                    return
                self.last_timeline_signatures[kind] = signature
            map_name = str(self.snapshot.get("current_map", ""))
            entry = {
                "time": self._time_text(message),
                "kind": kind,
                "event": label,
                "map": map_name,
                "task": self._task_name(message),
                "point": self._point_name(message),
                "detail": self._detail_text(message),
            }
            self.timeline.appendleft(entry)
            self.logs.appendleft(
                "{time} | {kind} | {event} | 地图={map} | 任务={task} | "
                "点位={point} | {detail}".format(**entry))
            self.timeline_version += 1

    def _store(self, key, raw):
        try:
            message = parse_message(raw)
            with self.lock:
                self.snapshot[key] = message
            data = message.get("data", {}) if isinstance(message.get("data"), dict) else {}
            if key == "inspection":
                flow = data.get("numbered_flow", {}) if isinstance(data.get("numbered_flow"), dict) else {}
                current = flow.get("current_execution") or {}
                signature = (
                    message.get("state"), message.get("point_name"),
                    current.get("name") if isinstance(current, dict) else "",
                    data.get("error", ""),
                )
                self._append_timeline("编排状态", message, signature)
            elif key == "task":
                signature = (
                    message.get("command_status"), data.get("task_state"),
                    data.get("main_task_name") or data.get("task_name"),
                    data.get("error") or data.get("error_info"),
                )
                self._append_timeline("底盘任务", message, signature)
            elif key == "detection":
                detection_id = data.get("detection_id") or message.get("message_id")
                signature = (
                    detection_id, message.get("state"), message.get("point_name"),
                    data.get("detected_frames"), data.get("total_frames"),
                )
                self._append_timeline("YOLO结果", message, signature)
            elif key == "ptz":
                signature = (
                    message.get("action"), message.get("status"),
                    data.get("preset"), data.get("device_state"), data.get("error"),
                )
                self._append_timeline("云台回执", message, signature)
        except Exception:
            pass

    def _inspection(self, raw):
        self._store("inspection", raw)

    def _task(self, raw):
        self._store("task", raw)

    def _telemetry(self, raw):
        self._store("telemetry", raw)

    def _detection(self, raw):
        self._store("detection", raw)

    def _ptz(self, raw):
        self._store("ptz", raw)

    def _health(self, raw):
        try:
            message = parse_message(raw)
            source = str(message.get("source", "unknown"))
            with self.lock:
                self.snapshot["health"][source] = message
            data = message.get("data", {}) if isinstance(message.get("data"), dict) else {}
            signature = (source, message.get("state"), data.get("error", ""))
            self._append_timeline("节点健康/{}".format(source), message, signature)
        except Exception:
            pass

    @staticmethod
    def _map_name(value):
        if isinstance(value, dict):
            return str(
                value.get("name") or value.get("map_name") or
                value.get("mapName") or ""
            ).strip()
        return str(value or "").strip()

    def _catalog(self, raw):
        try:
            message = parse_message(raw)
            if str(message.get("source", "")) != "robot_bridge":
                return
            data = message.get("data", {})
            tasks = data.get("tasks", [])
            names = [
                str(item.get("name") or item.get("task_name") or "").strip()
                for item in tasks if isinstance(item, dict)
            ]
            maps = [self._map_name(item) for item in data.get("maps", [])]
            current_map = self._map_name(data.get("current_map", {}))
            with self.lock:
                self.snapshot["catalog"] = list(dict.fromkeys(
                    name for name in names if name))
                self.snapshot["maps"] = list(dict.fromkeys(
                    name for name in maps if name))
                self.snapshot["current_map"] = current_map
            signature = (current_map, tuple(self.snapshot["catalog"]))
            catalog_message = dict(message)
            catalog_message["state"] = "CATALOG_READY"
            catalog_message["data"] = {
                "current_map": current_map,
                "task_count": len(self.snapshot["catalog"]),
            }
            self._append_timeline("真实目录", catalog_message, signature)
        except Exception:
            pass

    def _append_log(self, kind, raw):
        try:
            self._append_timeline(kind, parse_message(raw))
        except Exception:
            pass

    def _event(self, raw):
        self._append_log("编排事件", raw)

    def _result(self, raw):
        self._append_log("巡检结果", raw)

    @staticmethod
    def _set_label(label, value):
        value = str(value)
        if label.text() != value:
            label.setText(value)

    @staticmethod
    def _set_combo(combo, values, preferred=""):
        values = list(values)
        current = combo.currentText()
        target = current if current in values else preferred
        existing = [combo.itemText(index) for index in range(combo.count())]
        if existing == values:
            if target and current != target:
                combo.setCurrentText(target)
            return

        combo.blockSignals(True)
        combo.clear()
        combo.addItems(values)
        combo.setCurrentText(target)
        combo.blockSignals(False)

    @staticmethod
    def _fill_timeline_row(table, row, entry):
        columns = ("time", "kind", "event", "map", "task", "point", "detail")
        for column, key in enumerate(columns):
            table.setItem(row, column, QTableWidgetItem(str(entry.get(key, ""))))

    def _refresh_timeline(self, timeline, logs, version):
        if timeline is not None and version != self.displayed_timeline_version:
            delta = version - self.displayed_timeline_version
            full_rebuild = (
                self.displayed_timeline_version < 0
                or delta <= 0
                or delta > len(timeline)
                or self.timeline_table.rowCount() > len(timeline)
            )
            self.timeline_table.setUpdatesEnabled(False)
            try:
                if full_rebuild:
                    self.timeline_table.setRowCount(len(timeline))
                    for row, entry in enumerate(timeline):
                        self._fill_timeline_row(self.timeline_table, row, entry)
                else:
                    for entry in reversed(timeline[:delta]):
                        self.timeline_table.insertRow(0)
                        self._fill_timeline_row(self.timeline_table, 0, entry)
                    while self.timeline_table.rowCount() > len(timeline):
                        self.timeline_table.removeRow(self.timeline_table.rowCount() - 1)
            finally:
                self.timeline_table.setUpdatesEnabled(True)
            self.displayed_timeline_version = version

        if (
            logs is not None
            and self.details.currentWidget() is self.log_view
            and version != self.displayed_log_version
        ):
            self.log_view.setPlainText("\n".join(logs))
            self.displayed_log_version = version
    def refresh(self):
        show_raw_log = self.details.currentWidget() is self.log_view
        with self.lock:
            data = dict(self.snapshot)
            timeline_version = self.timeline_version
            timeline = list(self.timeline) \
                if timeline_version != self.displayed_timeline_version else None
            logs = list(self.logs) \
                if show_raw_log and timeline_version != self.displayed_log_version else None

        catalog = list(data.get("catalog", []))
        auto = [
            name for name in catalog
            if TASK_NUMBER_RE.search(name)
            and int(TASK_NUMBER_RE.search(name).group(1)) == 0
        ]
        platform = [
            name for name in catalog
            if TASK_NUMBER_RE.search(name)
            and 1 <= int(TASK_NUMBER_RE.search(name).group(1)) <= 10
        ]
        self._set_combo(self.auto_task, auto, auto[0] if auto else "")
        self._set_combo(
            self.platform_task, platform, platform[0] if platform else "")

        inspection = data.get("inspection", {})
        inspection_data = (
            inspection.get("data", {})
            if isinstance(inspection.get("data"), dict) else {}
        )
        flow = inspection_data.get("numbered_flow", {})
        flow = flow if isinstance(flow, dict) else {}
        current = flow.get("current_execution") or {}
        task = data.get("task", {}).get("data", {})
        task = task if isinstance(task, dict) else {}
        telemetry = data.get("telemetry", {}).get("data", {})
        telemetry = telemetry if isinstance(telemetry, dict) else {}
        detection = data.get("detection", {})
        detect_data = (
            detection.get("data", {})
            if isinstance(detection.get("data"), dict) else {}
        )
        ptz = data.get("ptz", {})
        health = data.get("health", {})

        state = str(inspection.get("state", "等待"))
        self._set_label(self.labels["flow"], 
            "{}（{}）".format(FLOW_LABELS.get(state, state), state))
        self._set_label(self.labels["task"], "{} / 类型={} / 底盘={}".format(
            current.get("name", task.get("main_task_name", "无")),
            current.get("kind", "--"), task.get("task_state", "--")))
        expected = flow.get("expected_detection") or {}
        self._set_label(self.labels["point"], "{} (#{}，待检测={})".format(
            inspection.get("point_name", "--"), inspection.get("point_seq", 0),
            expected.get("point_name", "无") if isinstance(expected, dict) else "无"))

        battery = flow.get("battery")
        if battery is None:
            battery = self._scalar(
                telemetry, {"battery", "battery_soc", "soc", "percent", "power"})
        self._set_label(self.labels["battery"], 
            "{}% / 回充等待={} / 充电={}".format(
                battery if battery is not None else "--",
                flow.get("recharge_pending", False),
                flow.get("charging", False),
            )
        )
        obstacle = self._scalar(
            telemetry, {"obstacle", "obstacle_state", "collision"})
        self._set_label(self.labels["obstacle"], 
            "状态={} / 已等待={}秒".format(
                obstacle if obstacle is not None else "--",
                flow.get("obstacle_elapsed", 0),
            )
        )
        self._set_label(self.labels["yolo"], 
            "{} / 帧={} / 命中={} / 置信度={}".format(
                detection.get("state", "等待起始点"),
                detect_data.get("total_frames", 0),
                detect_data.get("detected_frames", 0),
                detect_data.get("best_confidence", 0),
            )
        )
        ptz_data = ptz.get("data", {}) if isinstance(ptz.get("data"), dict) else {}
        self._set_label(self.labels["ptz"], "{} / {} / 预置点{}".format(
            ptz.get("status", "等待"), ptz_data.get("device_state", "--"),
            ptz_data.get("preset", "--")))
        queue = flow.get("platform_queue", [])
        queue_names = [
            item.get("name", str(item)) if isinstance(item, dict) else str(item)
            for item in queue
        ]
        self._set_label(self.labels["queue"], 
            " → ".join(queue_names) if queue_names else "空")
        self._set_label(self.labels["nodes"], "  ".join(
            "{}={}".format(source, message.get("state", "--"))
            for source, message in sorted(health.items())
        ) or "等待节点心跳")
        self._set_label(self.labels["error"], 
            str(inspection_data.get("error", "")) or "无")

        self._refresh_timeline(timeline, logs, timeline_version)


if __name__ == "__main__":
    rospy.init_node("full_auto_test_panel", disable_signals=True)
    app = QApplication([])
    panel = FullAutoTestPanel()
    panel.show()
    raise SystemExit(app.exec_())
