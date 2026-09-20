#!/usr/bin/env python3
"""Full-auto monitor with current-map tasks and the complete real timeline."""

import os
import sys

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
if SOURCE_DIR not in sys.path:
    sys.path.insert(0, SOURCE_DIR)

import rospy
from PyQt5.QtWidgets import (
    QComboBox, QFormLayout, QGroupBox, QListWidget, QMessageBox, QPushButton)
from std_msgs.msg import String

from full_auto_test_panel import FullAutoTestPanel
from inspection_interfaces.protocol import Topics, dumps, make_message, new_id, parse_message


class FullAutoMapTestPanel(FullAutoTestPanel):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("真实自动巡检全流程监视（按当前地图）")
        self.map_combo = QComboBox()
        self.task_list = QListWidget()
        self.map_state = QGroupBox("当前地图与全部真实任务")
        form = QFormLayout(self.map_state)
        form.addRow("当前/目标地图", self.map_combo)
        self.task_list.setMaximumHeight(170)
        form.addRow("当前地图全部真实任务", self.task_list)
        button = QPushButton("切换地图（任务停止时）")
        button.clicked.connect(self.switch_map)
        form.addRow(button)
        self.layout().insertWidget(0, self.map_state)
        self.robot_pub = rospy.Publisher(
            Topics.ROBOT_COMMAND, String, queue_size=10)
        rospy.Subscriber(
            Topics.ROBOT_RESULT, String, self._robot_result, queue_size=20)

    def switch_map(self):
        name = self.map_combo.currentText().strip()
        if not name:
            return
        with self.lock:
            current = str(self.snapshot.get("current_map", ""))
        if name == current:
            return
        answer = QMessageBox.question(
            self, "确认切换地图",
            "确认从当前地图“{}”切换到“{}”？请先停止运行中的任务。".format(
                current or "未知", name),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        command = make_message(
            "robot.command", source="full_auto_map_test_panel",
            action="robot.map.switch", command_id=new_id("map-switch"),
            data={"map_name": name},
        )
        self.robot_pub.publish(String(data=dumps(command)))
        self._append_timeline("操作指令", command)

    def _robot_result(self, raw):
        try:
            message = parse_message(raw)
            if str(message.get("source", "")) != "robot_bridge":
                return
            self._append_timeline("车体HTTP回执", message)
        except Exception:
            pass

    def _set_task_list(self, tasks):
        existing = [
            self.task_list.item(index).text()
            for index in range(self.task_list.count())
        ]
        if existing == tasks:
            return
        self.task_list.clear()
        self.task_list.addItems(tasks)

    def refresh(self):
        super().refresh()
        with self.lock:
            maps = list(self.snapshot.get("maps", []))
            current = str(self.snapshot.get("current_map", ""))
            tasks = list(self.snapshot.get("catalog", []))
        self._set_combo(self.map_combo, maps, current)
        self._set_task_list(tasks)
        self.map_state.setTitle(
            "当前地图与全部真实任务（地图：{}，任务：{} 个）".format(
                current or "未知", len(tasks)
            )
        )


if __name__ == "__main__":
    rospy.init_node("full_auto_test_panel", disable_signals=True)
    from PyQt5.QtWidgets import QApplication
    app = QApplication([])
    panel = FullAutoMapTestPanel()
    panel.show()
    raise SystemExit(app.exec_())
