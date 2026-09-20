#!/usr/bin/env python3
"""Inspection panel that routes task-start tests through the real scheduler."""

import rospy
from PyQt5.QtWidgets import QMessageBox
from std_msgs.msg import String

from inspection_interfaces.protocol import dumps, make_message, new_id, parse_message
from inspection_panel import InspectionPanel


class ProductionInspectionPanel(InspectionPanel):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("内蒙古真实巡检任务面板")
        # The production entry point exposes only factual robot data. Manual
        # point/detection injection remains available in dedicated test launches.
        if self.main_pages.count() > 1:
            self.main_pages.removeTab(0)
            self.main_pages.setTabText(0, "真实巡检数据")

    def _catalog_cb(self, raw):
        try:
            message = parse_message(raw)
            source = str(message.get("source", ""))
            if source != "robot_bridge":
                self._append_execution_log(
                    "[目录已忽略] 非真实数据源：{}".format(source or "unknown")
                )
                return
            with self.lock:
                self.catalog = message.get("data", {})
                self.catalog_source = source
        except Exception:
            pass

    def test_start_task(self):
        name = self.task_test_combo.currentText().strip()
        if not name:
            QMessageBox.warning(self, "任务名为空", "请选择包含“任务N”的真实任务。")
            return
        with self.lock:
            catalog = dict(self.catalog)
            catalog_source = self.catalog_source
        real_task_names = [
            self._name(item) for item in catalog.get("tasks", [])
            if self._name(item)
        ]
        if catalog_source != "robot_bridge" or name not in real_task_names:
            QMessageBox.warning(self, "真实目录未就绪", "中台任务测试必须选择真实上位机任务目录中的名称。")
            return
        command_id = new_id("ui-platform")
        message = make_message(
            "inspection.control", source="inspection_ui",
            action="inspection.platform_task", command_id=command_id,
            data={"task_name": name},
        )
        self.control_pub.publish(String(data=dumps(message)))
        self._append_execution_log(
            "[统一调度任务] {}，command_id={}，等待真实HTTP回执".format(name, command_id))


if __name__ == "__main__":
    rospy.init_node("inspection_panel", disable_signals=True)
    app = __import__("PyQt5.QtWidgets", fromlist=["QApplication"]).QApplication([])
    window = ProductionInspectionPanel()
    window.show()
    raise SystemExit(app.exec_())


