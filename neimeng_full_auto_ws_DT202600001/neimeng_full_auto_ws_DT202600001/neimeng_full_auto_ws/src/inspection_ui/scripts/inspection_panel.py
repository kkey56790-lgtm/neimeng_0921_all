#!/usr/bin/env python3
import base64
import json
import sys
import threading

import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, new_id, parse_message

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QSpinBox, QTableWidget, QTableWidgetItem, QTabWidget,
    QTextEdit, QVBoxLayout, QWidget,
)


class InspectionPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("内蒙古巡检任务面板")
        self.resize(1450, 820)
        self.lock = threading.Lock()
        self.status = {}
        self.health = {}
        self.health_detail = {}
        self.results = []
        self.logs = []
        self.catalog = {}
        self.catalog_source = ""
        self.ptz_feedback = {"status": "等待云台回执", "data": {}}
        self.task_feedback = {"state": "等待底盘状态"}
        self.detection_feedback = {"state": "等待检测结果"}
        self.robot_feedback = {"status": "等待车体回执", "data": {}}
        self.task_transition = {"point": "", "detection": "", "requested": "", "actual": "", "status": "等待触发"}
        self.robot_move_direction = ""
        self.preview_b64 = ""
        self.preview_frame_id = -1
        self.displayed_preview_frame_id = -1

        self.control_pub = rospy.Publisher(Topics.CONTROL, String, queue_size=20)
        self.ptz_pub = rospy.Publisher(Topics.PTZ_COMMAND, String, queue_size=20)
        self.robot_pub = rospy.Publisher(Topics.ROBOT_COMMAND, String, queue_size=30)
        self.point_pub = rospy.Publisher(Topics.POINT_EVENT, String, queue_size=20)
        self.detection_pub = rospy.Publisher(Topics.DETECTION_RESULT, String, queue_size=20)
        self.task_command_pub = rospy.Publisher(Topics.TASK_COMMAND, String, queue_size=20)
        rospy.Subscriber(Topics.STATUS, String, self._status_cb, queue_size=20)
        rospy.Subscriber(Topics.HEALTH, String, self._health_cb, queue_size=50)
        rospy.Subscriber(Topics.EVENT, String, self._event_cb, queue_size=50)
        rospy.Subscriber(Topics.RESULT, String, self._result_cb, queue_size=30)
        rospy.Subscriber(Topics.ROBOT_CATALOG, String, self._catalog_cb, queue_size=5)
        rospy.Subscriber(Topics.PTZ_RESULT, String, self._ptz_result_cb, queue_size=30)
        rospy.Subscriber(Topics.TASK_STATUS, String, self._task_status_cb, queue_size=30)
        rospy.Subscriber(Topics.DETECTION_RESULT, String, self._detection_result_cb, queue_size=20)
        rospy.Subscriber(Topics.ROBOT_RESULT, String, self._robot_result_cb, queue_size=30)
        rospy.Subscriber(Topics.DETECTION_FRAME, String, self._detection_frame_cb, queue_size=5)

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        self.summary = QLabel("系统状态：等待ROS数据")
        layout.addWidget(self.summary)
        self.data_source = QLabel("事实数据源：等待上位机")
        layout.addWidget(self.data_source)
        self.vehicle_state_label = QLabel("车体状态：定位=等待  雷达=等待  IMU=等待  电量=等待  位姿=等待")
        layout.addWidget(self.vehicle_state_label)

        feedback_group = QGroupBox("真实执行反馈（设备回执）")
        feedback_layout = QGridLayout(feedback_group)
        self.ptz_feedback_label = QLabel("云台：等待连接")
        self.task_feedback_label = QLabel("底盘：等待连接")
        self.detection_feedback_label = QLabel("检测：等待结果")
        self.robot_feedback_label = QLabel("车体：等待连接")
        self.task_transition_label = QLabel("任务切换：等待有车/无车决策")
        feedback_layout.addWidget(self.ptz_feedback_label, 0, 0)
        feedback_layout.addWidget(self.task_feedback_label, 0, 1)
        feedback_layout.addWidget(self.detection_feedback_label, 0, 2)
        feedback_layout.addWidget(self.robot_feedback_label, 0, 3)
        feedback_layout.addWidget(self.task_transition_label, 1, 0, 1, 4)
        layout.addWidget(feedback_group)

        self.main_pages = QTabWidget()
        simulation_page = QWidget()
        real_page = QWidget()
        simulation_page_layout = QVBoxLayout(simulation_page)
        real_page_layout = QVBoxLayout(real_page)
        self.main_pages.addTab(simulation_page, "手动信号测试（真实数据）")
        self.main_pages.addTab(real_page, "真实巡检数据")
        layout.addWidget(self.main_pages, 1)

        task_group = QGroupBox("任务控制")
        task_layout = QHBoxLayout(task_group)
        self.map_combo = QComboBox()
        self.task_combo = QComboBox()
        # 任务只能选择上位机目录中的真实任务，避免 UI 构造不存在的名称。
        self.task_combo.setEditable(False)
        self.task_combo.currentIndexChanged.connect(self.refresh_route)
        task_layout.addWidget(QLabel("当前地图"))
        self.current_map_label = QLabel("实际：等待读取")
        task_layout.addWidget(self.current_map_label)
        task_layout.addWidget(self.map_combo)
        switch_map_button = QPushButton("切换地图")
        switch_map_button.clicked.connect(self.switch_map)
        task_layout.addWidget(switch_map_button)
        refresh_button = QPushButton("刷新真实目录")
        refresh_button.clicked.connect(lambda: self.robot_command("catalog.refresh"))
        task_layout.addWidget(refresh_button)
        task_layout.addWidget(QLabel("当前地图任务"))
        task_layout.addWidget(self.task_combo)
        for title, action in (("启动", "start"), ("暂停", "pause"), ("继续", "resume"), ("停止", "stop"), ("重检当前点", "retry_point")):
            button = QPushButton(title)
            button.clicked.connect(lambda checked=False, value=action: self.send_control(value))
            task_layout.addWidget(button)
        real_page_layout.addWidget(task_group)

        controls = QHBoxLayout()
        self.tabs = QTabWidget()
        self.route_table = QTableWidget(0, 6)
        self.route_table.setHorizontalHeaderLabels(["序号", "巡航点", "轨道", "地图", "速度", "任务类型"])
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["序号", "点位", "结果", "置信度", "扫描", "后续任务", "任务ID"])
        self.tabs.addTab(self.route_table, "任务轨道/巡航点")
        self.tabs.addTab(self.table, "巡检结果")
        self.real_catalog_table = QTableWidget(0, 5)
        self.real_catalog_table.setHorizontalHeaderLabels(["类型", "名称", "地图/X", "Y", "详细信息"])
        self.tabs.addTab(self.real_catalog_table, "真实点位/轨道")
        self.report_table = QTableWidget(0, 8)
        self.report_table.setHorizontalHeaderLabels(["主任务", "子任务", "类型", "状态", "地图", "剩余距离", "开始时间", "错误"])
        self.tabs.addTab(self.report_table, "底盘任务报告")
        real_page_layout.addWidget(self.tabs, 3)

        robot_group = QGroupBox("车体手动控制（真实HTTP）")
        robot_layout = QGridLayout(robot_group)
        self.linear_speed = QDoubleSpinBox()
        self.linear_speed.setRange(0.01, 0.50)
        self.linear_speed.setSingleStep(0.05)
        self.linear_speed.setValue(0.20)
        self.angular_speed = QDoubleSpinBox()
        self.angular_speed.setRange(0.05, 1.00)
        self.angular_speed.setSingleStep(0.05)
        self.angular_speed.setValue(0.40)
        robot_layout.addWidget(QLabel("线速度 m/s"), 0, 0)
        robot_layout.addWidget(self.linear_speed, 0, 1)
        robot_layout.addWidget(QLabel("角速度 rad/s"), 1, 0)
        robot_layout.addWidget(self.angular_speed, 1, 1)
        body_buttons = {
            "forward": ("前进", 2, 1), "left": ("左转", 3, 0),
            "stop": ("停止", 3, 1), "right": ("右转", 3, 2),
            "backward": ("后退", 4, 1),
        }
        for direction, (title, row, column) in body_buttons.items():
            button = QPushButton(title)
            if direction == "stop":
                button.clicked.connect(self.robot_stop)
            else:
                # 车体采用监听/保持模式：点击方向后持续续发，直到点击“停止”。
                button.clicked.connect(lambda checked=False, value=direction: self.robot_move(value))
            robot_layout.addWidget(button, row, column)
        controls.addWidget(robot_group, 1)

        ptz_group = QGroupBox("云台手动控制")
        ptz_layout = QGridLayout(ptz_group)
        positions = {"up": (0, 1), "left": (1, 0), "stop": (1, 1), "right": (1, 2), "down": (2, 1)}
        for direction, position in positions.items():
            button = QPushButton({"up": "↑", "down": "↓", "left": "←", "right": "→", "stop": "停止"}[direction])
            if direction == "stop":
                button.clicked.connect(self.ptz_stop)
            else:
                button.pressed.connect(lambda value=direction: self.ptz_move(value))
                button.released.connect(self.ptz_stop)
            ptz_layout.addWidget(button, *position)
        for index in range(1, 5):
            button = QPushButton("预置点{}".format(index))
            button.clicked.connect(lambda checked=False, value=index: self.ptz_preset(value))
            ptz_layout.addWidget(button, 3 + (index - 1) // 2, (index - 1) % 2)
        auto_button = QPushButton("切回自动")
        auto_button.clicked.connect(lambda: self.ptz_command("mode", {"mode": "auto"}, "auto"))
        ptz_layout.addWidget(auto_button, 5, 0, 1, 2)
        controls.addWidget(ptz_group, 1)
        simulation_page_layout.addLayout(controls)

        simulation_group = QGroupBox("测试信号（名称来自真实上位机，只模拟触发时刻/检测结论）")
        simulation_layout = QHBoxLayout(simulation_group)
        self.point_combo = QComboBox()
        self.point_combo.setEditable(True)
        self.point_sequence = QSpinBox()
        self.point_sequence.setRange(0, 9999)
        self.point_sequence.setValue(1)
        simulation_layout.addWidget(QLabel("巡航点名（可选择/输入）"))
        simulation_layout.addWidget(self.point_combo, 2)
        simulation_layout.addWidget(QLabel("序号"))
        simulation_layout.addWidget(self.point_sequence)
        for title, event in (("模拟接近", "approach"), ("模拟到达", "arrived"), ("模拟离开", "leave")):
            button = QPushButton(title)
            button.clicked.connect(lambda checked=False, value=event: self.simulate_point(value))
            simulation_layout.addWidget(button)
        vehicle_button = QPushButton("模拟有车")
        vehicle_button.clicked.connect(lambda: self.simulate_detection("VEHICLE_PRESENT", 0.88))
        simulation_layout.addWidget(vehicle_button)
        empty_button = QPushButton("模拟无车")
        empty_button.clicked.connect(lambda: self.simulate_detection("VEHICLE_ABSENT", 0.0))
        simulation_layout.addWidget(empty_button)
        error_button = QPushButton("模拟检测异常")
        error_button.clicked.connect(lambda: self.simulate_detection("DETECTION_ERROR", 0.0))
        simulation_layout.addWidget(error_button)
        simulation_page_layout.addWidget(simulation_group)

        task_test_group = QGroupBox("任务指令测试（任务来自真实目录，结果以底盘HTTP回执为准）")
        task_test_layout = QHBoxLayout(task_test_group)
        self.task_test_combo = QComboBox()
        self.task_test_combo.setEditable(True)
        task_test_layout.addWidget(QLabel("任务名"))
        task_test_layout.addWidget(self.task_test_combo, 2)
        start_test_task = QPushButton("测试启动任务")
        start_test_task.clicked.connect(self.test_start_task)
        task_test_layout.addWidget(start_test_task)
        start_flow_button = QPushButton("启动主流程")
        start_flow_button.clicked.connect(self.start_simulation_flow)
        task_test_layout.addWidget(start_flow_button)
        stop_test_task = QPushButton("测试停止任务")
        stop_test_task.clicked.connect(lambda: self.test_task_command("stop"))
        task_test_layout.addWidget(stop_test_task)
        self.vehicle_task_combo = QComboBox()
        self.vehicle_task_combo.setEditable(True)
        self.empty_task_combo = QComboBox()
        self.empty_task_combo.setEditable(True)
        task_test_layout.addWidget(QLabel("有车切换"))
        task_test_layout.addWidget(self.vehicle_task_combo, 1)
        task_test_layout.addWidget(QLabel("无车切换"))
        task_test_layout.addWidget(self.empty_task_combo, 1)
        self.override_branch_tasks = QCheckBox("临时覆盖配置分支")
        self.override_branch_tasks.setChecked(False)
        task_test_layout.addWidget(self.override_branch_tasks)
        simulation_page_layout.addWidget(task_test_group)

        arrival_task_group = QGroupBox("到点巡检任务（与云台动作、有车/无车独立）")
        arrival_task_layout = QHBoxLayout(arrival_task_group)
        self.arrival_task_combo = QComboBox()
        self.arrival_task_combo.setEditable(True)
        arrival_task_layout.addWidget(QLabel("真实巡检任务"))
        arrival_task_layout.addWidget(self.arrival_task_combo, 1)
        self.override_arrival_task = QCheckBox("模拟到达时执行")
        self.override_arrival_task.setChecked(False)
        arrival_task_layout.addWidget(self.override_arrival_task)
        arrival_task_layout.addWidget(QLabel("不勾选时使用inspection.yaml的arrival_tasks；有车/无车不读取这里"))
        simulation_page_layout.addWidget(arrival_task_group)

        command_test_group = QGroupBox("模拟指令输入")
        command_test_layout = QHBoxLayout(command_test_group)
        self.test_command_input = QLineEdit()
        self.test_command_input.setPlaceholderText("输入：巡检 主干道往复任务 / 有车 / 无车 / 到达 起始点01 / 离开 点名")
        self.test_command_input.returnPressed.connect(self.execute_test_command)
        command_test_layout.addWidget(QLabel("测试指令"))
        command_test_layout.addWidget(self.test_command_input, 1)
        execute_test_button = QPushButton("发送")
        execute_test_button.clicked.connect(self.execute_test_command)
        command_test_layout.addWidget(execute_test_button)
        simulation_page_layout.addWidget(command_test_group)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        simulation_page_layout.addWidget(self.log, 2)

        video_group = QGroupBox("YOLO实时检测画面（truck=有车）")
        video_layout = QVBoxLayout(video_group)
        self.video_label = QLabel("等待 rtsp_truck.py 视频帧")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 260)
        self.video_label.setStyleSheet("background:#16191d; color:#d6d6d6;")
        video_layout.addWidget(self.video_label)
        real_page_layout.addWidget(video_group, 2)

        voice_group = QGroupBox("语音播报（真实上位机）")
        voice_layout = QGridLayout(voice_group)
        self.voice_combo = QComboBox()
        self.voice_name = QLineEdit()
        self.voice_name.setPlaceholderText("新语音名称，不含.wav也可")
        self.voice_text = QLineEdit()
        self.voice_text.setPlaceholderText("输入需要生成和播报的文本")
        self.voice_language = QComboBox()
        self.voice_language.addItems(["Chinese", "English"])
        self.voice_interval = QDoubleSpinBox()
        self.voice_interval.setRange(0.5, 300.0)
        self.voice_interval.setValue(5.0)
        self.voice_interval.setSuffix(" s")
        voice_layout.addWidget(QLabel("已有语音"), 0, 0)
        voice_layout.addWidget(self.voice_combo, 0, 1, 1, 3)
        play_voice_button = QPushButton("立即播放")
        play_voice_button.clicked.connect(self.play_voice)
        voice_layout.addWidget(play_voice_button, 0, 4)
        delete_voice_button = QPushButton("删除语音")
        delete_voice_button.clicked.connect(self.delete_voice)
        voice_layout.addWidget(delete_voice_button, 0, 5)
        loop_voice_button = QPushButton("循环播报任务")
        loop_voice_button.clicked.connect(lambda: self.voice_task(True))
        voice_layout.addWidget(loop_voice_button, 0, 6)
        stop_voice_button = QPushButton("停止播报任务")
        stop_voice_button.clicked.connect(lambda: self.voice_task(False))
        voice_layout.addWidget(stop_voice_button, 0, 7)
        voice_layout.addWidget(QLabel("循环间隔"), 0, 8)
        voice_layout.addWidget(self.voice_interval, 0, 9)
        voice_layout.addWidget(QLabel("文本生成"), 1, 0)
        voice_layout.addWidget(self.voice_text, 1, 1, 1, 3)
        voice_layout.addWidget(self.voice_name, 1, 4)
        voice_layout.addWidget(self.voice_language, 1, 5)
        create_voice_button = QPushButton("生成语音")
        create_voice_button.clicked.connect(self.create_voice)
        voice_layout.addWidget(create_voice_button, 1, 6)
        real_page_layout.addWidget(voice_group)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(300)
        self.robot_move_timer = QTimer(self)
        self.robot_move_timer.timeout.connect(self._repeat_robot_move)

    def send_control(self, action):
        data = {"task_name": self.task_combo.currentText().strip()} if action == "start" else {}
        message = make_message("inspection.control", source="inspection_ui", action=action, data=data)
        self.control_pub.publish(String(data=dumps(message)))

    def ptz_command(self, action, data=None, mode="manual"):
        message = make_message("ptz.command", source="inspection_ui", action=action, mode=mode, data=data or {})
        self.ptz_pub.publish(String(data=dumps(message)))

    def ptz_move(self, direction):
        self.ptz_command("move", {"direction": direction, "start": True, "speed": 5})

    def ptz_stop(self):
        self.ptz_command("stop")

    def ptz_preset(self, preset):
        self.ptz_command("goto_preset", {"preset": str(preset)})

    def robot_command(self, action, data=None):
        message = make_message("robot.command", source="inspection_ui", action=action, data=data or {})
        self.robot_pub.publish(String(data=dumps(message)))

    def play_voice(self):
        file_name = self.voice_combo.currentText().strip()
        if not file_name:
            QMessageBox.warning(self, "没有语音", "上位机语音列表为空。")
            return
        self.robot_command("voice.play", {"file_name": file_name})

    def create_voice(self):
        text = self.voice_text.text().strip()
        name = self.voice_name.text().strip()
        if not text or not name:
            QMessageBox.warning(self, "参数不完整", "语音名称和播报文本不能为空。")
            return
        self.robot_command(
            "voice.create",
            {"text": text, "name": name, "type": self.voice_language.currentText()},
        )

    def delete_voice(self):
        file_name = self.voice_combo.currentText().strip()
        if not file_name:
            QMessageBox.warning(self, "没有语音", "请选择需要删除的语音文件。")
            return
        answer = QMessageBox.question(
            self, "确认删除语音", "确认从上位机永久删除“{}”？".format(file_name),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.robot_command("voice.delete", {"file_name": file_name})

    def voice_task(self, start):
        file_name = self.voice_combo.currentText().strip()
        if not file_name:
            QMessageBox.warning(self, "没有语音", "请选择需要播报的语音文件。")
            return
        self.robot_command(
            "voice.task" if start else "voice.stop",
            {"file_name": file_name, "task_loop": bool(start), "task_time": float(self.voice_interval.value()), "task_cmd": "start" if start else "stop"},
        )

    def robot_move(self, direction):
        self.robot_move_direction = direction
        self._repeat_robot_move()
        self.robot_move_timer.start(250)
        self.robot_feedback_label.setText("车体：持续{}（点击停止结束）".format({
            "forward": "前进", "backward": "后退", "left": "左转", "right": "右转",
        }.get(direction, direction)))

    def _repeat_robot_move(self):
        direction = self.robot_move_direction
        if not direction:
            return
        linear = float(self.linear_speed.value())
        angular = float(self.angular_speed.value())
        values = {
            "forward": (linear, 0.0), "backward": (-linear, 0.0),
            "left": (0.0, angular), "right": (0.0, -angular),
        }
        speed_x, speed_z = values[direction]
        self.robot_command("move", {"speed_x": speed_x, "speed_z": speed_z})

    def robot_stop(self):
        self.robot_move_timer.stop()
        self.robot_move_direction = ""
        self.robot_command("stop")

    def closeEvent(self, event):
        # 关闭面板前明确停止车体和云台；底盘桥接节点另有0.8秒看门狗兜底。
        self.robot_stop()
        self.ptz_stop()
        event.accept()

    def switch_map(self):
        map_name = self.map_combo.currentText().strip()
        if not map_name:
            return
        answer = QMessageBox.question(
            self, "确认切换地图", "确认切换到地图“{}”？运行中的任务必须先停止。".format(map_name),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.robot_command("map.switch", {"map_name": map_name})

    def simulate_point(self, event):
        name = self.point_combo.currentText().strip()
        if not name:
            QMessageBox.warning(self, "无真实点位", "尚未从上位机读取到巡航点。")
            return
        with self.lock:
            known_names = self._point_names(self.catalog.get("points", []))
            catalog_source = self.catalog_source
        if catalog_source != "robot_bridge":
            QMessageBox.warning(self, "真实目录未就绪", "尚未收到真实上位机巡航点目录，不能发送测试到点信号。")
            return
        if name not in known_names:
            QMessageBox.warning(
                self, "点位不属于真实目录",
                "当前使用真实车体数据，模拟信号只能选择上位机实际返回的巡航点。\n"
                "请先刷新真实目录，再选择点位。",
            )
            return
        name_source = "robot_bridge" if name in known_names else "user_input"
        test_arrival_task_name = ""
        if event == "arrived" and self.override_arrival_task.isChecked():
            test_arrival_task_name = self.arrival_task_combo.currentText().strip()
            with self.lock:
                catalog = dict(self.catalog)
            real_task_names = [
                self._name(item) for item in (catalog.get("all_tasks") or catalog.get("tasks", []))
                if self._name(item)
            ]
            if not test_arrival_task_name or test_arrival_task_name not in real_task_names:
                QMessageBox.warning(self, "到点任务无效", "请选择上位机真实任务目录中的到点巡检任务。")
                return
        message = make_message(
            "point." + event, source="inspection_ui", event=event,
            point_seq=int(self.point_sequence.value()), point_name=name,
            distance=0.0, simulated_signal=True, factual_source=name_source,
            test_arrival_task_name=test_arrival_task_name,
        )
        self.point_pub.publish(String(data=dumps(message)))
        self._append_execution_log(
            "[模拟点位信号] {} #{} {}（真实点名；到点任务={}）".format(
                event, self.point_sequence.value(), name, test_arrival_task_name or "按YAML/不执行",
            )
        )

    def test_start_task(self):
        name = self.task_test_combo.currentText().strip()
        if not name:
            QMessageBox.warning(self, "任务名为空", "请输入需要测试的任务名。")
            return
        with self.lock:
            catalog = dict(self.catalog)
            catalog_source = self.catalog_source
        real_task_names = [
            self._name(item) for item in (catalog.get("all_tasks") or catalog.get("tasks", []))
            if self._name(item)
        ]
        if catalog_source != "robot_bridge":
            QMessageBox.warning(self, "真实目录未就绪", "尚未收到真实上位机任务目录，不能发送测试任务指令。")
            return
        if name not in real_task_names:
            QMessageBox.warning(
                self, "任务不属于真实目录",
                "当前使用真实车体数据，测试任务必须来自上位机实际任务列表。",
            )
            return
        self.test_task_command("start", {"task_name": name, "loop_time": 1})

    def start_simulation_flow(self):
        """通过编排器启动主任务，使分支任务完成后能够返回同一个主任务。"""
        name = self.task_test_combo.currentText().strip()
        if not name:
            QMessageBox.warning(self, "主任务名为空", "请选择或输入主干道往复任务。")
            return
        with self.lock:
            catalog = dict(self.catalog)
            catalog_source = self.catalog_source
        real_task_names = [
            self._name(item) for item in (catalog.get("all_tasks") or catalog.get("tasks", []))
            if self._name(item)
        ]
        if catalog_source != "robot_bridge":
            QMessageBox.warning(self, "真实目录未就绪", "尚未收到真实上位机任务目录，不能启动测试流程。")
            return
        if name not in real_task_names:
            QMessageBox.warning(
                self, "任务不属于真实目录",
                "当前使用真实车体数据，主任务必须来自上位机实际任务列表。",
            )
            return
        message = make_message(
            "inspection.control", source="inspection_ui", action="start",
            simulated_signal=True,
            data={"task_name": name, "loop_time": 1},
        )
        self.control_pub.publish(String(data=dumps(message)))
        self._append_execution_log("[手动启动指令] 真实主任务 {}，等待底盘HTTP回执".format(name))

    def test_task_command(self, action, data=None):
        message = make_message(
            "task.command", source="inspection_ui", action=action,
            command_id=new_id("ui-task"), data=data or {},
        )
        self.task_command_pub.publish(String(data=dumps(message)))
        self._append_execution_log("[任务测试指令] action={} data={}".format(action, data or {}))

    def execute_test_command(self):
        text = self.test_command_input.text().strip()
        if not text:
            return
        normalized = text.replace(" ", "").lower()
        if normalized in ("有车", "vehicle", "present"):
            self.simulate_detection("VEHICLE_PRESENT", 0.88)
        elif normalized in ("无车", "empty", "absent"):
            self.simulate_detection("VEHICLE_ABSENT", 0.0)
        elif normalized in ("异常", "error"):
            self.simulate_detection("DETECTION_ERROR", 0.0)
        else:
            parts = text.split(maxsplit=1)
            command = parts[0].lower()
            value = parts[1].strip() if len(parts) > 1 else ""
            if command in ("到达", "arrive", "arrived") and value:
                self.point_combo.setEditText(value)
                self.simulate_point("arrived")
            elif command in ("接近", "approach") and value:
                self.point_combo.setEditText(value)
                self.simulate_point("approach")
            elif command in ("离开", "leave") and value:
                self.point_combo.setEditText(value)
                self.simulate_point("leave")
            elif command in ("任务", "task", "start") and value:
                self.task_test_combo.setEditText(value)
                self.test_start_task()
            elif command in ("巡检", "inspection", "flow") and value:
                self.task_test_combo.setEditText(value)
                self.start_simulation_flow()
            elif command in ("停止", "stop"):
                self.test_task_command("stop")
            else:
                QMessageBox.warning(self, "无法识别", "支持：巡检 主任务名、有车、无车、到达 点名、接近 点名、离开 点名、任务 任务名、停止")
                return
        self._append_execution_log("[文本测试指令] {}".format(text))

    def simulate_detection(self, state, confidence):
        with self.lock:
            status = dict(self.status)
            catalog = dict(self.catalog)
            catalog_source = self.catalog_source
        # 车辆信号与云台动作独立，直接使用当前下拉框中的真实点位。
        point_name = self.point_combo.currentText().strip()
        if not point_name:
            QMessageBox.warning(self, "没有点位", "请先从真实上位机目录选择车辆信号所属点位。")
            return
        known_names = self._point_names(catalog.get("points", []))
        if catalog_source != "robot_bridge" or point_name not in known_names:
            QMessageBox.warning(self, "点位不属于真实目录", "车辆信号必须关联上位机真实巡航点。")
            return
        vehicle = True if state == "VEHICLE_PRESENT" else False if state == "VEHICLE_ABSENT" else None
        data = {
            "state": state, "vehicle_present": vehicle, "total_frames": 30,
            "detected_frames": 20 if vehicle else 0,
            "frame_ratio": 0.667 if vehicle else 0.0,
            "best_confidence": float(confidence), "simulated": True,
        }
        # 默认严格使用inspection.yaml的正式点位分支；只有勾选时才用UI临时覆盖。
        if self.override_branch_tasks.isChecked():
            if state == "VEHICLE_PRESENT":
                data["test_task_name"] = self.vehicle_task_combo.currentText().strip()
            elif state == "VEHICLE_ABSENT":
                data["test_task_name"] = self.empty_task_combo.currentText().strip()
            real_task_names = [
                self._name(item) for item in (catalog.get("all_tasks") or catalog.get("tasks", []))
                if self._name(item)
            ]
            if state in ("VEHICLE_PRESENT", "VEHICLE_ABSENT") and (
                not data.get("test_task_name") or data["test_task_name"] not in real_task_names
            ):
                QMessageBox.warning(self, "切换任务无效", "有车/无车临时任务必须来自上位机真实任务目录。")
                return
        message = make_message(
            "detection.result", source="inspection_ui", state=state,
            task_id=status.get("task_id", ""), point_seq=int(self.point_sequence.value()),
            point_name=point_name, data=data,
        )
        self.detection_pub.publish(String(data=dumps(message)))
        self._append_execution_log("[模拟检测] {} {}，等待真实任务切换回执".format(point_name, state))

    def _status_cb(self, raw):
        try:
            with self.lock:
                self.status = parse_message(raw)
        except Exception:
            pass

    def _health_cb(self, raw):
        try:
            message = parse_message(raw)
            with self.lock:
                self.health[message.get("source", "unknown")] = message.get("state", "unknown")
                self.health_detail[message.get("source", "unknown")] = message.get("data", {})
        except Exception:
            pass

    def _event_cb(self, raw):
        try:
            message = parse_message(raw)
            line = "{}  {}  {}".format(message.get("event", ""), message.get("point_name", ""), message.get("data", {}))
            with self.lock:
                self.logs.append(line)
                self.logs = self.logs[-200:]
                if message.get("event") == "ARRIVAL_TASK_REQUESTED":
                    decision = message.get("data", {}).get("decision", {})
                    requested = decision.get("task_name", "") if isinstance(decision, dict) else ""
                    self.task_transition.update({
                        "point": message.get("point_name", ""),
                        "detection": "到点巡检任务",
                        "requested": requested,
                        "status": "等待底盘真实回执" if requested else "未配置到点任务",
                    })
        except Exception:
            pass

    def _result_cb(self, raw):
        try:
            message = parse_message(raw)
            decision = message.get("data", {}).get("decision", {})
            requested = decision.get("task_name", "") if isinstance(decision, dict) else ""
            with self.lock:
                self.results.append(message)
                self.results = self.results[-100:]
                self.task_transition.update({
                    "point": message.get("point_name", ""), "detection": message.get("state", ""),
                    "requested": requested, "status": "等待底盘真实回执" if requested else "无需切换任务",
                })
        except Exception:
            pass

    def _catalog_cb(self, raw):
        try:
            message = parse_message(raw)
            with self.lock:
                self.catalog = message.get("data", {})
                self.catalog_source = str(message.get("source", ""))
        except Exception:
            pass

    def _append_execution_log(self, line):
        with self.lock:
            self.logs.append(line)
            self.logs = self.logs[-200:]

    def _ptz_result_cb(self, raw):
        try:
            message = parse_message(raw)
            with self.lock:
                self.ptz_feedback = message
            status = message.get("status", "")
            data = message.get("data", {})
            if status in ("success", "failed", "rejected", "cancelled"):
                self._append_execution_log(
                    "[云台实际回执] action={} status={} state={} preset={} position={} error={}".format(
                        message.get("action", ""), status, data.get("device_state", ""),
                        data.get("preset", ""), data.get("position"), data.get("error", ""),
                    )
                )
        except Exception:
            pass

    def _task_status_cb(self, raw):
        try:
            message = parse_message(raw)
            data = message.get("data", {})
            with self.lock:
                self.task_feedback = data
                actual = str(data.get("main_task_name") or data.get("task_name") or "")
                if actual:
                    self.task_transition["actual"] = actual
                    requested = self.task_transition.get("requested", "")
                    if requested:
                        self.task_transition["status"] = "切换成功" if actual == requested else "底盘当前任务待确认"
            if message.get("command_status"):
                self._append_execution_log(
                    "[底盘实际回执] command={} status={} data={}".format(
                        message.get("command_id", ""), message.get("command_status"), data,
                    )
                )
        except Exception:
            pass

    def _detection_result_cb(self, raw):
        try:
            message = parse_message(raw)
            with self.lock:
                self.detection_feedback = message
            self._append_execution_log(
                "[检测最终输出] point={} state={} frames={}/{} confidence={}".format(
                    message.get("point_name", ""), message.get("state", ""),
                    message.get("data", {}).get("detected_frames", 0),
                    message.get("data", {}).get("total_frames", 0),
                    message.get("data", {}).get("best_confidence", 0),
                )
            )
        except Exception:
            pass

    def _detection_frame_cb(self, raw):
        try:
            message = parse_message(raw)
            data = message.get("data", {})
            preview = str(data.get("preview_jpeg_b64", ""))
            if not preview:
                return
            with self.lock:
                self.preview_b64 = preview
                self.preview_frame_id = int(data.get("frame_id", self.preview_frame_id + 1))
        except Exception:
            pass

    def _robot_result_cb(self, raw):
        try:
            message = parse_message(raw)
            with self.lock:
                self.robot_feedback = message
            self._append_execution_log(
                "[车体实际HTTP回执] action={} status={} data={}".format(
                    message.get("action", ""), message.get("status", ""), message.get("data", {}),
                )
            )
        except Exception:
            pass

    @staticmethod
    def _name(item):
        if isinstance(item, dict):
            for key in ("name", "task_name", "main_task_name", "point_name", "track_name", "map_name", "file_name"):
                value = item.get(key)
                if value not in (None, ""):
                    return str(value)
            for value in item.values():
                nested = InspectionPanel._name(value)
                if nested:
                    return nested
            return ""
        if isinstance(item, list):
            return InspectionPanel._name(item[0]) if item else ""
        return "" if item is None else str(item)

    @staticmethod
    def _point_names(node):
        names = []
        def walk(value):
            if isinstance(value, dict):
                name = value.get("name")
                if isinstance(name, str) and name.strip():
                    names.append(name.strip())
                for child in value.values():
                    walk(child)
            elif isinstance(value, list):
                for child in value:
                    walk(child)
        walk(node)
        return list(dict.fromkeys(names))

    def refresh_catalog(self, catalog):
        current_map = self._name(catalog.get("current_map", {}))
        map_names = [self._name(item) for item in catalog.get("maps", [])]
        map_names = [name for name in map_names if name]
        if current_map and current_map not in map_names:
            map_names.insert(0, current_map)
        previous_map = self.map_combo.currentText()
        map_items_changed = [self.map_combo.itemText(i) for i in range(self.map_combo.count())] != map_names
        if map_items_changed:
            self.map_combo.clear()
            self.map_combo.addItems(map_names)
            selected_map = previous_map if previous_map in map_names else current_map
            index = self.map_combo.findText(selected_map)
            if index >= 0:
                self.map_combo.setCurrentIndex(index)
        self.current_map_label.setText("实际：{}".format(current_map or "未知"))

        point_names = self._point_names(catalog.get("points", []))
        previous_point = self.point_combo.currentText()
        if [self.point_combo.itemText(i) for i in range(self.point_combo.count())] != point_names:
            self.point_combo.clear()
            self.point_combo.addItems(point_names)
            point_index = self.point_combo.findText(previous_point)
            if point_index >= 0:
                self.point_combo.setCurrentIndex(point_index)
            elif previous_point:
                self.point_combo.setEditText(previous_point)

        tasks = catalog.get("tasks", [])
        task_names = [self._name(task) for task in tasks if self._name(task)]
        previous_task = self.task_combo.currentText()
        if [self.task_combo.itemText(i) for i in range(self.task_combo.count())] != task_names:
            self.task_combo.blockSignals(True)
            self.task_combo.clear()
            self.task_combo.addItems(task_names)
            index = self.task_combo.findText(previous_task)
            self.task_combo.setCurrentIndex(index if index >= 0 else 0)
            self.task_combo.blockSignals(False)

        all_task_names = list(task_names)
        previous_test_task = self.task_test_combo.currentText()
        if [self.task_test_combo.itemText(i) for i in range(self.task_test_combo.count())] != all_task_names:
            previous_vehicle_task = self.vehicle_task_combo.currentText()
            previous_empty_task = self.empty_task_combo.currentText()
            previous_arrival_task = self.arrival_task_combo.currentText()
            self.task_test_combo.clear()
            self.task_test_combo.addItems(all_task_names)
            self.task_test_combo.setEditText(previous_test_task or (all_task_names[0] if all_task_names else ""))
            self.vehicle_task_combo.clear()
            self.vehicle_task_combo.addItems(all_task_names)
            self.vehicle_task_combo.setEditText(previous_vehicle_task)
            self.empty_task_combo.clear()
            self.empty_task_combo.addItems(all_task_names)
            self.empty_task_combo.setEditText(previous_empty_task)
            self.arrival_task_combo.clear()
            self.arrival_task_combo.addItems(all_task_names)
            self.arrival_task_combo.setEditText(previous_arrival_task)

        voice_names = [self._name(item) for item in catalog.get("voices", []) if self._name(item)]
        previous_voice = self.voice_combo.currentText()
        if [self.voice_combo.itemText(i) for i in range(self.voice_combo.count())] != voice_names:
            self.voice_combo.clear()
            self.voice_combo.addItems(voice_names)
            voice_index = self.voice_combo.findText(previous_voice)
            if voice_index >= 0:
                self.voice_combo.setCurrentIndex(voice_index)

    @staticmethod
    def _named_records(node):
        records = []
        def walk(value):
            if isinstance(value, dict):
                if isinstance(value.get("name"), str) and value.get("name").strip():
                    records.append(value)
                for child in value.values():
                    walk(child)
            elif isinstance(value, list):
                for child in value:
                    walk(child)
        walk(node)
        unique = []
        seen = set()
        for record in records:
            # 轨道的坐标、地图字段可能是嵌套dict/list，序列化后再去重，避免unhashable错误。
            key = json.dumps(record, ensure_ascii=False, sort_keys=True, default=str)
            if key not in seen:
                seen.add(key)
                unique.append(record)
        return unique

    def refresh_real_data(self, catalog):
        points = self._named_records(catalog.get("points", []))
        tracks = self._named_records(catalog.get("tracks", []))
        rows = []
        for item in points:
            rows.append(["巡航点", item.get("name", ""), item.get("map", item.get("x", "")), item.get("y", ""), item.get("type", "")])
        for item in tracks:
            rows.append(["轨道", item.get("name", ""), item.get("map", ""), "", item.get("type", item.get("path_mode", ""))])
        self.real_catalog_table.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for column, value in enumerate(values):
                self.real_catalog_table.setItem(row, column, QTableWidgetItem(str(value)))

        report = catalog.get("task_report", [])
        if isinstance(report, dict):
            report = [report]
        report = [item for item in report if isinstance(item, dict)]
        self.report_table.setRowCount(len(report))
        for row, item in enumerate(report):
            values = [
                item.get("main_task_name", ""), item.get("task_name", ""), item.get("task_type", ""),
                item.get("task_state", ""), item.get("map", ""), item.get("remai_distance", ""),
                item.get("task_time", ""), item.get("error_info", ""),
            ]
            for column, value in enumerate(values):
                self.report_table.setItem(row, column, QTableWidgetItem(str(value)))

    def refresh_route(self):
        with self.lock:
            catalog = dict(self.catalog)
        selected = self.task_combo.currentText().strip()
        task = next((item for item in catalog.get("tasks", []) if self._name(item) == selected), {})
        moves = [
            item for item in task.get("tasks", [])
            if isinstance(item, dict) and item.get("task_type") == "TASK_MOVE_TO"
        ] if isinstance(task, dict) else []
        self.route_table.setRowCount(len(moves))
        for row, move in enumerate(moves):
            values = [
                row + 1, move.get("task_name", move.get("name", "")),
                move.get("path_name", ""), move.get("map", ""),
                move.get("speed", ""), move.get("task_type", ""),
            ]
            for column, value in enumerate(values):
                self.route_table.setItem(row, column, QTableWidgetItem(str(value)))

    def refresh(self):
        with self.lock:
            status, health, results, logs, catalog = dict(self.status), dict(self.health), list(self.results), list(self.logs), dict(self.catalog)
            health_detail = dict(self.health_detail)
            catalog_source = self.catalog_source
            ptz_feedback = dict(self.ptz_feedback)
            task_feedback = dict(self.task_feedback)
            detection_feedback = dict(self.detection_feedback)
            robot_feedback = dict(self.robot_feedback)
            task_transition = dict(self.task_transition)
            preview_b64 = self.preview_b64
            preview_frame_id = self.preview_frame_id
        self.refresh_catalog(catalog)
        self.refresh_real_data(catalog)
        self.refresh_route()
        if catalog_source == "robot_bridge":
            self.data_source.setText("事实数据源：真实上位机 HTTP（地图 / 任务 / 轨道 / 巡航点）")
            self.data_source.setStyleSheet("color: #16803a; font-weight: bold;")
        elif catalog_source:
            self.data_source.setText("事实数据源：{}（模拟目录）".format(catalog_source))
            self.data_source.setStyleSheet("color: #a56600; font-weight: bold;")
        else:
            self.data_source.setText("事实数据源：尚未收到目录，请检查上位机网络")
            self.data_source.setStyleSheet("color: #b00020; font-weight: bold;")
        self.summary.setText(
            "任务：{}  状态：{}  当前点：{}  节点：{}".format(
                status.get("task_name", ""), status.get("state", ""),
                status.get("point_name", ""), "  ".join("{}={}".format(k, v) for k, v in health.items()),
            )
        )
        robot_detail = health_detail.get("robot_bridge", {})
        pose = robot_detail.get("pose", {}) if isinstance(robot_detail, dict) else {}
        sensors = robot_detail.get("sensors", {}) if isinstance(robot_detail, dict) else {}
        localization = sensors.get("localization", task_feedback.get("local_state", "等待"))
        self.vehicle_state_label.setText(
            "车体状态：定位={}  雷达={}  IMU={}  电量={}  避障={}  急停={}  位姿=({},{},{})".format(
                localization, sensors.get("radar", "等待"), sensors.get("imu", "等待"),
                sensors.get("battery", "等待"), sensors.get("obstacle", task_feedback.get("obstacle", "等待")),
                sensors.get("emergency_stop", "等待"), pose.get("x", "-"), pose.get("y", "-"), pose.get("yaw", "-"),
            )
        )
        if preview_b64 and preview_frame_id != self.displayed_preview_frame_id:
            try:
                pixmap = QPixmap()
                if pixmap.loadFromData(base64.b64decode(preview_b64)):
                    self.video_label.setPixmap(
                        pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    )
                    self.displayed_preview_frame_id = preview_frame_id
            except Exception as exc:
                self.video_label.setText("YOLO画面解码失败: {}".format(exc))
        ptz_data = ptz_feedback.get("data", {})
        self.ptz_feedback_label.setText(
            "云台：{} / {}  预置点={}  位置={}".format(
                ptz_feedback.get("status", "等待"), ptz_data.get("device_state", ""),
                ptz_data.get("preset", ""), ptz_data.get("position"),
            )
        )
        self.task_feedback_label.setText(
            "底盘：{}  当前={}  剩余={}m".format(
                task_feedback.get("task_state", "等待"), task_feedback.get("task_name", ""),
                task_feedback.get("remai_distance", 0),
            )
        )
        detection_data = detection_feedback.get("data", {})
        self.detection_feedback_label.setText(
            "检测：{}  点位={}  置信度={}".format(
                detection_feedback.get("state", "等待"), detection_feedback.get("point_name", ""),
                detection_data.get("best_confidence", 0),
            )
        )
        self.robot_feedback_label.setText(
            "车体：{} / {}".format(robot_feedback.get("action", "等待"), robot_feedback.get("status", "等待"))
        )
        self.task_transition_label.setText(
            "任务切换：点位={}  检测={}  请求={}  实际={}  状态={}".format(
                task_transition.get("point", ""), task_transition.get("detection", ""),
                task_transition.get("requested", ""), task_transition.get("actual", ""),
                task_transition.get("status", ""),
            )
        )
        self.table.setRowCount(len(results))
        for row, result in enumerate(results):
            detection = result.get("data", {}).get("detection", {})
            decision = result.get("data", {}).get("decision", {})
            values = [
                result.get("point_seq", 0), result.get("point_name", ""), result.get("state", ""),
                detection.get("best_confidence", 0), result.get("data", {}).get("rule", {}).get("sweep", False),
                decision.get("task_name", decision.get("action", "")) if isinstance(decision, dict) else decision,
                result.get("task_id", ""),
            ]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(str(value)))
        self.log.setPlainText("\n".join(logs))
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())


if __name__ == "__main__":
    rospy.init_node("inspection_panel", disable_signals=True)
    app = QApplication(sys.argv)
    panel = InspectionPanel()
    panel.show()
    code = app.exec_()
    rospy.signal_shutdown("UI closed")
    sys.exit(code)
