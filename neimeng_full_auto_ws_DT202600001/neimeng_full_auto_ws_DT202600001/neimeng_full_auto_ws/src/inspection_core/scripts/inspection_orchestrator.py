#!/usr/bin/env python3
import copy
import threading
import time

import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, new_id, parse_message


class InspectionOrchestrator:
    def __init__(self):
        self.robot_code = str(rospy.get_param("~robot_code", "DT000000000"))
        self.point_rules = rospy.get_param("~point_rules", [])
        self.default_rule = rospy.get_param("~default_rule", {})
        self.points = rospy.get_param("~points", {})
        # 三套规则彼此独立：点位云台动作、到点任务、车辆信号任务决策。
        self.arrival_tasks = rospy.get_param("~arrival_tasks", {})
        self.vehicle_task_rules = rospy.get_param("~vehicle_task_rules", {})
        self.detection_points = rospy.get_param("~detection_points", {})
        self.detection_collection_enabled = bool(rospy.get_param("~detection_collection_enabled", True))
        self.vehicle_default_rule = rospy.get_param("~vehicle_default_rule", {
            "on_vehicle": {"action": "none"},
            "on_empty": {"action": "none"},
            "on_error": {"action": "none"},
        })
        self.sweep_profiles = rospy.get_param("~sweep_profiles", {})
        self.detection_config = rospy.get_param("~detection", {})
        self.voice_events = rospy.get_param("~voice_events", {})
        self.pause_on_arrival = bool(rospy.get_param("~pause_on_arrival", True))
        self.branch_restart_delay = max(0.1, float(rospy.get_param("~branch/restart_delay", 0.8)))

        self.control_pub = rospy.Publisher(Topics.STATUS, String, queue_size=20, latch=True)
        self.event_pub = rospy.Publisher(Topics.EVENT, String, queue_size=50)
        self.result_pub = rospy.Publisher(Topics.RESULT, String, queue_size=20)
        self.ptz_pub = rospy.Publisher(Topics.PTZ_COMMAND, String, queue_size=20)
        self.detection_pub = rospy.Publisher(Topics.DETECTION_CONTROL, String, queue_size=20)
        self.task_pub = rospy.Publisher(Topics.TASK_COMMAND, String, queue_size=20)
        self.robot_pub = rospy.Publisher(Topics.ROBOT_COMMAND, String, queue_size=20)

        rospy.Subscriber(Topics.CONTROL, String, self._control, queue_size=20)
        rospy.Subscriber(Topics.POINT_EVENT, String, self._point_event, queue_size=30)
        rospy.Subscriber(Topics.PTZ_RESULT, String, self._ptz_result, queue_size=30)
        rospy.Subscriber(Topics.DETECTION_RESULT, String, self._detection_result, queue_size=20)
        rospy.Subscriber(Topics.TASK_STATUS, String, self._task_status, queue_size=20)
        rospy.Subscriber(Topics.ROBOT_CATALOG, String, self._catalog, queue_size=5)

        self.lock = threading.RLock()
        self.task_id = ""
        self.task_name = ""
        self.state = "IDLE"
        self.active_point = None
        self.active_rule = None
        self.last_task_status = {}
        self.pending_ptz = {}
        self.detection_started_at = None
        self.detection_context = None
        self.stop_timer = None
        self.branch_restart_timer = None
        self.branch_task_name = ""
        self.branch_return_to_main = False
        # 底盘暂停状态和当前点云台到位状态必须分开记录。
        self.point_ptz_ready = False
        self.prepared_points = set()
        self.catalog_ready = False
        self.catalog_is_real = False
        self.available_tasks = set()
        self.available_points = set()
        self._publish_status()

    def _catalog(self, raw):
        try:
            message = parse_message(raw)
            catalog = message.get("data", {})
            tasks = catalog.get("all_tasks") or catalog.get("tasks", [])
            names = {
                str(item.get("name") or item.get("task_name") or "").strip()
                for item in tasks
                if isinstance(item, dict) and str(item.get("name") or item.get("task_name") or "").strip()
            }
            point_names = set()

            def collect_point_names(node):
                if isinstance(node, dict):
                    value = node.get("name") or node.get("point_name")
                    if isinstance(value, str) and value.strip():
                        point_names.add(value.strip())
                    for child in node.values():
                        collect_point_names(child)
                elif isinstance(node, list):
                    for child in node:
                        collect_point_names(child)

            collect_point_names(catalog.get("points", []))
            with self.lock:
                self.available_tasks = names
                self.available_points = point_names
                self.catalog_ready = True
                self.catalog_is_real = message.get("source") == "robot_bridge"
        except Exception:
            pass

    def _resolve_rule(self, point_name):
        result = copy.deepcopy(self.default_rule)
        # 关键词采用包含匹配，不要求完整点名一致；忽略空格和英文大小写。
        # 例如关键词“左”可匹配“台位1左”，关键词“起始点”可匹配“起始点02”。
        normalized_name = str(point_name).replace(" ", "").lower()
        for rule in self.point_rules:
            keywords = [str(value).replace(" ", "").lower() for value in rule.get("keywords", [])]
            if any(keyword and keyword in normalized_name for keyword in keywords):
                result.update(copy.deepcopy(rule))
                break
        exact = self.points.get(point_name, {}) if isinstance(self.points, dict) else {}
        result.update(copy.deepcopy(exact))
        return result

    def _sequence(self, point_name, supplied=0):
        exact = self.points.get(point_name, {}) if isinstance(self.points, dict) else {}
        return int(exact.get("sequence", supplied or 0))

    def _resolve_vehicle_rule(self, point_name):
        result = copy.deepcopy(self.vehicle_default_rule)
        exact = self.vehicle_task_rules.get(point_name, {}) if isinstance(self.vehicle_task_rules, dict) else {}
        result.update(copy.deepcopy(exact))
        return result

    def _arrival_task_decision(self, point_name, message):
        configured = self.arrival_tasks.get(point_name, {}) if isinstance(self.arrival_tasks, dict) else {}
        configured = copy.deepcopy(configured) if isinstance(configured, dict) else {}
        test_name = str(message.get("test_arrival_task_name", "")).strip()
        if bool(message.get("simulated_signal", False)) and test_name:
            return {
                "action": "start_task", "task_name": test_name,
                "stop_before_start": True, "return_to_main_task": True,
                "source": "ui_test_override",
            }
        if not bool(configured.get("enabled", False)):
            return None
        task_name = str(configured.get("task_name", "")).strip()
        if not task_name:
            return None
        return {
            "action": "start_task", "task_name": task_name,
            "stop_before_start": bool(configured.get("stop_before_start", True)),
            "return_to_main_task": bool(configured.get("return_to_main_task", True)),
            "loop_time": int(configured.get("loop_time", 1)),
            "source": "arrival_tasks",
        }

    def _publish_status(self, error=""):
        point = self.active_point or {}
        message = make_message(
            "inspection.status", source="inspection_orchestrator",
            robot_code=self.robot_code, task_id=self.task_id, task_name=self.task_name,
            state=self.state, point_seq=point.get("point_seq", 0),
            point_name=point.get("point_name", ""),
            data={
                "rule": self.active_rule or {}, "error": error,
                "robot_task": self.last_task_status,
                "ptz_ready": self.point_ptz_ready,
            },
        )
        self.control_pub.publish(String(data=dumps(message)))

    def _event(self, event, data=None):
        point = self.active_point or {}
        message = make_message(
            "inspection.event", source="inspection_orchestrator", event=event,
            robot_code=self.robot_code, task_id=self.task_id, task_name=self.task_name,
            point_seq=point.get("point_seq", 0), point_name=point.get("point_name", ""),
            data=data or {},
        )
        self.event_pub.publish(String(data=dumps(message)))

    def _send_task(self, action, data=None, command_id=None):
        message = make_message(
            "task.command", source="inspection_orchestrator", action=action,
            command_id=command_id or new_id("task"), task_id=self.task_id,
            data=data or {},
        )
        self.task_pub.publish(String(data=dumps(message)))

    def _send_ptz(self, action, data, phase):
        point = self.active_point or {}
        command_id = new_id("ptz")
        self.pending_ptz[command_id] = phase
        message = make_message(
            "ptz.command", source="inspection_orchestrator", action=action, mode="auto",
            command_id=command_id, task_id=self.task_id,
            point_seq=point.get("point_seq", 0), point_name=point.get("point_name", ""), data=data,
        )
        self.ptz_pub.publish(String(data=dumps(message)))

    def _play_voice(self, file_name, event_name):
        file_name = str(file_name or "").strip()
        if not file_name:
            return
        message = make_message(
            "robot.command", source="inspection_orchestrator", action="voice.play",
            command_id=new_id("voice"), task_id=self.task_id,
            data={"file_name": file_name, "event": event_name},
        )
        self.robot_pub.publish(String(data=dumps(message)))

    def _control(self, raw):
        try:
            message = parse_message(raw)
            action = str(message.get("action", message.get("method", ""))).lower()
            data = message.get("data", {})
            with self.lock:
                if action in ("start", "inspection.start"):
                    requested_task_name = str(data.get("task_name", data.get("name", "巡检任务")))
                    if bool(message.get("simulated_signal", False)) and not (self.catalog_ready and self.catalog_is_real):
                        raise ValueError("real robot catalog is not ready")
                    if self.catalog_ready and self.catalog_is_real and requested_task_name not in self.available_tasks:
                        raise ValueError("main task is not in current real catalog: " + requested_task_name)
                    self.task_id = str(data.get("task_id") or new_id("inspection"))
                    self.task_name = requested_task_name
                    self.state = "STARTING"
                    self.active_point = None
                    self.active_rule = None
                    self.point_ptz_ready = False
                    self.branch_task_name = ""
                    self.branch_return_to_main = False
                    self.prepared_points.clear()
                    self._send_task("start", {"task_name": self.task_name, "loop_time": data.get("loop_time", 1)}, message.get("tid"))
                    self._event("TASK_STARTED")
                    self._play_voice(self.voice_events.get("task_started", ""), "TASK_STARTED")
                elif action in ("pause", "inspection.pause"):
                    self._send_task("pause", command_id=message.get("tid"))
                    self.state = "PAUSED"
                elif action in ("resume", "inspection.resume"):
                    self._send_task("resume", command_id=message.get("tid"))
                    self.state = "MOVING"
                elif action in ("stop", "inspection.stop"):
                    self._cancel_active()
                    self._send_task("stop", command_id=message.get("tid"))
                    self.branch_task_name = ""
                    self.branch_return_to_main = False
                    self.state = "IDLE"
                    self._event("TASK_STOPPED")
                elif action in ("retry_point", "inspection.retry_point"):
                    if not self.active_point:
                        raise ValueError("no active point")
                    self._begin_point()
                else:
                    raise ValueError("unsupported inspection action: " + action)
                self._publish_status()
        except Exception as exc:
            rospy.logerr("巡检控制失败: %s", exc)
            self.state = "ERROR"
            self._publish_status(str(exc))

    def _point_event(self, raw):
        try:
            message = parse_message(raw)
            event = str(message.get("event", "")).lower()
            point_name = str(message.get("point_name", "")).strip()
            if not point_name:
                return
            with self.lock:
                if bool(message.get("simulated_signal", False)) and not (self.catalog_ready and self.catalog_is_real):
                    rospy.logwarn("真实上位机目录尚未就绪，忽略手动点位信号: %s", point_name)
                    return
                if self.catalog_ready and self.catalog_is_real and point_name not in self.available_points:
                    self._event("POINT_EVENT_REJECTED", {
                        "point_name": point_name,
                        "reason": "point is not in current real catalog",
                        "simulated_signal": bool(message.get("simulated_signal", False)),
                    })
                    rospy.logwarn("忽略不属于真实上位机目录的点位: %s", point_name)
                    return
                rule = self._resolve_rule(point_name)
                seq = self._sequence(point_name, message.get("point_seq", 0))
                if event == "approach":
                    if point_name in self.prepared_points:
                        return
                    self.prepared_points.add(point_name)
                    self.active_point = {"point_name": point_name, "point_seq": seq}
                    self.active_rule = rule
                    self.point_ptz_ready = False
                    self.state = "APPROACHING"
                    # 接近事件只记录，不重复控制云台；固定点动作统一在arrived事件执行。
                    self._event("POINT_APPROACHING", {"distance": message.get("distance")})
                elif event == "arrived":
                    self.active_point = {"point_name": point_name, "point_seq": seq}
                    self.active_rule = rule
                    self._begin_point()
                    self._start_point_detection(point_name, seq)
                    arrival_decision = self._arrival_task_decision(point_name, message)
                    if arrival_decision:
                        self._event("ARRIVAL_TASK_REQUESTED", {
                            "point_name": point_name,
                            "decision": arrival_decision,
                        })
                        self._execute_decision(arrival_decision)
                elif event == "leave":
                    self.point_ptz_ready = False
                    self._event("POINT_LEFT")
                    if self.state not in ("ERROR", "PAUSED", "IDLE"):
                        self.state = "MOVING"
                self._publish_status()
        except Exception as exc:
            rospy.logerr("点位事件处理失败: %s", exc)

    def _begin_point(self):
        self._cancel_timer()
        self.detection_started_at = None
        self.point_ptz_ready = False
        self.state = "PTZ_POSITIONING"
        self._event("POINT_ARRIVED")
        rule = self.active_rule or {}
        self._play_voice(rule.get("voice_on_arrival", self.voice_events.get("point_arrived", "")), "POINT_ARRIVED")
        self._send_ptz("mode", {"mode": "auto"}, "arrived_mode")

    def _ptz_result(self, raw):
        try:
            message = parse_message(raw)
            command_id = message.get("command_id", "")
            if message.get("status") != "success":
                if message.get("status") in ("failed", "rejected", "cancelled"):
                    self.pending_ptz.pop(command_id, None)
                if message.get("status") == "failed":
                    self.point_ptz_ready = False
                    self.state = "ERROR"
                    self._event("PTZ_ERROR", message.get("data", {}))
                    self._publish_status(message.get("data", {}).get("error", "PTZ failed"))
                return
            with self.lock:
                phase = self.pending_ptz.pop(command_id, "")
                if phase == "approach_mode":
                    rule = self.active_rule or {}
                    self._send_ptz(
                        "goto_preset",
                        {"preset": str(rule.get("preset", "4")), "speed": rule.get("speed", 1.0), "settle_seconds": rule.get("approach_settle_seconds", 1.0)},
                        "approach",
                    )
                elif phase == "approach":
                    self._event("PTZ_APPROACH_READY")
                    return
                elif phase == "arrived_mode":
                    rule = self.active_rule or {}
                    self._send_ptz(
                        "home" if rule.get("home", False) else "goto_preset",
                        {"preset": str(rule.get("preset", "1")), "speed": rule.get("speed", 1.0), "settle_seconds": rule.get("settle_seconds", 2.0)},
                        "arrived_preset",
                    )
                    # 巡航先入保护队列；预置点完成后自动执行，新预置点会取消/抢占旧巡航。
                    if bool(rule.get("sweep", False)):
                        self._start_sweep("ptz_only_sweep")
                elif phase == "arrived_preset":
                    # 该success只表示真实ONVIF云台动作完成，与底盘任务和车辆信号无关。
                    self.point_ptz_ready = True
                    self._event("PTZ_READY")
                    if not bool((self.active_rule or {}).get("sweep", False)):
                        self._finish_ptz_action()
                elif phase == "ptz_only_sweep":
                    self._event("PTZ_SWEEP_FINISHED")
                    self._finish_ptz_action()
        except Exception as exc:
            rospy.logerr("PTZ结果处理失败: %s", exc)

    def _start_point_detection(self, point_name, point_seq):
        """可选的YOLO采集链路，与云台和到点任务并行、互不等待。"""
        if not self.detection_collection_enabled:
            return
        config = self.detection_points.get(point_name, {}) if isinstance(self.detection_points, dict) else {}
        if not isinstance(config, dict) or not bool(config.get("enabled", False)):
            return
        self._cancel_timer()
        self.detection_started_at = time.monotonic()
        self.detection_context = {"point_name": point_name, "point_seq": int(point_seq)}
        message = make_message(
            "detection.control", source="inspection_orchestrator", action="start",
            task_id=self.task_id, point_seq=int(point_seq), point_name=point_name, data={},
        )
        self.detection_pub.publish(String(data=dumps(message)))
        self._event("DETECTION_STARTED", {"point_name": point_name, "independent": True})
        seconds = float(config.get("collect_seconds", self.detection_config.get("collect_seconds", 6.0)))
        self.stop_timer = rospy.Timer(rospy.Duration(max(0.05, seconds)), self._stop_detection, oneshot=True)

    def _start_sweep(self, phase):
        profile_name = str((self.active_rule or {}).get("sweep_profile", "normal"))
        profile = self.sweep_profiles.get(profile_name, {})
        self.state = "SCANNING"
        self._send_ptz("sweep", {"steps": profile.get("steps", []), "repeat": profile.get("repeat", 1)}, phase)
        self._publish_status()

    def _stop_detection(self, _event=None):
        with self.lock:
            point = self.detection_context or {}
            message = make_message(
                "detection.control", source="inspection_orchestrator", action="stop",
                task_id=self.task_id, point_seq=point.get("point_seq", 0),
                point_name=point.get("point_name", ""), data={},
            )
            self.detection_pub.publish(String(data=dumps(message)))
            self._event("DETECTION_COLLECTION_FINISHED", {"point_name": point.get("point_name", "")})
            self.stop_timer = None
            self.detection_context = None

    def _detection_result(self, raw):
        try:
            message = parse_message(raw)
            with self.lock:
                point_name = str(message.get("point_name", "")).strip()
                if not point_name:
                    rospy.logwarn("忽略没有真实点名的车辆信号")
                    return
                if self.task_id and message.get("task_id") and message.get("task_id") != self.task_id:
                    rospy.logwarn("忽略旧任务检测结果: %s", message.get("task_id"))
                    return
                if self.catalog_ready and self.catalog_is_real and point_name not in self.available_points:
                    rospy.logwarn("忽略不属于真实目录的车辆信号点位: %s", point_name)
                    return
                point_seq = self._sequence(point_name, message.get("point_seq", 0))
                decision_rule = self._resolve_vehicle_rule(point_name)
                state = str(message.get("state", message.get("data", {}).get("state", "DETECTION_ERROR")))
                key = "on_vehicle" if state == "VEHICLE_PRESENT" else "on_empty" if state == "VEHICLE_ABSENT" else "on_error"
                action = copy.deepcopy(decision_rule.get(key, {"action": "none"}))
                detection_data = message.get("data", {})
                # 人工车辆信号可临时指定真实任务；否则使用独立vehicle_task_rules。
                test_task_name = str(detection_data.get("test_task_name", "")).strip()
                if bool(detection_data.get("simulated", False)) and test_task_name and state in ("VEHICLE_PRESENT", "VEHICLE_ABSENT"):
                    action = {
                        "action": "start_task", "task_name": test_task_name,
                        "stop_before_start": True, "return_to_main_task": True,
                        "test_override": True,
                    }
                voice_key = "voice_on_vehicle" if state == "VEHICLE_PRESENT" else "voice_on_empty" if state == "VEHICLE_ABSENT" else "voice_on_error"
                global_voice_key = "vehicle_present" if state == "VEHICLE_PRESENT" else "vehicle_absent" if state == "VEHICLE_ABSENT" else "detection_error"
                self._play_voice(decision_rule.get(voice_key, self.voice_events.get(global_voice_key, "")), state)
                result = make_message(
                    "inspection.result", source="inspection_orchestrator", robot_code=self.robot_code,
                    task_id=self.task_id, task_name=self.task_name,
                    point_seq=point_seq, point_name=point_name,
                    state=state, data={"detection": detection_data, "decision": action, "rule": decision_rule},
                )
                self.result_pub.publish(String(data=dumps(result)))
                self._event(state, {"point_name": point_name, "point_seq": point_seq, "decision": action})
                self.state = "DISPATCHING"
                self._execute_decision(action)
                self._publish_status()
        except Exception as exc:
            rospy.logerr("检测结果决策失败: %s", exc)
            self.state = "ERROR"
            self._publish_status(str(exc))

    def _execute_decision(self, decision):
        if isinstance(decision, str):
            decision = {"action": decision}
        action = str(decision.get("action", "continue")).lower()
        if action in ("continue", "resume"):
            self._send_task("resume")
            self.state = "MOVING"
        elif action == "start_task":
            name = str(decision.get("task_name", ""))
            if not name:
                raise ValueError("decision task_name is empty")
            if self.catalog_ready and self.catalog_is_real and name not in self.available_tasks:
                self.state = "ERROR"
                self._event("TASK_DECISION_REJECTED", {"task_name": name, "reason": "task is not in current real catalog"})
                raise ValueError("decision task is not in current real catalog: " + name)
            self.branch_task_name = name
            self.branch_return_to_main = bool(decision.get("return_to_main_task", False))
            self._event("BRANCH_TASK_REQUESTED", {
                "branch_task": name,
                "return_to_main_task": self.branch_return_to_main,
                "main_task": self.task_name,
            })
            if decision.get("stop_before_start", False):
                self._send_task("stop")
                threading.Timer(0.5, lambda: self._send_task("start", {"task_name": name, "loop_time": decision.get("loop_time", 1)})).start()
            else:
                self._send_task("start", {"task_name": name, "loop_time": decision.get("loop_time", 1)})
            self.state = "MOVING"
        elif action in ("stop", "stop_task"):
            self._send_task("stop")
            self.state = "IDLE"
        elif action in ("pause", "wait", "none"):
            if action == "pause":
                self.state = "PAUSED"
            elif action == "wait":
                self.state = "DECIDING"
            else:
                # none明确表示车辆信号只记录，不改变底盘任务。
                robot_state = str(self.last_task_status.get("task_state", ""))
                self.state = "MOVING" if robot_state == "STATE_DOING" else self.state
        else:
            raise ValueError("unsupported decision action: " + action)

    def _finish_ptz_action(self):
        """云台点位动作结束，不自动暂停/恢复底盘，也不等待车辆信号。"""
        self._event("POINT_ACTION_FINISHED")
        if self.state in ("PTZ_POSITIONING", "SCANNING"):
            self.state = "MOVING"
        self._publish_status()

    def _task_status(self, raw):
        try:
            message = parse_message(raw)
            self.last_task_status = message.get("data", {})
            state = str(self.last_task_status.get("task_state", ""))
            reported_name = str(
                self.last_task_status.get("main_task_name") or self.last_task_status.get("task_name") or ""
            ).strip()
            if (
                self.branch_task_name
                and state in ("STATE_DOING", "STATE_PAUSE")
                and (not reported_name or reported_name == self.branch_task_name)
            ):
                self.state = "BRANCH_RUNNING"
            elif state == "STATE_DOING" and not self.branch_task_name and self.state in (
                "STARTING", "RETURNING_MAIN", "FINISHED"
            ):
                self.state = "MOVING"
            if state in ("STATE_FAIL", "STATE_CANCEL"):
                self.state = "ERROR" if state == "STATE_FAIL" else "IDLE"
                self._event(state, self.last_task_status)
            elif state == "STATE_FINISH" and self.state not in ("COLLECTING", "SCANNING", "DECIDING"):
                is_finished_branch = bool(self.branch_task_name) and (
                    reported_name == self.branch_task_name
                    or self.state in ("BRANCH_RUNNING", "DISPATCHING", "MOVING")
                )
                if is_finished_branch and self.branch_return_to_main and self.task_name:
                    branch_name = self.branch_task_name
                    self.branch_task_name = ""
                    self.branch_return_to_main = False
                    self.state = "RETURNING_MAIN"
                    self._event("BRANCH_TASK_FINISHED", {
                        "branch_task": branch_name, "main_task": self.task_name,
                    })
                    if self.branch_restart_timer is not None:
                        self.branch_restart_timer.shutdown()
                    self.branch_restart_timer = rospy.Timer(
                        rospy.Duration(self.branch_restart_delay),
                        self._restart_main_task,
                        oneshot=True,
                    )
                else:
                    self.state = "FINISHED"
                    self._event("TASK_FINISHED")
                    self._play_voice(self.voice_events.get("task_finished", ""), "TASK_FINISHED")
            self._publish_status()
        except Exception:
            pass

    def _restart_main_task(self, _event=None):
        with self.lock:
            self.branch_restart_timer = None
            if not self.task_name or self.state == "IDLE":
                return
            self.active_point = None
            self.active_rule = None
            self.point_ptz_ready = False
            self._send_task("start", {"task_name": self.task_name, "loop_time": 1})
            self.state = "MOVING"
            self._event("MAIN_TASK_RESTART_REQUESTED", {"main_task": self.task_name})
            self._publish_status()

    def _cancel_timer(self):
        if self.stop_timer is not None:
            self.stop_timer.shutdown()
            self.stop_timer = None
        self.detection_context = None

    def _cancel_active(self):
        self._cancel_timer()
        if self.branch_restart_timer is not None:
            self.branch_restart_timer.shutdown()
            self.branch_restart_timer = None
        point = self.active_point or {}
        cancel = make_message("detection.control", action="cancel", task_id=self.task_id, point_seq=point.get("point_seq", 0), point_name=point.get("point_name", ""))
        self.detection_pub.publish(String(data=dumps(cancel)))
        self.ptz_pub.publish(String(data=dumps(make_message("ptz.command", action="stop", mode="auto"))))
        self.pending_ptz.clear()
        self.point_ptz_ready = False


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    InspectionOrchestrator()
    rospy.spin()
