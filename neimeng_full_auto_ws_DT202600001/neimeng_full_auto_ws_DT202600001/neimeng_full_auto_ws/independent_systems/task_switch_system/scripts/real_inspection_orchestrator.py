#!/usr/bin/env python3
"""Real numbered inspection flow with gated YOLO, recharge and obstacle handling."""

import copy
import threading
import time

import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, new_id, parse_message
from inspection_orchestrator import InspectionOrchestrator
from real_flow_rules import (
    TaskCatalogResolver, available_task_numbers, extract_battery_percent,
    extract_obstacle_state, next_task_number, start_point_number,
    task_area_prefix, task_number,
)


class RealInspectionOrchestrator(InspectionOrchestrator):
    def __init__(self):
        self.flow_initialized = False
        self.max_task_number = int(rospy.get_param("~numbered_flow/max_task_number", 10))
        self.low_battery_threshold = float(rospy.get_param("~numbered_flow/low_battery_threshold", 30.0))
        self.charge_target = float(rospy.get_param("~numbered_flow/charge_target", 80.0))
        self.recharge_keywords = list(rospy.get_param(
            "~numbered_flow/recharge_task_keywords", ["回充", "充电", "返航充电"]))
        self.task_switch_delay = max(0.1, float(rospy.get_param("~numbered_flow/task_switch_delay", 0.8)))
        self.obstacle_timeout = max(0.1, float(rospy.get_param("~numbered_flow/obstacle_timeout", 5.0)))
        self.reverse_speed = abs(float(rospy.get_param("~numbered_flow/reverse_speed", 0.15)))
        self.reverse_duration = max(0.1, float(rospy.get_param("~numbered_flow/reverse_duration", 2.0)))
        self.reverse_wait_for_start = bool(rospy.get_param(
            "~numbered_flow/reverse_wait_for_start_point", True))
        # 实测默认不因“无车”结论立刻切换编号任务，避免模型误检/漏检导致跳任务。
        self.advance_on_empty = bool(rospy.get_param(
            "~numbered_flow/advance_on_empty", False))

        self.task_resolver = TaskCatalogResolver(self.max_task_number)
        self.catalog_task_names = []
        self.preferred_area = ""
        self.current_execution = None
        self.execution_generation = 0
        self.handled_finish_generation = -1
        self.platform_sequence = 0
        self.platform_queue = []
        self.recharge_pending = False
        self.charging = False
        self.recharge_resume_number = 1
        self.last_battery = None
        self.expected_detection = None
        self.expected_inspection_detection = None
        self.detected_pairs = set()
        self.paused_for_detection = False
        self.vehicle_confirmed_generation = -1
        self.obstacle_present_since = None
        self.obstacle_pause_sent = False
        self.reverse_command_id = ""
        self.switch_timer = None
        self.switching_task = False
        self.scheduler_status_pub = None

        super().__init__()
        self.scheduler_status_pub = rospy.Publisher(Topics.TASK_STATUS, String, queue_size=20)
        rospy.Subscriber(Topics.ROBOT_TELEMETRY, String, self._robot_telemetry, queue_size=20)
        rospy.Subscriber(Topics.ROBOT_RESULT, String, self._robot_result, queue_size=20)
        self.flow_initialized = True
        self._publish_status()

    def _catalog(self, raw):
        super()._catalog(raw)
        try:
            message = parse_message(raw)
            catalog = message.get("data", {})
            tasks = catalog.get("all_tasks") or catalog.get("tasks", [])
            names = [
                str(item.get("name") or item.get("task_name") or "").strip()
                for item in tasks if isinstance(item, dict)
            ]
            names = [name for name in names if name]
            with self.lock:
                self.catalog_task_names = names
                self.task_resolver.update(names)
        except Exception as exc:
            rospy.logwarn("编号任务目录解析失败: %s", exc)

    def _publish_status(self, error=""):
        if not getattr(self, "flow_initialized", False):
            return super()._publish_status(error)
        point = self.active_point or {}
        execution = copy.deepcopy(self.current_execution) if self.current_execution else {}
        message = make_message(
            "inspection.status", source="real_inspection_orchestrator",
            robot_code=self.robot_code, task_id=self.task_id, task_name=self.task_name,
            state=self.state, point_seq=point.get("point_seq", 0),
            point_name=point.get("point_name", ""),
            data={
                "rule": self.active_rule or {}, "error": error,
                "robot_task": self.last_task_status, "ptz_ready": self.point_ptz_ready,
                "numbered_flow": {
                    "max_task_number": self.max_task_number,
                    "available_task_numbers": available_task_numbers(
                        self.catalog_task_names, self.max_task_number),
                    "preferred_area": self.preferred_area,
                    "current_execution": execution,
                    "platform_queue": copy.deepcopy(self.platform_queue),
                    "recharge_pending": self.recharge_pending,
                    "charging": self.charging,
                    "battery": self.last_battery,
                    "expected_detection": copy.deepcopy(self.expected_detection),
                    "obstacle_elapsed": (
                        round(time.monotonic() - self.obstacle_present_since, 2)
                        if self.obstacle_present_since else 0.0),
                },
            },
        )
        self.control_pub.publish(String(data=dumps(message)))

    def _publish_command_status(self, command_id, status, detail):
        if not command_id or self.scheduler_status_pub is None:
            return
        message = make_message(
            "task.status", source="real_inspection_orchestrator",
            command_id=command_id, command_status=status,
            data=detail if isinstance(detail, dict) else {"detail": detail},
        )
        self.scheduler_status_pub.publish(String(data=dumps(message)))

    def _next_number(self, number, maximum=None):
        return next_task_number(
            number, self.catalog_task_names,
            self.max_task_number if maximum is None else maximum,
        )

    def _resolve_numbered_task(self, number, requested_name=""):
        return self.task_resolver.resolve_number(
            int(number), preferred_area=self.preferred_area, requested_name=requested_name)

    def _set_execution(self, name, number, kind, command_id="", platform_sequence=0):
        self.execution_generation += 1
        self.current_execution = {
            "name": str(name), "number": number, "kind": str(kind),
            "command_id": str(command_id or ""),
            "platform_sequence": int(platform_sequence or 0),
            "generation": self.execution_generation,
        }
        self.handled_finish_generation = -1
        self.vehicle_confirmed_generation = -1
        self.paused_for_detection = False
        self.obstacle_present_since = None
        self.obstacle_pause_sent = False
        self.reverse_command_id = ""

    def _cancel_switch_timer(self):
        if self.switch_timer is not None:
            self.switch_timer.shutdown()
            self.switch_timer = None

    def _start_actual_task(self, name, number, kind, command_id="", platform_sequence=0, stop_first=True):
        self._cancel_switch_timer()
        old = copy.deepcopy(self.current_execution) if self.current_execution else None
        self._set_execution(name, number, kind, command_id, platform_sequence)
        self.branch_task_name = str(name)
        self.branch_return_to_main = False
        self.switching_task = bool(stop_first and old)
        if self.switching_task:
            self._send_task("stop")

        def start(_event=None):
            with self.lock:
                self.switch_timer = None
                self.switching_task = False
                current = self.current_execution or {}
                if current.get("name") != name:
                    return
                self._send_task(
                    "start", {"task_name": name, "loop_time": 1},
                    command_id=command_id or None,
                )
                self.state = "RECHARGING" if kind == "recharge" else "STARTING_NUMBERED_TASK"
                self._event("NUMBERED_TASK_START_REQUESTED", {
                    "task_name": name, "task_number": number, "kind": kind,
                })
                self._publish_status()

        if self.switching_task:
            self.switch_timer = rospy.Timer(
                rospy.Duration(self.task_switch_delay), start, oneshot=True)
        else:
            start()

    def _start_numbered_task(self, number, kind="auto", command_id="", requested_name="", platform_sequence=0):
        number = int(number)
        if number < 1 or number > self.max_task_number:
            number = 1
        actual = self._resolve_numbered_task(number, requested_name)
        area = task_area_prefix(actual)
        if area:
            self.preferred_area = area
        self._start_actual_task(
            actual, number, kind, command_id=command_id,
            platform_sequence=platform_sequence, stop_first=bool(self.current_execution),
        )

    def _control(self, raw):
        try:
            message = parse_message(raw)
            action = str(message.get("action", message.get("method", ""))).lower()
            data = dict(message.get("data", {}))
            if action in ("inspection.platform_task", "platform_task", "task.start"):
                with self.lock:
                    self._queue_platform_task(data, message.get("command_id", message.get("tid", "")))
                    self._publish_status()
                return
            if action in ("start", "inspection.start"):
                requested = str(data.get("task_name", data.get("name", ""))).strip()
                number = task_number(requested)
                if number is None:
                    raise ValueError("真实编号流程启动任务必须包含“任务0…任务{}”关键词".format(
                        self.max_task_number))
                actual = self._resolve_numbered_task(number, requested)
                area = task_area_prefix(actual)
                if area:
                    self.preferred_area = area
                self._set_execution(actual, number, "auto", message.get("command_id", message.get("tid", "")))
                self.detected_pairs.clear()
                data["task_name"] = actual
                data["name"] = actual
                forwarded = dict(message)
                forwarded["data"] = data
                super()._control(forwarded)
                self._event("NUMBERED_FLOW_STARTED", {
                    "task_number": number, "actual_task_name": actual,
                    "preferred_area": self.preferred_area,
                })
                return
            super()._control(message)
        except Exception as exc:
            rospy.logerr("真实编号巡检控制失败: %s", exc)
            self.state = "ERROR"
            self._publish_status(str(exc))

    def _queue_platform_task(self, data, command_id):
        requested = str(data.get("task_name", data.get("taskName", data.get("name", "")))).strip()
        number = task_number(requested)
        if number is None:
            raw_number = data.get("task_number", data.get("taskNumber"))
            if raw_number is None:
                raise ValueError("中台任务必须包含任务编号或名称中的“任务N”关键词")
            number = int(raw_number)
        actual = self._resolve_numbered_task(number, requested)
        area = task_area_prefix(actual)
        if area:
            self.preferred_area = area
        self.platform_sequence += 1
        item = {
            "name": actual, "number": int(number), "command_id": str(command_id or ""),
            "sequence": self.platform_sequence,
        }
        recharge_window = self.recharge_pending or self.charging or (
            self.current_execution and self.current_execution.get("kind") == "recharge")
        if recharge_window:
            for old in self.platform_queue:
                self._publish_command_status(old.get("command_id"), "cancelled", {
                    "reason": "superseded_by_latest_during_recharge",
                    "superseded_by": actual,
                })
            self.platform_queue = [item]
            self._event("PLATFORM_TASK_REPLACED_DURING_RECHARGE", item)
        else:
            self.platform_queue.append(item)
            self._event("PLATFORM_TASK_QUEUED", item)
        if not self.current_execution:
            self._start_next_platform_if_any()

    def _start_next_platform_if_any(self):
        if not self.platform_queue:
            return False
        item = self.platform_queue.pop(0)
        self._start_numbered_task(
            item["number"], kind="platform", command_id=item.get("command_id", ""),
            requested_name=item["name"], platform_sequence=item["sequence"],
        )
        return True

    def _point_event(self, raw):
        try:
            message = parse_message(raw)
            point_name = str(message.get("point_name", "")).strip()
            point_number = start_point_number(point_name)
            if (
                str(message.get("event", "")).lower() == "arrived"
                and self.state == "RETURNING_START_AFTER_REVERSE"
                and self.current_execution
                and point_number == self.current_execution.get("number")
            ):
                with self.lock:
                    self.active_point = {"point_name": point_name, "point_seq": point_number}
                    self._event("REVERSE_RETURN_POINT_CONFIRMED", {"point_name": point_name})
                    self._finish_current_execution("reverse_return_confirmed")
                    self._publish_status()
                return
            super()._point_event(message)
        except Exception as exc:
            rospy.logerr("真实点位流程处理失败: %s", exc)

    def _arrival_task_decision(self, point_name, message):
        # Numbered flow owns real task switching. Keep UI override only for explicit simulation.
        if bool(message.get("simulated_signal", False)) and message.get("test_arrival_task_name"):
            return super()._arrival_task_decision(point_name, message)
        return None

    def _start_point_detection(self, point_name, point_seq):
        if not self.detection_collection_enabled:
            return
        current = self.current_execution or {}
        current_number = current.get("number")
        point_number = start_point_number(point_name)
        truck_mode = (
            point_number is not None
            and 1 <= point_number <= self.max_task_number
            and (current_number == point_number or (current_number == 0 and point_number == 1))
        )
        collection_mode = "truck" if truck_mode else "inspection"
        if collection_mode == "inspection" and not bool(
            self.detection_config.get("collect_non_truck_at_other_points", True)
        ):
            return

        pair_key = (
            current.get("generation"), collection_mode,
            point_number if truck_mode else str(point_name),
        )
        if pair_key in self.detected_pairs:
            return
        if self.expected_detection or self.expected_inspection_detection:
            self._event("YOLO_COLLECTION_SKIPPED_BUSY", {
                "point_name": point_name, "collection_mode": collection_mode,
            })
            return

        self.detected_pairs.add(pair_key)
        self._cancel_timer()
        detection_id = new_id("detection")
        expected = {
            "detection_id": detection_id,
            "point_name": point_name,
            "point_number": point_number,
            "generation": current.get("generation"),
            "collection_mode": collection_mode,
        }
        if truck_mode:
            self.expected_detection = expected
        else:
            self.expected_inspection_detection = expected

        sequence = int(point_number if truck_mode else (point_seq or 0))
        self.detection_started_at = time.monotonic()
        self.detection_context = {
            "point_name": point_name,
            "point_seq": sequence,
            "collection_mode": collection_mode,
        }
        if truck_mode:
            self.paused_for_detection = True
            self._send_task("pause")

        message = make_message(
            "detection.control", source="real_inspection_orchestrator", action="start",
            task_id=self.task_id, point_seq=sequence, point_name=point_name,
            data={
                "detection_id": detection_id,
                "task_number": point_number,
                "collection_mode": collection_mode,
            },
        )
        self.detection_pub.publish(String(data=dumps(message)))
        if truck_mode:
            self.state = "WAITING_YOLO"
            self._event("DETECTION_STARTED", {
                "point_name": point_name, "task_number": point_number,
                "detection_id": detection_id, "truck_only": True,
            })
        else:
            self._event("INSPECTION_DETECTION_STARTED", {
                "point_name": point_name, "detection_id": detection_id,
                "exclude_class": "truck", "record_only": True,
            })

        point_config = self.detection_points.get(point_name, {}) \
            if isinstance(self.detection_points, dict) else {}
        seconds = float(point_config.get(
            "collect_seconds", self.detection_config.get("collect_seconds", 6.0))) \
            if isinstance(point_config, dict) else float(
                self.detection_config.get("collect_seconds", 6.0))
        self.stop_timer = rospy.Timer(
            rospy.Duration(max(0.05, seconds)), self._stop_detection, oneshot=True)

    def _detection_result(self, raw):
        try:
            message = parse_message(raw)
            data = message.get("data", {}) if isinstance(message.get("data", {}), dict) else {}
            collection_mode = str(data.get("collection_mode", "truck")).lower()
            detection_id = str(data.get("detection_id", ""))
            point_name = str(message.get("point_name", "")).strip()

            if collection_mode == "inspection":
                expected = self.expected_inspection_detection or {}
                if not expected or point_name != expected.get("point_name"):
                    rospy.logwarn("忽略非当前点位的非truck巡检结果: %s", point_name)
                    return
                if detection_id and detection_id != expected.get("detection_id"):
                    rospy.logwarn("忽略旧的非truck巡检结果: %s", detection_id)
                    return
                self.expected_inspection_detection = None
                state = str(message.get("state", data.get("state", "DETECTION_ERROR")))
                result = make_message(
                    "inspection.result", source="real_inspection_orchestrator",
                    robot_code=self.robot_code, task_id=self.task_id,
                    task_name=self.task_name, point_seq=message.get("point_seq", 0),
                    point_name=point_name, state=state,
                    data={
                        "detection": data,
                        "decision": {"action": "record_only"},
                        "excluded_class": "truck",
                    },
                )
                self.result_pub.publish(String(data=dumps(result)))
                self._event("NON_TRUCK_INSPECTION_RESULT", {
                    "point_name": point_name,
                    "state": state,
                    "detected_frames": data.get("detected_frames", 0),
                    "max_count": data.get("max_count", 0),
                })
                self._publish_status()
                return

            point_number = start_point_number(point_name)
            expected = self.expected_detection or {}
            if not expected or point_number != expected.get("point_number"):
                rospy.logwarn("忽略非当前起始点的truck结果: %s", point_name)
                return
            if detection_id and detection_id != expected.get("detection_id"):
                rospy.logwarn("忽略旧的truck采集结果: %s", detection_id)
                return
            self.expected_detection = None
            super()._detection_result(message)
        except Exception as exc:
            rospy.logerr("真实YOLO结果校验失败: %s", exc)
    def _resolve_vehicle_rule(self, point_name):
        number = start_point_number(point_name)
        if number is None or number < 1 or number > self.max_task_number:
            return super()._resolve_vehicle_rule(point_name)
        return {
            "on_vehicle": {"action": "numbered_vehicle", "task_number": number},
            "on_empty": {"action": "numbered_empty", "task_number": number},
            "on_error": {"action": "numbered_detection_error", "task_number": number},
        }

    def _execute_decision(self, decision):
        if isinstance(decision, str):
            decision = {"action": decision}
        action = str(decision.get("action", "")).lower()
        number = int(decision.get("task_number", 0) or 0)
        if action == "numbered_vehicle":
            self.vehicle_confirmed_generation = (self.current_execution or {}).get("generation", -1)
            self._event("TRUCK_CONFIRMED", {"task_number": number})
            current_number = (self.current_execution or {}).get("number")
            if current_number == number:
                if self.paused_for_detection:
                    self._send_task("resume")
                self.paused_for_detection = False
                self.state = "NUMBERED_TASK_RUNNING"
            else:
                self._start_numbered_task(number, kind="auto")
            return
        if action == "numbered_empty":
            self.paused_for_detection = False
            if self.advance_on_empty:
                self._event("NO_TRUCK_TASK_SKIPPED", {
                    "task_number": number, "advance_on_empty": True,
                })
                next_number = self._next_number(number, self.max_task_number)
                self._start_numbered_task(next_number, kind="auto")
            else:
                # 漏检时保持当前真实任务；只有任务自身完成后才由流程继续下一编号。
                if self.current_execution:
                    self._send_task("resume")
                self.state = "NUMBERED_TASK_RUNNING"
                self._event("NO_TRUCK_TASK_CONTINUED", {
                    "task_number": number, "advance_on_empty": False,
                })
            return
        if action == "numbered_detection_error":
            self.state = "YOLO_ERROR_WAITING_OPERATOR"
            self._event("YOLO_ERROR_TASK_HELD", {"task_number": number})
            return
        super()._execute_decision(decision)

    def _reported_task_name(self, data):
        return str(data.get("main_task_name") or data.get("task_name") or data.get("taskName") or "").strip()

    def _reported_matches_current(self, reported_name):
        current = self.current_execution or {}
        if not current:
            return False
        if reported_name and reported_name == current.get("name"):
            return True
        current_number = current.get("number")
        return current_number is not None and task_number(reported_name) == current_number

    def _task_status(self, raw):
        try:
            message = parse_message(raw)
            data = message.get("data", {})
            if not isinstance(data, dict):
                return
            self.last_task_status = data
            state = str(data.get("task_state", ""))
            reported_name = self._reported_task_name(data)
            if not state:
                self._publish_status()
                return
            with self.lock:
                if state in ("STATE_DOING", "STATE_PAUSE") and self._reported_matches_current(reported_name):
                    if self.current_execution and self.current_execution.get("kind") == "recharge":
                        self.state = "RECHARGING"
                    elif state == "STATE_PAUSE" and self.paused_for_detection:
                        self.state = "WAITING_YOLO"
                    elif self.obstacle_present_since:
                        self.state = "OBSTACLE_WAIT"
                    else:
                        self.state = "NUMBERED_TASK_RUNNING"
                elif state in ("STATE_FAIL", "STATE_FAILED") and not self.switching_task:
                    self.state = "ERROR"
                    self._event("REAL_TASK_FAILED", data)
                elif state in ("STATE_CANCEL", "STATE_CANCELED") and not self.switching_task:
                    self.state = "IDLE"
                    self._event("REAL_TASK_CANCELLED", data)
                elif state == "STATE_FINISH" and self._reported_matches_current(reported_name):
                    generation = (self.current_execution or {}).get("generation", -1)
                    if generation != self.handled_finish_generation:
                        self.handled_finish_generation = generation
                        if self.current_execution and self.current_execution.get("kind") == "recharge":
                            self.charging = True
                            self.state = "CHARGING_TO_80"
                            self._event("RECHARGE_ROUTE_FINISHED", {"battery": self.last_battery})
                            self._maybe_finish_recharge()
                        else:
                            self._finish_current_execution("real_task_finished")
                self._publish_status()
        except Exception as exc:
            rospy.logwarn("真实任务状态解析失败: %s", exc)

    def _finish_current_execution(self, reason):
        current = copy.deepcopy(self.current_execution) if self.current_execution else None
        if not current:
            return
        number = current.get("number")
        self._event("NUMBERED_TASK_FINISHED", {
            "task_name": current.get("name"), "task_number": number,
            "kind": current.get("kind"), "reason": reason,
        })
        self.current_execution = None
        self.branch_task_name = ""
        self.paused_for_detection = False
        self.vehicle_confirmed_generation = -1
        self.obstacle_present_since = None
        self.obstacle_pause_sent = False
        next_number = self._next_number(number or 0, self.max_task_number)
        if self.recharge_pending:
            self._start_recharge(next_number)
        elif self._start_next_platform_if_any():
            pass
        else:
            self._start_numbered_task(next_number, kind="auto")

    def _collapse_platform_queue_for_recharge(self):
        if len(self.platform_queue) <= 1:
            return
        latest = max(self.platform_queue, key=lambda item: item.get("sequence", 0))
        for old in self.platform_queue:
            if old is not latest:
                self._publish_command_status(old.get("command_id"), "cancelled", {
                    "reason": "superseded_by_latest_during_recharge",
                    "superseded_by": latest.get("name"),
                })
        self.platform_queue = [latest]

    def _start_recharge(self, resume_number):
        if self.current_execution and self.current_execution.get("kind") == "recharge":
            return
        self.recharge_pending = True
        self.recharge_resume_number = int(resume_number or 1)
        self._collapse_platform_queue_for_recharge()
        actual = self.task_resolver.resolve_keywords(self.recharge_keywords, self.preferred_area)
        self.charging = False
        self._start_actual_task(actual, None, "recharge", stop_first=bool(self.current_execution))
        self._event("RECHARGE_STARTED", {
            "task_name": actual, "battery": self.last_battery,
            "resume_task_number": self.recharge_resume_number,
        })

    def _maybe_finish_recharge(self):
        if not self.recharge_pending or self.last_battery is None or self.last_battery < self.charge_target:
            return
        self._event("RECHARGE_COMPLETED", {"battery": self.last_battery})
        self._send_task("stop")
        self.current_execution = None
        self.recharge_pending = False
        self.charging = False
        if self.platform_queue:
            latest = max(self.platform_queue, key=lambda item: item.get("sequence", 0))
            self.platform_queue = [latest]
            self._start_next_platform_if_any()
        else:
            self._start_numbered_task(self.recharge_resume_number, kind="auto")

    def _robot_telemetry(self, raw):
        try:
            message = parse_message(raw)
            telemetry = message.get("data", {})
            battery = extract_battery_percent(telemetry)
            obstacle = extract_obstacle_state(telemetry)
            with self.lock:
                if battery is not None:
                    self.last_battery = battery
                    if battery < self.low_battery_threshold and not self.recharge_pending:
                        self.recharge_pending = True
                        self._event("LOW_BATTERY_RECHARGE_REQUESTED", {"battery": battery})
                        current = copy.deepcopy(self.current_execution) if self.current_execution else None
                        if current and current.get("kind") == "platform":
                            self.platform_queue.append({
                                "name": current.get("name"), "number": current.get("number"),
                                "command_id": "", "sequence": current.get("platform_sequence", 0),
                            })
                            self._collapse_platform_queue_for_recharge()
                            self._start_recharge(current.get("number", 1))
                        elif not current:
                            self._start_recharge(self.recharge_resume_number)
                    self._maybe_finish_recharge()
                self._handle_obstacle(obstacle)
                self._publish_status()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "真实遥测调度解析失败: %s", exc)

    def _handle_obstacle(self, obstacle):
        current = self.current_execution or {}
        monitoring = (
            obstacle is not None and current.get("number") is not None
            and current.get("generation") == self.vehicle_confirmed_generation
            and current.get("kind") != "recharge"
        )
        if not monitoring:
            self.obstacle_present_since = None
            return
        if obstacle:
            if self.obstacle_present_since is None:
                self.obstacle_present_since = time.monotonic()
                self.obstacle_pause_sent = True
                self._send_task("pause")
                self.state = "OBSTACLE_WAIT"
                self._event("OBSTACLE_TIMER_STARTED", {"timeout": self.obstacle_timeout})
                return
            elapsed = time.monotonic() - self.obstacle_present_since
            if elapsed >= self.obstacle_timeout and not self.reverse_command_id:
                self.reverse_command_id = new_id("reverse")
                command = make_message(
                    "robot.command", source="real_inspection_orchestrator",
                    action="robot.reverse", command_id=self.reverse_command_id,
                    task_id=self.task_id,
                    data={"speed": self.reverse_speed, "duration": self.reverse_duration},
                )
                self.robot_pub.publish(String(data=dumps(command)))
                self.state = "REVERSING"
                self._event("REVERSE_REQUESTED", {
                    "elapsed": round(elapsed, 2), "speed": self.reverse_speed,
                    "duration": self.reverse_duration,
                })
        elif self.obstacle_present_since is not None:
            elapsed = time.monotonic() - self.obstacle_present_since
            self.obstacle_present_since = None
            self.obstacle_pause_sent = False
            if not self.reverse_command_id:
                self._send_task("resume")
                self.state = "NUMBERED_TASK_RUNNING"
                self._event("OBSTACLE_CLEARED_TASK_RESUMED", {"elapsed": round(elapsed, 2)})

    def _robot_result(self, raw):
        try:
            message = parse_message(raw)
            if not self.reverse_command_id or message.get("command_id") != self.reverse_command_id:
                return
            with self.lock:
                status = str(message.get("status", "")).lower()
                if status == "success":
                    self._event("REVERSE_COMPLETED", message.get("data", {}))
                    if self.reverse_wait_for_start:
                        self.state = "RETURNING_START_AFTER_REVERSE"
                    else:
                        self._finish_current_execution("reverse_completed")
                elif status in ("failed", "rejected", "cancelled"):
                    self.state = "ERROR"
                    self._event("REVERSE_FAILED", message.get("data", {}))
                self._publish_status()
        except Exception as exc:
            rospy.logwarn("倒车回执解析失败: %s", exc)

    def _cancel_active(self):
        super()._cancel_active()
        self._cancel_switch_timer()
        self.current_execution = None
        self.platform_queue = []
        self.recharge_pending = False
        self.charging = False
        self.expected_detection = None
        self.expected_inspection_detection = None
        self.obstacle_present_since = None
        self.reverse_command_id = ""


if __name__ == "__main__":
    rospy.init_node("inspection_orchestrator")
    RealInspectionOrchestrator()
    rospy.spin()
