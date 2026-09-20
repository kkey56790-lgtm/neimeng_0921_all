#!/usr/bin/env python3
import os
import threading
import time

import rospy
from onvif import ONVIFCamera
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, parse_message


class HikvisionPTZ:
    def __init__(self, host, port, username, password):
        rospy.loginfo("连接海康ONVIF云台 %s:%s", host, port)
        self.camera = ONVIFCamera(host, port, username, password)
        self.media = self.camera.create_media_service()
        self.ptz = self.camera.create_ptz_service()
        profiles = self.media.GetProfiles()
        profile = next((p for p in profiles if getattr(p, "PTZConfiguration", None)), profiles[0])
        self.token = profile.token
        rospy.loginfo("ONVIF连接成功，Profile=%s", getattr(profile, "Name", self.token))

    def move(self, pan, tilt, zoom=0.0):
        request = self.ptz.create_type("ContinuousMove")
        request.ProfileToken = self.token
        request.Velocity = {
            "PanTilt": {"x": float(pan), "y": float(tilt)},
            "Zoom": {"x": float(zoom)},
        }
        self.ptz.ContinuousMove(request)

    def stop(self):
        request = self.ptz.create_type("Stop")
        request.ProfileToken = self.token
        request.PanTilt = True
        request.Zoom = True
        self.ptz.Stop(request)

    def goto_preset(self, preset, speed=1.0):
        request = self.ptz.create_type("GotoPreset")
        request.ProfileToken = self.token
        request.PresetToken = str(preset)
        speed = max(0.1, min(float(speed), 1.0))
        request.Speed = {
            "PanTilt": {"x": speed, "y": speed},
            "Zoom": {"x": speed},
        }
        try:
            self.ptz.GotoPreset(request)
        except Exception:
            # 部分旧固件不接受Speed字段，回退到摄像机预置点默认速度。
            request = self.ptz.create_type("GotoPreset")
            request.ProfileToken = self.token
            request.PresetToken = str(preset)
            self.ptz.GotoPreset(request)

    def home(self):
        request = self.ptz.create_type("GotoHomePosition")
        request.ProfileToken = self.token
        self.ptz.GotoHomePosition(request)

    def get_position(self):
        """读取设备实际PTZ状态；部分摄像机固件可能不提供位置。"""
        request = self.ptz.create_type("GetStatus")
        request.ProfileToken = self.token
        status = self.ptz.GetStatus(request)
        position = getattr(status, "Position", None)
        if position is None:
            return None
        pan_tilt = getattr(position, "PanTilt", None)
        zoom = getattr(position, "Zoom", None)
        return {
            "pan": float(getattr(pan_tilt, "x", 0.0)) if pan_tilt is not None else None,
            "tilt": float(getattr(pan_tilt, "y", 0.0)) if pan_tilt is not None else None,
            "zoom": float(getattr(zoom, "x", 0.0)) if zoom is not None else None,
        }


class PTZController:
    DIRECTIONS = {
        "up": (0.0, 1.0), "down": (0.0, -1.0),
        "left": (-1.0, 0.0), "right": (1.0, 0.0),
    }

    def __init__(self):
        self.host = rospy.get_param("~host", "192.168.2.64")
        self.port = int(rospy.get_param("~port", 80))
        self.username = os.environ.get("PTZ_USERNAME", rospy.get_param("~username", "admin"))
        self.password = os.environ.get("PTZ_PASSWORD", rospy.get_param("~password", "okwy1688"))
        self.reconnect_seconds = float(rospy.get_param("~reconnect_seconds", 5.0))
        self.default_settle = float(rospy.get_param("~preset_settle_seconds", 2.0))
        self.preset_speed = max(0.1, min(float(rospy.get_param("~preset_speed", 1.0)), 1.0))
        self.default_steps = rospy.get_param("~default_sweep_steps", [])
        self.mode = "auto"
        self.device_state = "IDLE"
        self.last_preset = ""
        self.last_position = None
        self.version = 0
        self.pending = None
        self.condition = threading.Condition()
        self.driver_lock = threading.RLock()
        self.driver = None
        self.connection_error = "尚未连接"
        # 先建立ROS诊断链路，再连接设备；即使ONVIF失败，话题也必须存在。
        self.result_pub = rospy.Publisher(Topics.PTZ_RESULT, String, queue_size=30)
        self.health_pub = rospy.Publisher(Topics.HEALTH, String, queue_size=10)
        rospy.Subscriber(Topics.PTZ_COMMAND, String, self._command, queue_size=30)
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.shutdown)
        self._connect()
        self.reconnect_timer = rospy.Timer(rospy.Duration(self.reconnect_seconds), self._reconnect)

    def _connect(self):
        with self.driver_lock:
            if self.driver is not None:
                return True
            try:
                self.driver = HikvisionPTZ(self.host, self.port, self.username, self.password)
                self.connection_error = ""
                self._publish_health("online")
                return True
            except Exception as exc:
                self.driver = None
                self.connection_error = str(exc)
                rospy.logerr("ONVIF连接失败，将自动重试: %s", exc)
                self._publish_health("offline", self.connection_error)
                return False

    def _reconnect(self, _event=None):
        if self.driver is None and not rospy.is_shutdown():
            self._connect()

    def _require_driver(self):
        if self.driver is None and not self._connect():
            raise RuntimeError("ONVIF未连接: " + self.connection_error)
        return self.driver

    def _publish_health(self, state, error=""):
        message = make_message("health.ptz", source="ptz_controller", state=state, data={"mode": self.mode, "error": error})
        self.health_pub.publish(String(data=dumps(message)))

    def _publish_result(self, command, status, error=""):
        message = make_message(
            "ptz.result", source="ptz_controller", status=status,
            command_id=command.get("command_id", command.get("tid", command.get("message_id", ""))),
            task_id=command.get("task_id", ""), point_seq=command.get("point_seq", 0),
            point_name=command.get("point_name", ""), action=command.get("action", ""),
            data={
                "mode": self.mode,
                "device_state": self.device_state,
                "preset": self.last_preset,
                "position": self.last_position,
                "error": error,
            },
        )
        self.result_pub.publish(String(data=dumps(message)))

    def _command(self, raw):
        try:
            command = parse_message(raw)
            action = str(command.get("action", command.get("method", ""))).lower()
            command["action"] = action
            if action == "mode":
                mode = str(command.get("data", {}).get("mode", "auto")).lower()
                if mode not in ("auto", "manual"):
                    raise ValueError("mode must be auto or manual")
                with self.condition:
                    self.version += 1
                    self.pending = None
                    self.mode = mode
                    self._require_driver().stop()
                self._publish_result(command, "success")
                self._publish_health("online")
                return
            if action == "stop":
                with self.condition:
                    self.version += 1
                    self.pending = None
                    self._require_driver().stop()
                    self.device_state = "STOPPED"
                    self._refresh_position()
                self._publish_result(command, "success")
                return
            requested_mode = str(command.get("mode", "auto")).lower()
            if requested_mode == "manual" and self.mode != "manual":
                self.mode = "manual"
            if requested_mode == "auto" and self.mode == "manual":
                self._publish_result(command, "rejected", "PTZ is in manual mode")
                return
            with self.condition:
                self.version += 1
                command["_version"] = self.version
                self.pending = command
                self.condition.notify_all()
            self.device_state = "QUEUED_{}".format(action.upper())
            self._publish_result(command, "accepted")
        except Exception as exc:
            rospy.logerr("PTZ命令错误: %s", exc)
            if 'command' in locals():
                self.device_state = "ERROR"
                self._publish_result(command, "failed", str(exc))
            self._publish_health("offline" if self.driver is None else "error", str(exc))

    def _valid(self, version):
        with self.condition:
            return version == self.version and not rospy.is_shutdown()

    def _wait(self, seconds, version):
        end = time.monotonic() + max(0.0, float(seconds))
        while time.monotonic() < end:
            if not self._valid(version):
                return False
            time.sleep(min(0.05, end - time.monotonic()))
        return True

    def _timed_move(self, pan, tilt, duration, version):
        self.driver.move(pan, tilt)
        try:
            return self._wait(duration, version)
        finally:
            self.driver.stop()

    def _execute(self, command):
        self._require_driver()
        action, data, version = command["action"], command.get("data", {}), command["_version"]
        if action == "goto_preset":
            self.device_state = "MOVING_TO_PRESET"
            self.last_preset = str(data.get("preset", "1"))
            self.driver.stop()
            self.driver.goto_preset(self.last_preset, data.get("speed", self.preset_speed))
            if not self._wait(data.get("settle_seconds", self.default_settle), version):
                raise InterruptedError("superseded")
            self._refresh_position()
            self.device_state = "PRESET_READY"
        elif action == "home":
            self.device_state = "MOVING_HOME"
            self.last_preset = str(data.get("preset", "1"))
            self.driver.stop()
            try:
                self.driver.goto_preset(data.get("preset", "1"), data.get("speed", self.preset_speed))
            except Exception:
                self.driver.home()
            if not self._wait(data.get("settle_seconds", self.default_settle), version):
                raise InterruptedError("superseded")
            self._refresh_position()
            self.device_state = "HOME_READY"
        elif action == "move":
            direction = str(data.get("direction", "stop")).lower()
            if not data.get("start", True) or direction == "stop":
                self.driver.stop()
                self.device_state = "STOPPED"
                self._refresh_position()
            else:
                if direction not in self.DIRECTIONS:
                    raise ValueError("invalid direction: " + direction)
                speed = max(0.1, min(float(data.get("speed", 5)) / 10.0, 1.0))
                pan, tilt = self.DIRECTIONS[direction]
                self.driver.move(pan * speed, tilt * speed)
                self.device_state = "MOVING_{}".format(direction.upper())
        elif action == "sweep":
            self.device_state = "SCANNING"
            steps = data.get("steps") or self.default_steps
            repeat = max(1, int(data.get("repeat", 1)))
            for _ in range(repeat):
                for step in steps:
                    direction = str(step.get("direction", "")).lower()
                    if direction not in self.DIRECTIONS:
                        raise ValueError("invalid sweep direction: " + direction)
                    speed = max(0.1, min(float(step.get("speed", 5)) / 10.0, 1.0))
                    pan, tilt = self.DIRECTIONS[direction]
                    if not self._timed_move(pan * speed, tilt * speed, step.get("duration", 1.0), version):
                        raise InterruptedError("superseded")
                    if not self._wait(step.get("pause", 0.1), version):
                        raise InterruptedError("superseded")
            self._refresh_position()
            self.device_state = "SCAN_FINISHED"
        else:
            raise ValueError("unsupported PTZ action: " + action)

    def _refresh_position(self):
        try:
            self.last_position = self.driver.get_position()
        except Exception as exc:
            # 有些固件支持动作但不支持GetStatus，不能因此判定动作失败。
            rospy.logwarn_throttle(10.0, "无法读取PTZ实际位置: %s", exc)
            self.last_position = None

    def _worker(self):
        while not rospy.is_shutdown():
            with self.condition:
                while self.pending is None and not rospy.is_shutdown():
                    self.condition.wait(timeout=0.5)
                command, self.pending = self.pending, None
            if command is None:
                continue
            try:
                self._execute(command)
                if self._valid(command["_version"]):
                    self._publish_result(command, "success")
            except InterruptedError:
                self.device_state = "CANCELLED"
                self._publish_result(command, "cancelled")
            except Exception as exc:
                rospy.logerr("PTZ动作失败: %s", exc)
                try:
                    if self.driver is not None:
                        self.driver.stop()
                except Exception:
                    pass
                self.device_state = "ERROR"
                self._publish_result(command, "failed", str(exc))
                self._publish_health("error", str(exc))

    def shutdown(self):
        with self.condition:
            self.version += 1
            self.condition.notify_all()
        try:
            if self.driver is not None:
                self.driver.stop()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("ptz_controller")
    PTZController()
    rospy.spin()
