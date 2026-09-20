#!/usr/bin/env python3
"""Translate real robot point-arrival events into protected PTZ intents."""

import threading

import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, new_id, parse_message


class PTZFollowAdapter:
    def __init__(self):
        self.rules = rospy.get_param("~point_rules", [])
        self.exact = rospy.get_param("~points", {})
        self.default_rule = rospy.get_param("~default_rule", {})
        self.sweep_profiles = rospy.get_param("~sweep_profiles", {})
        self.lock = threading.RLock()
        self.generation = 0
        self.pending_mode = {}
        self.command_pub = rospy.Publisher(Topics.PTZ_COMMAND, String, queue_size=20)
        rospy.Subscriber(Topics.POINT_EVENT, String, self._point, queue_size=20)
        rospy.Subscriber(Topics.PTZ_RESULT, String, self._result, queue_size=30)

    def _resolve(self, point_name):
        exact = self.exact.get(point_name)
        if isinstance(exact, dict):
            return dict(exact)
        compact = point_name.replace(" ", "").lower()
        for rule in self.rules:
            keywords = rule.get("keywords", []) if isinstance(rule, dict) else []
            if any(str(word).replace(" ", "").lower() in compact for word in keywords):
                return dict(rule)
        return dict(self.default_rule)

    def _publish(self, action, point_name, command_id=None, data=None):
        command_id = command_id or new_id("ptz")
        message = make_message(
            "ptz.command", source="ptz_follow_adapter", action=action,
            command_id=command_id, point_name=point_name,
            data=data if isinstance(data, dict) else {},
        )
        self.command_pub.publish(String(data=dumps(message)))
        return command_id

    def _point(self, raw):
        message = parse_message(raw)
        if str(message.get("event", "")).lower() != "arrived":
            return
        point_name = str(message.get("point_name", "")).strip()
        if not point_name:
            return
        rule = self._resolve(point_name)
        with self.lock:
            self.generation += 1
            generation = self.generation
            mode_id = new_id("ptz-mode")
            self.pending_mode.clear()
            self.pending_mode[mode_id] = (generation, point_name, rule)
        self._publish("mode", point_name, mode_id, {"mode": "auto"})

    def _result(self, raw):
        message = parse_message(raw)
        command_id = str(message.get("command_id", ""))
        status = str(message.get("status", "")).lower()
        with self.lock:
            pending = self.pending_mode.get(command_id)
            if not pending:
                return
            if status in ("failed", "rejected", "cancelled"):
                self.pending_mode.pop(command_id, None)
                return
            if status != "success":
                return
            self.pending_mode.pop(command_id, None)
            generation, point_name, rule = pending
            if generation != self.generation:
                return
        action = "home" if bool(rule.get("home", False)) else "goto_preset"
        self._publish(action, point_name, data={
            "preset": str(rule.get("preset", "4")),
            "speed": float(rule.get("speed", 1.0)),
            "settle_seconds": float(rule.get("settle_seconds", 2.0)),
        })
        if bool(rule.get("sweep", False)):
            profile = self.sweep_profiles.get(str(rule.get("sweep_profile", "normal")), {})
            self._publish("sweep", point_name, data={
                "steps": profile.get("steps", []),
                "repeat": int(profile.get("repeat", 1)),
            })


if __name__ == "__main__":
    rospy.init_node("ptz_follow_adapter")
    PTZFollowAdapter()
    rospy.spin()
