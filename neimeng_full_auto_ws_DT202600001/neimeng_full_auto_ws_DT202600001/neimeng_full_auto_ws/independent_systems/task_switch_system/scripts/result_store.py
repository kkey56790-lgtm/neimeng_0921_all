#!/usr/bin/env python3
import json
import os
import sqlite3
import threading

import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, parse_message


class ResultStore:
    def __init__(self):
        path = os.path.expanduser(rospy.get_param("~database", "~/.ros/neimeng_xunjian/inspection.db"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.connection = sqlite3.connect(path, check_same_thread=False)
        self.lock = threading.Lock()
        with self.connection:
            self.connection.execute(
                "CREATE TABLE IF NOT EXISTS messages ("
                "id INTEGER PRIMARY KEY AUTOINCREMENT, message_id TEXT UNIQUE, "
                "timestamp_ms INTEGER, category TEXT, task_id TEXT, point_name TEXT, "
                "state TEXT, payload TEXT, uploaded INTEGER DEFAULT 0)"
            )
        rospy.Subscriber(Topics.EVENT, String, lambda msg: self._save("event", msg), queue_size=100)
        rospy.Subscriber(Topics.RESULT, String, lambda msg: self._save("result", msg), queue_size=100)
        rospy.loginfo("巡检结果数据库: %s", path)

    def _save(self, category, raw):
        try:
            message = parse_message(raw)
            with self.lock, self.connection:
                self.connection.execute(
                    "INSERT OR IGNORE INTO messages(message_id,timestamp_ms,category,task_id,point_name,state,payload) VALUES(?,?,?,?,?,?,?)",
                    (
                        message.get("message_id", ""), message.get("timestamp_ms", 0), category,
                        message.get("task_id", ""), message.get("point_name", ""),
                        message.get("state", message.get("event", "")),
                        json.dumps(message, ensure_ascii=False),
                    ),
                )
        except Exception as exc:
            rospy.logerr("结果保存失败: %s", exc)


if __name__ == "__main__":
    rospy.init_node("result_store")
    ResultStore()
    rospy.spin()
