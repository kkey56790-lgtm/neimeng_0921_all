#!/usr/bin/env python3
"""Real YOLO bridge that accepts collection only for numbered start points."""

import rospy

from inspection_interfaces.protocol import parse_message
from real_flow_rules import start_point_number
from yolo_bridge import YoloBridge


class GatedYoloBridge(YoloBridge):
    def __init__(self):
        self.max_task_number = int(rospy.get_param("~max_task_number", 10))
        super().__init__()

    def _control(self, raw):
        try:
            message = parse_message(raw)
            action = str(message.get("action", "")).lower()
            if action == "start":
                point_name = str(message.get("point_name", "")).strip()
                data = message.get("data", {}) if isinstance(message.get("data", {}), dict) else {}
                collection_mode = str(data.get("collection_mode", "truck")).strip().lower()
                number = start_point_number(point_name)
                if collection_mode != "inspection" and (
                    number is None or number < 1 or number > self.max_task_number
                ):
                    rospy.logwarn("拒绝非起始点truck采集: %s", point_name)
                    return
                if collection_mode == "inspection" and not point_name:
                    rospy.logwarn("拒绝无点位名称的巡检采集")
                    return
                requested_number = data.get("task_number")
                if (
                    collection_mode != "inspection"
                    and requested_number is not None
                    and int(requested_number) != number
                ):
                    rospy.logwarn(
                        "拒绝任务/起始点编号不一致的YOLO采集: task=%s point=%s",
                        requested_number, number,
                    )
                    return
                super()._control(message)
                with self.lock:
                    if self.active is not None:
                        self.active["detection_id"] = str(data.get("detection_id", ""))
                        self.active["task_number"] = number
                        self.active["collection_mode"] = collection_mode
                return
            super()._control(message)
        except Exception as exc:
            rospy.logerr("起始点/巡检点YOLO控制失败: %s", exc)
    def _collect_frame(self, frame):
        collection = self.active
        collection["total_frames"] += 1
        objects = frame.get("objects", []) if isinstance(frame, dict) else []
        matches = []
        target = self.target_class.strip().lower()
        collection_mode = str(collection.get("collection_mode", "truck")).lower()
        for obj in objects:
            name = str(obj.get("class_name", "")).strip().lower()
            try:
                class_id = int(obj.get("class_id", -1))
                confidence = float(obj.get("confidence", 0.0))
            except (TypeError, ValueError):
                continue
            id_matches = self.target_class_id >= 0 and class_id == self.target_class_id
            is_truck = name == target or id_matches
            wanted = not is_truck if collection_mode == "inspection" else is_truck
            if wanted and confidence >= self.min_confidence:
                matches.append(obj)
        if matches:
            collection["detected_frames"] += 1
            collection["best_confidence"] = max(
                collection["best_confidence"],
                max(float(obj.get("confidence", 0.0)) for obj in matches),
            )
            collection["max_count"] = max(collection["max_count"], len(matches))
            collection["best_objects"] = matches

if __name__ == "__main__":
    rospy.init_node("yolo_bridge")
    GatedYoloBridge()
    rospy.spin()
