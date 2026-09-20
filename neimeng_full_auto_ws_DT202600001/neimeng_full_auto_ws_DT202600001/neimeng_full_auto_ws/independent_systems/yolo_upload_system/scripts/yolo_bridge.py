#!/usr/bin/env python3
import threading
import time

import rospy
from std_msgs.msg import String

from inspection_interfaces.protocol import Topics, dumps, make_message, parse_message

try:
    import zmq
except ImportError:
    zmq = None


class YoloBridge:
    def __init__(self):
        if zmq is None:
            raise RuntimeError("pyzmq未安装，请执行 sudo apt install python3-zmq")
        self.address = rospy.get_param("~zmq_address", "tcp://127.0.0.1:5566")
        self.target_class = str(rospy.get_param("~target_class", "truck"))
        self.target_class_id = int(rospy.get_param("~target_class_id", 9))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.5))
        self.minimum_frames = int(rospy.get_param("~minimum_frames", 10))
        self.minimum_detected_frames = int(rospy.get_param("~minimum_detected_frames", 3))
        self.minimum_frame_ratio = float(rospy.get_param("~minimum_frame_ratio", 0.30))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 2.0))
        self.frame_publish_interval = float(rospy.get_param("~frame_publish_interval", 0.5))

        self.frame_pub = rospy.Publisher(Topics.DETECTION_FRAME, String, queue_size=10)
        self.result_pub = rospy.Publisher(Topics.DETECTION_RESULT, String, queue_size=10)
        self.health_pub = rospy.Publisher(Topics.HEALTH, String, queue_size=10)
        rospy.Subscriber(Topics.DETECTION_CONTROL, String, self._control, queue_size=10)

        self.lock = threading.Lock()
        self.active = None
        self.latest = None
        self.last_receive = 0.0
        self.last_frame_publish = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()
        rospy.on_shutdown(self.shutdown)

    def _receive(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.setsockopt(zmq.RCVTIMEO, 500)
        socket.connect(self.address)
        rospy.loginfo("连接YOLO ZMQ发布端: %s", self.address)
        try:
            while self.running and not rospy.is_shutdown():
                try:
                    frame = socket.recv_json()
                except zmq.Again:
                    self._publish_health()
                    continue
                now = time.monotonic()
                with self.lock:
                    self.latest = frame
                    self.last_receive = now
                    if self.active is not None:
                        self._collect_frame(frame)
                if now - self.last_frame_publish >= self.frame_publish_interval:
                    self.last_frame_publish = now
                    message = make_message("detection.frame", source="yolo_bridge", data=frame)
                    self.frame_pub.publish(String(data=dumps(message)))
                self._publish_health()
        finally:
            socket.close(linger=0)
            context.term()

    def _collect_frame(self, frame):
        collection = self.active
        collection["total_frames"] += 1
        objects = frame.get("objects", []) if isinstance(frame, dict) else []
        trucks = []
        for obj in objects:
            name = str(obj.get("class_name", ""))
            class_id = int(obj.get("class_id", -1))
            confidence = float(obj.get("confidence", 0.0))
            id_matches = self.target_class_id >= 0 and class_id == self.target_class_id
            if (name == self.target_class or id_matches) and confidence >= self.min_confidence:
                trucks.append(obj)
        if trucks:
            collection["detected_frames"] += 1
            collection["best_confidence"] = max(
                collection["best_confidence"],
                max(float(obj.get("confidence", 0.0)) for obj in trucks),
            )
            collection["max_count"] = max(collection["max_count"], len(trucks))
            collection["best_objects"] = trucks

    def _control(self, raw):
        try:
            message = parse_message(raw)
            action = str(message.get("action", "")).lower()
            if action == "start":
                control_data = message.get("data", {}) if isinstance(message.get("data", {}), dict) else {}
                with self.lock:
                    self.active = {
                        "task_id": message.get("task_id", ""),
                        "point_seq": int(message.get("point_seq", 0)),
                        "point_name": message.get("point_name", ""),
                        "started_ms": int(time.time() * 1000),
                        "total_frames": 0,
                        "detected_frames": 0,
                        "best_confidence": 0.0,
                        "max_count": 0,
                        "best_objects": [],
                        "collection_mode": str(control_data.get("collection_mode", "truck")).lower(),
                    }
                rospy.loginfo("开始采集点位: %s", message.get("point_name", ""))
            elif action == "stop":
                self._finish("requested")
            elif action == "cancel":
                with self.lock:
                    self.active = None
            else:
                raise ValueError("unsupported detection action: " + action)
        except Exception as exc:
            rospy.logerr("检测控制命令错误: %s", exc)

    def _finish(self, reason):
        with self.lock:
            collection, self.active = self.active, None
            age = time.monotonic() - self.last_receive if self.last_receive else float("inf")
        if collection is None:
            return
        total = collection["total_frames"]
        detected = collection["detected_frames"]
        collection_mode = str(collection.get("collection_mode", "truck")).lower()
        ratio = float(detected) / total if total else 0.0
        if total == 0 or age > self.stale_timeout:
            state = "DETECTION_ERROR"
        elif (
            total >= self.minimum_frames
            and detected >= self.minimum_detected_frames
            and ratio >= self.minimum_frame_ratio
            and collection["best_confidence"] >= self.min_confidence
        ):
            state = "INSPECTION_ITEMS_PRESENT" if collection_mode == "inspection" else "VEHICLE_PRESENT"
        else:
            state = "INSPECTION_ITEMS_ABSENT" if collection_mode == "inspection" else "VEHICLE_ABSENT"
        data = dict(collection)
        data.update({
            "state": state,
            "vehicle_present": True if state == "VEHICLE_PRESENT" else False if state == "VEHICLE_ABSENT" else None,
            "frame_ratio": round(ratio, 4),
            "finished_ms": int(time.time() * 1000),
            "finish_reason": reason,
        })
        message = make_message(
            "detection.result", source="yolo_bridge", state=state,
            task_id=collection["task_id"], point_seq=collection["point_seq"],
            point_name=collection["point_name"], data=data,
        )
        self.result_pub.publish(String(data=dumps(message)))
        rospy.loginfo("点位检测完成 %s: %s", collection["point_name"], state)

    def _publish_health(self):
        age = time.monotonic() - self.last_receive if self.last_receive else float("inf")
        state = "online" if age <= self.stale_timeout else "offline"
        message = make_message("health.yolo", source="yolo_bridge", state=state, data={"last_frame_age": None if age == float("inf") else round(age, 3), "collecting": self.active is not None})
        self.health_pub.publish(String(data=dumps(message)))

    def shutdown(self):
        self.running = False
        self.thread.join(timeout=1.0)


if __name__ == "__main__":
    rospy.init_node("yolo_bridge")
    YoloBridge()
    rospy.spin()
