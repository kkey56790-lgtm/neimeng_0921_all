#!/usr/bin/env python3
"""YOLO检测项目实时截图上传节点。

检测到任一配置类别后，立即保存一张标注截图，通过
thing/robot/{robotCode}/file 上传；收到 MQTT QoS 1 PUBACK 后删除本地
图片和结果文件。该文件不修改现有YOLO、任务编排或按任务归档上传逻辑。
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

from mqtt_yolo_point_uploader import (
    DEFAULT_DEVICE_CONFIG,
    DEFAULT_MQTT_CONFIG,
    DEFAULT_MQTT_CREDENTIALS,
    DEFAULT_YOLO_CONFIG,
    DEFAULT_YOLO_ENV,
    DEFAULT_DETECTOR_FILE,
    MqttOutbox,
    atomic_write_bytes,
    atomic_write_json,
    build_event_payload,
    build_rtsp_url,
    cv2,
    infer_robot_code,
    load_detection_items,
    now_ms,
    payload_with_image,
    read_yaml,
    resolve_local_file,
    rospy,
    safe_filename,
    String,
    zmq,
)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_RESULTS_ROOT = os.path.join(MODULE_DIR, "realtime_detection_results")
STATUS_TOPIC = "/inspection/status"
INACTIVE_STATES = {"", "IDLE", "FINISHED", "STOPPED"}


class RealtimeSnapshotUploader:
    def __init__(self, args):
        mqtt_config_path = resolve_local_file(args.mqtt_config, "mqtt.yaml")
        credentials_path = resolve_local_file(
            args.mqtt_credentials, "mqtt_credentials.yaml")
        device_path = resolve_local_file(args.device_config, "device.yaml")
        yolo_config_path = resolve_local_file(args.yolo_config, "full_auto_yolo.yaml")
        yolo_env_path = resolve_local_file(args.yolo_env, "yolo.env")
        detector_file = resolve_local_file(args.detector_file, "rtsp_truck.py")

        mqtt_config = read_yaml(mqtt_config_path)
        credentials = read_yaml(credentials_path)
        device_config = read_yaml(device_path)
        yolo_config = read_yaml(yolo_config_path)

        authentication = credentials.setdefault("authentication", {})
        authentication["username"] = str(args.mqtt_username)
        authentication["password"] = str(args.mqtt_password)
        broker = mqtt_config.setdefault("broker", {})
        if args.mqtt_host:
            broker["host"] = args.mqtt_host
        if args.mqtt_port:
            broker["port"] = int(args.mqtt_port)
        if args.file_topic:
            mqtt_config.setdefault("topics", {})["file"] = args.file_topic

        self.robot_code = infer_robot_code(
            args.robot_code,
            os.environ.get("ROBOT_CODE"),
            device_config.get("robot_code"),
            os.path.abspath(__file__),
            os.getcwd(),
        )
        if not self.robot_code:
            raise ValueError("无法确定robot_code，请使用 --robot-code DT202600001")

        self.items = load_detection_items(
            detector_file,
            profile=args.class_profile,
            env_file=yolo_env_path,
            explicit_items=args.detection_items,
        )
        self.by_id = {item["id"]: item for item in self.items}
        self.by_name = {}
        for item in self.items:
            self.by_name[item["name"].strip().lower()] = item
            self.by_name[item["display_name"].strip().lower()] = item

        self.min_confidence = float(
            args.min_confidence
            if args.min_confidence is not None
            else yolo_config.get("min_confidence", 0.35)
        )
        self.upload_interval = max(0.0, float(args.upload_interval))
        self.image_max_width = max(320, int(args.image_max_width))
        self.jpeg_quality = min(100, max(30, int(args.jpeg_quality)))
        self.zmq_address = str(
            args.zmq_address
            or os.environ.get("YOLO_ZMQ_ADDRESS")
            or yolo_config.get("zmq_address")
            or "tcp://127.0.0.1:5566"
        )
        self.rtsp_url = build_rtsp_url(args.rtsp_url, yolo_env_path)
        self.results_root = os.path.abspath(args.results_root)
        os.makedirs(self.results_root, exist_ok=True)

        self.lock = threading.RLock()
        self.latest_image = None
        self.task_active = False
        self.task_id = ""
        self.task_name = ""
        self.point_name = ""
        self.last_upload = {}
        self.stopping = False

        self.outbox = MqttOutbox(
            mqtt_config, credentials, self.robot_code, dry_run=args.dry_run)
        self.outbox_thread = threading.Thread(target=self.outbox.run, daemon=True)
        self.outbox_thread.start()
        self._recover_pending_results()

        self.rtsp_thread = threading.Thread(target=self._receive_rtsp, daemon=True)
        self.rtsp_thread.start()
        self.zmq_thread = threading.Thread(target=self._receive_yolo, daemon=True)
        self.zmq_thread.start()

        rospy.Subscriber(STATUS_TOPIC, String, self._status_callback, queue_size=20)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "YOLO实时截图上传已启动: robot=%s classes=%d topic=%s interval=%.1fs",
            self.robot_code, len(self.items), self.outbox.topic, self.upload_interval,
        )

    def _status_callback(self, raw):
        try:
            message = json.loads(raw.data)
            state = str(message.get("state", "")).upper()
            with self.lock:
                self.task_active = state not in INACTIVE_STATES
                self.task_id = str(message.get("task_id", "")).strip()
                self.task_name = str(message.get("task_name", "")).strip()
                self.point_name = str(message.get("point_name", "")).strip()
                if not self.task_active:
                    self.last_upload.clear()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "任务状态解析失败: %s", exc)

    def _receive_rtsp(self):
        if cv2 is None:
            rospy.logerr("缺少python3-opencv，无法生成实时检测截图")
            return
        capture = None
        while not self.stopping and not rospy.is_shutdown():
            try:
                if capture is None or not capture.isOpened():
                    if capture is not None:
                        capture.release()
                    capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    if not capture.isOpened():
                        capture.release()
                        capture = None
                        rospy.logwarn_throttle(10.0, "截图RTSP连接失败，将继续重试")
                        time.sleep(2.0)
                        continue
                ok, frame = capture.read()
                if not ok or frame is None:
                    capture.release()
                    capture = None
                    time.sleep(0.5)
                    continue
                with self.lock:
                    self.latest_image = frame
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "截图RTSP读取失败: %s", exc)
                if capture is not None:
                    capture.release()
                    capture = None
                time.sleep(1.0)
        if capture is not None:
            capture.release()

    def _match_objects(self, frame):
        matched = []
        objects = frame.get("objects", []) if isinstance(frame, dict) else []
        if not isinstance(objects, list):
            return matched
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            try:
                confidence = float(obj.get("confidence", 0.0))
            except (TypeError, ValueError):
                continue
            if confidence < self.min_confidence:
                continue
            item = None
            supplied_name = False
            for field in ("class_name", "class_name_cn"):
                name = str(obj.get(field, "")).strip().lower()
                supplied_name = supplied_name or bool(name)
                if name in self.by_name:
                    item = self.by_name[name]
                    break
            if item is None and not supplied_name:
                try:
                    item = self.by_id.get(int(obj.get("class_id", -1)))
                except (TypeError, ValueError):
                    pass
            if item is None:
                continue
            value = dict(obj)
            value["canonical_name"] = item["name"]
            value["display_name"] = item["display_name"]
            matched.append(value)
        return matched

    def _receive_yolo(self):
        if zmq is None:
            rospy.logerr("缺少python3-zmq，无法接收YOLO检测结果")
            return
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.setsockopt(zmq.RCVTIMEO, 500)
        socket.connect(self.zmq_address)
        try:
            while not self.stopping and not rospy.is_shutdown():
                try:
                    frame = socket.recv_json()
                except zmq.Again:
                    continue
                except Exception as exc:
                    rospy.logwarn_throttle(5.0, "YOLO ZMQ接收失败: %s", exc)
                    continue
                matched = self._match_objects(frame)
                if matched:
                    self._handle_detection(matched)
        finally:
            socket.close(linger=0)
            context.term()

    def _handle_detection(self, objects):
        with self.lock:
            if not self.task_active or not self.task_name:
                return
            if self.latest_image is None:
                rospy.logwarn_throttle(5.0, "检测到项目，但尚未取得RTSP截图")
                return
            class_signature = tuple(sorted(set(
                str(obj["canonical_name"]) for obj in objects)))
            key = (self.task_id or self.task_name, self.point_name, class_signature)
            current = time.monotonic()
            if current - self.last_upload.get(key, -1e9) < self.upload_interval:
                return
            self.last_upload[key] = current
            image = self.latest_image.copy()
            task_id = self.task_id
            task_name = self.task_name
            point_name = self.point_name or "未定位点位"
        try:
            self._save_and_enqueue(image, objects, task_id, task_name, point_name)
        except Exception as exc:
            with self.lock:
                self.last_upload.pop(key, None)
            rospy.logerr("实时检测截图保存上传失败: %s", exc)

    def _annotate(self, image, objects):
        height, width = image.shape[:2]
        for obj in objects:
            bbox = obj.get("bbox", [])
            if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
                continue
            try:
                x1, y1, x2, y2 = [int(float(value)) for value in bbox[:4]]
            except (TypeError, ValueError):
                continue
            x1, x2 = sorted((max(0, min(width - 1, x1)), max(0, min(width - 1, x2))))
            y1, y2 = sorted((max(0, min(height - 1, y1)), max(0, min(height - 1, y2))))
            label = "{} {:.2f}".format(
                obj.get("canonical_name", "object"),
                float(obj.get("confidence", 0.0)),
            )
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.putText(
                image, label, (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2, cv2.LINE_AA,
            )
        if width > self.image_max_width:
            scale = float(self.image_max_width) / float(width)
            image = cv2.resize(
                image, (self.image_max_width, max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        return image

    def _save_and_enqueue(self, image, objects, task_id, task_name, point_name):
        stamp = now_ms()
        task_dir = os.path.join(
            self.results_root,
            safe_filename(task_name, "task") + "_" + safe_filename(task_id or "no-id"),
        )
        os.makedirs(task_dir, exist_ok=True)
        stem = "{}_{}".format(safe_filename(point_name, "point"), stamp)
        image_path = os.path.join(task_dir, stem + ".jpg")
        result_path = os.path.join(task_dir, stem + ".result.json")

        annotated = self._annotate(image, objects)
        ok, encoded = cv2.imencode(
            ".jpg", annotated,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("JPEG编码失败")
        atomic_write_bytes(image_path, encoded.tobytes())

        counts = {item["name"]: 0 for item in self.items}
        for obj in objects:
            name = str(obj.get("canonical_name", ""))
            if name in counts:
                counts[name] += 1
        payload = build_event_payload(
            self.robot_code, task_id, task_name, point_name,
            self.items, counts, 1, "REALTIME_DETECTED", timestamp_ms=stamp,
        )
        record = {
            "payload": payload,
            "imageFile": os.path.basename(image_path),
            "savedMs": stamp,
        }
        atomic_write_json(result_path, record)
        upload_payload = payload_with_image(payload, image_path)
        if not self.outbox.enqueue(
            upload_payload,
            cleanup_paths=[result_path, image_path],
            cleanup_dir=task_dir,
        ):
            raise RuntimeError("MQTT待发队列已满")
        rospy.loginfo(
            "检测项目截图已进入上传队列: task=%s point=%s objects=%d",
            task_name, point_name, len(objects),
        )

    def _recover_pending_results(self):
        for root, _directories, files in os.walk(self.results_root):
            for name in sorted(files):
                if not name.endswith(".result.json"):
                    continue
                result_path = os.path.join(root, name)
                try:
                    with open(result_path, "r", encoding="utf-8") as stream:
                        record = json.load(stream)
                    payload = record.get("payload")
                    image_name = os.path.basename(str(record.get("imageFile", "")))
                    image_path = os.path.join(root, image_name)
                    if not isinstance(payload, dict) or not os.path.isfile(image_path):
                        continue
                    if self.outbox.enqueue(
                        payload_with_image(payload, image_path),
                        cleanup_paths=[result_path, image_path],
                        cleanup_dir=root,
                    ):
                        rospy.loginfo("恢复未确认的实时截图: %s", image_path)
                except Exception as exc:
                    rospy.logwarn("恢复实时截图失败 %s: %s", result_path, exc)

    def shutdown(self):
        self.stopping = True
        self.outbox.stop()
        self.zmq_thread.join(timeout=1.0)
        self.rtsp_thread.join(timeout=1.0)
        self.outbox_thread.join(timeout=1.0)


def create_parser():
    parser = argparse.ArgumentParser(description="YOLO检测项目实时截图MQTT上传")
    parser.add_argument("--robot-code", default="")
    parser.add_argument("--mqtt-config", default=DEFAULT_MQTT_CONFIG)
    parser.add_argument("--mqtt-credentials", default=DEFAULT_MQTT_CREDENTIALS)
    parser.add_argument("--mqtt-host", default="")
    parser.add_argument("--mqtt-port", type=int, default=None)
    parser.add_argument("--mqtt-username", default="autocar")
    parser.add_argument("--mqtt-password", default="123456")
    parser.add_argument("--file-topic", default="")
    parser.add_argument("--device-config", default=DEFAULT_DEVICE_CONFIG)
    parser.add_argument("--yolo-config", default=DEFAULT_YOLO_CONFIG)
    parser.add_argument("--yolo-env", default=DEFAULT_YOLO_ENV)
    parser.add_argument("--detector-file", default=DEFAULT_DETECTOR_FILE)
    parser.add_argument("--zmq-address", default="")
    parser.add_argument("--rtsp-url", default="")
    parser.add_argument("--results-root", default=DEFAULT_RESULTS_ROOT)
    parser.add_argument("--class-profile", default="auto")
    parser.add_argument("--detection-items", default="")
    parser.add_argument("--min-confidence", type=float, default=None)
    parser.add_argument("--upload-interval", type=float, default=3.0)
    parser.add_argument("--image-max-width", type=int, default=1280)
    parser.add_argument("--jpeg-quality", type=int, default=82)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser


def self_test(args):
    items = load_detection_items(
        args.detector_file, args.class_profile, args.yolo_env, args.detection_items)
    payload = build_event_payload(
        "DT202600001", "TASK-1", "任务1", "巡航点01",
        items, {items[0]["name"]: 1}, 1, "REALTIME_DETECTED", 1700000000000,
    )
    assert payload["method"] == "file_upload_request"
    assert payload["data"]["detectionItems"][0]["count"] == 1
    assert payload["data"]["stationName"] == "任务1"
    print("SELF_TEST_OK classes={}".format(len(items)))


def main():
    parser = create_parser()
    arguments = rospy.myargv(argv=sys.argv)[1:] if rospy is not None else sys.argv[1:]
    args = parser.parse_args(arguments)
    if args.self_test:
        self_test(args)
        return 0
    if rospy is None or String is None:
        parser.error("运行需要ROS Noetic的rospy和std_msgs")
    if cv2 is None:
        parser.error("运行需要python3-opencv")
    rospy.init_node("mqtt_yolo_realtime_snapshot_uploader")
    RealtimeSnapshotUploader(args)
    rospy.spin()
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda _signum, _frame: sys.exit(0))
    sys.exit(main())

