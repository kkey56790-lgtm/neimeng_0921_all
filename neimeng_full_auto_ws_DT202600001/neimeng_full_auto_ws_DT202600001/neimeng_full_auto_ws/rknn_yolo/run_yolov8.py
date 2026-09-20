#!/usr/bin/env python3
"""独立启动标准YOLOv8 COCO RKNN；只发布检测JSON，不向UI发送视频。"""

import os

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
os.environ["RKNN_MODEL"] = os.environ.get(
    "YOLOV8_RKNN_MODEL", os.path.join(SCRIPT_DIR, "models", "yolov8n.rknn"))
os.environ["YOLO_CLASS_PROFILE"] = "coco80"
from rtsp_truck import main


if __name__ == "__main__":
    main()