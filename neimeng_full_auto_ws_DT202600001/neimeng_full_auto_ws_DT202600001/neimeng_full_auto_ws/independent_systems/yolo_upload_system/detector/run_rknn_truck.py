#!/usr/bin/env python3
"""独立启动原10类truck RKNN；只发布检测JSON，不向UI发送视频。"""

import os

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
os.environ["RKNN_MODEL"] = os.environ.get(
    "RKNN_TRUCK_MODEL", os.path.join(SCRIPT_DIR, "models", "best_truck.rknn"))
os.environ["YOLO_CLASS_PROFILE"] = "truck10"
from rtsp_truck import main


if __name__ == "__main__":
    main()