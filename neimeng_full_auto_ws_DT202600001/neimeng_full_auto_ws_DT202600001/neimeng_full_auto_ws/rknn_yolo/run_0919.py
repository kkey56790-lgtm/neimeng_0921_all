#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 0919-2.rknn 的 11 类检测，向直播程序发布 ZMQ 检测结果。"""

import os

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
os.environ["RKNN_MODEL"] = os.environ.get(
    "INSPECTION_RKNN_MODEL", os.path.join(SCRIPT_DIR, "models", "0919-2.rknn"))
os.environ["YOLO_CLASS_PROFILE"] = "inspection11"
print("RKNN启动: source={} model={} profile=inspection11".format(
    os.path.abspath(__file__), os.environ["RKNN_MODEL"]), flush=True)

from rtsp_truck import main


if __name__ == "__main__":
    main()
