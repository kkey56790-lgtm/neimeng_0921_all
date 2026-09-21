#!/usr/bin/env python3
"""按配置选择RKNN模型与类别表；不再强制覆盖成十类。"""

import os

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
os.environ["RKNN_MODEL"] = os.environ.get(
    "RKNN_TRUCK_MODEL", os.path.join(SCRIPT_DIR, "models", "best_truck.rknn"))
profile = os.environ.get("YOLO_CLASS_PROFILE", "truck10").strip().lower()
if profile not in ("truck10", "inspection11"):
    raise ValueError("RKNN类别配置必须为truck10或inspection11，实际为: " + profile)
os.environ["YOLO_CLASS_PROFILE"] = profile
print("RKNN启动: source={} model={} profile={}".format(
    os.path.abspath(__file__), os.environ["RKNN_MODEL"], profile), flush=True)
from rtsp_truck import main


if __name__ == "__main__":
    main()
