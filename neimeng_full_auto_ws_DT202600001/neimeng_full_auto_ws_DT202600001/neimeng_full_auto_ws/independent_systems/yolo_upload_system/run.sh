#!/usr/bin/env bash
set -Eeuo pipefail

MODULE_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$MODULE_DIR/../.." && pwd)"
BUILD_DIR="$WORKSPACE_DIR/build_ubuntu20"
DEVEL_DIR="$WORKSPACE_DIR/devel_ubuntu20"
BACKEND="rknn"
if [[ $# -gt 0 ]]; then
  BACKEND="$1"
  shift
fi

case "$BACKEND" in
  yolov8|rknn|rknn_truck|custom) ;;
  *) echo "可选后端: yolov8 或 rknn" >&2; exit 2 ;;
esac
if [[ "$BACKEND" == "yolov8" && ! -f "$MODULE_DIR/detector/models/yolov8n.rknn" ]]; then
  echo "缺少 detector/models/yolov8n.rknn，请放入模型后再启动YOLOv8" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
if [[ ! -r "$DEVEL_DIR/setup.bash" ]]; then
  catkin_make -C "$WORKSPACE_DIR" --source "$WORKSPACE_DIR/src" --build "$BUILD_DIR" \
    -DCATKIN_DEVEL_PREFIX="$DEVEL_DIR" -DPYTHON_EXECUTABLE=/usr/bin/python3
fi
source "$DEVEL_DIR/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/independent_systems:$ROS_PACKAGE_PATH"
chmod +x "$MODULE_DIR"/scripts/*.py "$MODULE_DIR/detector/run_yolo.sh"
python3 -m compileall -q "$MODULE_DIR/scripts"

roslaunch yolo_upload_system yolo_upload.launch "$@" &
BRIDGE_PID=$!
cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill -TERM "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
  exit "$code"
}
trap cleanup EXIT INT TERM
sleep 2
kill -0 "$BRIDGE_PID" 2>/dev/null || { echo "ROS YOLO桥启动失败" >&2; exit 1; }

CONDA_SH="$HOME/anaconda3/etc/profile.d/conda.sh"
if [[ ! -r "$CONDA_SH" ]]; then
  CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
fi
if [[ ! -r "$CONDA_SH" ]]; then
  echo "未找到conda.sh，请设置标准anaconda3或miniconda3环境" >&2
  exit 1
fi
source "$CONDA_SH"
conda activate rknn_yolo
python3 -c 'import cv2, numpy, zmq; from rknnlite.api import RKNNLite'

export YOLOV8_RKNN_MODEL="$MODULE_DIR/detector/models/yolov8n.rknn"
export RKNN_TRUCK_MODEL="$MODULE_DIR/detector/models/best_truck.rknn"
NEIMENG_YOLO_ENV="$MODULE_DIR/config/yolo.env" \
  bash "$MODULE_DIR/detector/run_yolo.sh" "$BACKEND"
