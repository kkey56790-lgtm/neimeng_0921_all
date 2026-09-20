#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_ENV_FILE="${SCRIPT_DIR}/yolo.env"
SYSTEM_ENV_FILE="/etc/neimeng_xunjian/yolo.env"

# 优先使用项目内配置，方便现场修改；未创建时才兼容旧的系统全局配置。
ENV_FILE="${NEIMENG_YOLO_ENV:-$LOCAL_ENV_FILE}"
if [[ -z "${NEIMENG_YOLO_ENV:-}" && ! -f "$ENV_FILE" && -f "$SYSTEM_ENV_FILE" ]]; then
  ENV_FILE="$SYSTEM_ENV_FILE"
fi

if [[ -f "$ENV_FILE" ]]; then
  set -a
  source "$ENV_FILE"
  set +a
fi
PYTHON_BIN="${RKNN_PYTHON:-python3}"
BACKEND="${1:-${YOLO_BACKEND:-yolov8}}"

case "$BACKEND" in
  yolov8)
    exec "$PYTHON_BIN" "$SCRIPT_DIR/run_yolov8.py"
    ;;
  rknn|rknn_truck|custom)
    exec "$PYTHON_BIN" "$SCRIPT_DIR/run_rknn_truck.py"
    ;;
  *)
    echo "未知识别器: $BACKEND；可选 yolov8 或 rknn" >&2
    exit 2
    ;;
esac
