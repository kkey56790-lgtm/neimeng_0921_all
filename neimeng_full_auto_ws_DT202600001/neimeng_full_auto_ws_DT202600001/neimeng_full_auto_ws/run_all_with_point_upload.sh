#!/usr/bin/env bash
set -Eeuo pipefail

# 独立的一键启动入口：原有全流程 + jieguo启动补传/任务结束上传。
# 不修改 run_ubuntu20_all.sh 和任何原有 ROS/YOLO 源代码。

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 可选本地配置文件（Bash语法）；不存在时沿用原有默认值。
MQTT_ENV_FILE="${MQTT_ENV_FILE:-$WORKSPACE_DIR/quick_cruise.env}"
if [[ -f "$MQTT_ENV_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$MQTT_ENV_FILE"
  set +a
fi
BACKEND="${1:-inspection11}"
ROBOT_CODE="${ROBOT_CODE:-DT202600001}"
ROBOT_MQTT_HOST="${ROBOT_MQTT_HOST:-222.187.130.102}"
ROBOT_MQTT_PORT="${ROBOT_MQTT_PORT:-1883}"
DEVEL_DIR="${NEIMENG_DEVEL_DIR:-$WORKSPACE_DIR/devel_ubuntu20}"
shift || true

MAIN_LAUNCHER="$WORKSPACE_DIR/run_ubuntu20_all.sh"
UPLOADER="$WORKSPACE_DIR/independent_systems/yolo_upload_system/scripts/mqtt_yolo_point_uploader.py"
UPLOAD_HELPER="$WORKSPACE_DIR/independent_systems/yolo_upload_system/scripts/jieguo_uploader.py"
CRUISE_RECEIVER="$WORKSPACE_DIR/independent_systems/yolo_upload_system/scripts/mqtt_quick_cruise_receiver.py"
YOLO_ENV="$WORKSPACE_DIR/rknn_yolo/yolo.env"
DETECTOR_FILE="$WORKSPACE_DIR/rknn_yolo/rtsp_truck.py"
RESULTS_ROOT="$WORKSPACE_DIR/jieguo"
MAIN_BACKEND="$BACKEND"
case "$BACKEND" in
  inspection11|0919)
    UPLOAD_CLASS_PROFILE=inspection11
    MAIN_BACKEND=rknn
    INSPECTION_MODEL="${INSPECTION_RKNN_MODEL:-$WORKSPACE_DIR/rknn_yolo/models/0919-2.rknn}"
    # 下游可能切换工作目录，统一传递绝对路径。
    [[ "$INSPECTION_MODEL" = /* ]] || INSPECTION_MODEL="$PWD/$INSPECTION_MODEL"
    ;;
  yolov8) UPLOAD_CLASS_PROFILE=coco80 ;;
  *) UPLOAD_CLASS_PROFILE=auto ;;
esac
if [[ "$UPLOAD_CLASS_PROFILE" == "inspection11" && ! -f "${INSPECTION_RKNN_MODEL:-$WORKSPACE_DIR/rknn_yolo/models/0919-2.rknn}" ]]; then
  echo "缺少十一类模型: ${INSPECTION_RKNN_MODEL:-$WORKSPACE_DIR/rknn_yolo/models/0919-2.rknn}" >&2
  exit 1
fi

if [[ ! -f "$MAIN_LAUNCHER" ]]; then
  echo "找不到原有启动脚本: $MAIN_LAUNCHER" >&2
  exit 1
fi
if [[ ! -f "$UPLOADER" ]]; then
  echo "找不到巡航点上传程序: $UPLOADER" >&2
  exit 1
fi
if [[ ! -f "$UPLOAD_HELPER" ]]; then
  echo "缺少上传模块，请同步测试成功的文件: $UPLOAD_HELPER" >&2
  exit 1
fi

MAIN_PID=""
UPLOADER_PID=""
CRUISE_PID=""
INSPECTION_ENV_FILE=""
if [[ ! -f "$CRUISE_RECEIVER" ]]; then
  echo "缺少中台巡检接收程序: $CRUISE_RECEIVER" >&2
  exit 1
fi

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if [[ -n "$CRUISE_PID" ]] && kill -0 "$CRUISE_PID" 2>/dev/null; then
    echo "停止中台巡检接收程序(pid=$CRUISE_PID)..."
    kill -TERM "$CRUISE_PID" 2>/dev/null || true
    wait "$CRUISE_PID" 2>/dev/null || true
  fi

  if [[ -n "$UPLOADER_PID" ]] && kill -0 "$UPLOADER_PID" 2>/dev/null; then
    echo "停止巡航点上传程序(pid=$UPLOADER_PID)..."
    kill -TERM "$UPLOADER_PID" 2>/dev/null || true
    wait "$UPLOADER_PID" 2>/dev/null || true
  fi

  if [[ -n "$MAIN_PID" ]] && kill -0 "$MAIN_PID" 2>/dev/null; then
    echo "停止原有全流程(pid=$MAIN_PID)..."
    kill -TERM "$MAIN_PID" 2>/dev/null || true
    wait "$MAIN_PID" 2>/dev/null || true
  fi

  if [[ -n "$INSPECTION_ENV_FILE" ]]; then
    rm -f -- "$INSPECTION_ENV_FILE"
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

# 上传程序沿用当前python3（与单独测试时一致），加载ROS环境。
if [[ -r /opt/ros/noetic/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  set -u
fi
if [[ -r "$DEVEL_DIR/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "$DEVEL_DIR/setup.bash"
  set -u
fi

# ARM64上提前装载系统OpenMP库，避免导入OpenCV时静态TLS空间不足。
# 只作用于上传程序及其导入检查，不修改原YOLO/Conda进程的加载环境。
UPLOAD_ENV=(env)
if [[ "$(uname -m)" == "aarch64" ]]; then
  UPLOAD_GOMP=""
  for candidate in /lib/aarch64-linux-gnu/libgomp.so.1 /usr/lib/aarch64-linux-gnu/libgomp.so.1; do
    if [[ -r "$candidate" ]]; then
      UPLOAD_GOMP="$candidate"
      break
    fi
  done
  if [[ -n "$UPLOAD_GOMP" ]]; then
    UPLOAD_ENV+=("LD_PRELOAD=$UPLOAD_GOMP${LD_PRELOAD:+:$LD_PRELOAD}")
    echo "上传进程提前加载OpenMP库: $UPLOAD_GOMP"
  else
    echo "未找到系统libgomp.so.1；如出现静态TLS错误，请检查libgomp1安装。" >&2
  fi
fi

echo "检查上传依赖: $(command -v python3)"
"${UPLOAD_ENV[@]}" python3 -c 'import yaml, rospy, zmq; import paho.mqtt.client; import cv2, requests' || {
  echo "上传依赖导入失败，尚未启动全流程。请根据上方实际异常排查；TLS错误不代表缺少Python模块。" >&2
  exit 1
}

# 依赖检查通过后再启动硬件全流程。
OCR_ENABLED="${PLATE_OCR_ENABLED:-0}"
if [[ "${OCR_ENABLED,,}" =~ ^(1|true|yes)$ ]]; then
  echo "检查车牌OCR依赖并初始化识别模型（首次运行可能需要下载）..."
  "${UPLOAD_ENV[@]}" python3 -c 'import sys; sys.path.insert(0, sys.argv[1]); from plate_ocr import PlateOCR; PlateOCR.from_env(); print("车牌OCR模型就绪")' \
    "$WORKSPACE_DIR/independent_systems/yolo_upload_system/scripts" || {
    echo "车牌OCR初始化失败，尚未启动全流程。请在上传进程使用的Python环境中安装easyocr，并检查OCR模型文件；详见上方异常。" >&2
    exit 1
  }
fi
if [[ "$UPLOAD_CLASS_PROFILE" == "inspection11" ]]; then
  # run_yolo.sh 会 source 配置；在原配置之后固定模型和类别，避免被旧值覆盖。
  ORIGINAL_YOLO_ENV="${NEIMENG_YOLO_ENV:-$YOLO_ENV}"
  if [[ -z "${NEIMENG_YOLO_ENV:-}" && ! -f "$ORIGINAL_YOLO_ENV" && -f /etc/neimeng_xunjian/yolo.env ]]; then
    ORIGINAL_YOLO_ENV=/etc/neimeng_xunjian/yolo.env
  fi
  [[ "$ORIGINAL_YOLO_ENV" = /* ]] || ORIGINAL_YOLO_ENV="$PWD/$ORIGINAL_YOLO_ENV"
  INSPECTION_ENV_FILE="$(mktemp "${TMPDIR:-/tmp}/neimeng-inspection11.XXXXXX")"
  {
    if [[ -f "$ORIGINAL_YOLO_ENV" ]]; then
      printf 'source %q\n' "$ORIGINAL_YOLO_ENV"
    fi
    printf 'export RKNN_TRUCK_MODEL=%q\n' "$INSPECTION_MODEL"
    printf 'export INSPECTION_RKNN_MODEL=%q\n' "$INSPECTION_MODEL"
    printf 'export YOLO_CLASS_PROFILE=inspection11\n'
  } > "$INSPECTION_ENV_FILE"
  export NEIMENG_YOLO_ENV="$INSPECTION_ENV_FILE"
  export RKNN_TRUCK_MODEL="$INSPECTION_MODEL"
  export YOLO_CLASS_PROFILE=inspection11
fi
echo "启动原有全流程: backend=$MAIN_BACKEND profile=$UPLOAD_CLASS_PROFILE"
bash "$MAIN_LAUNCHER" "$MAIN_BACKEND" --inline "$@" &
MAIN_PID=$!

echo "等待ROS话题 /inspection/status（最长90秒）..."
ROS_READY=0
for _ in $(seq 1 90); do
  if ! kill -0 "$MAIN_PID" 2>/dev/null; then
    wait "$MAIN_PID" || true
    echo "原有全流程在上传程序启动前退出" >&2
    exit 1
  fi
  if command -v rostopic >/dev/null 2>&1 && \
     rostopic list 2>/dev/null | grep -qx '/inspection/status'; then
    ROS_READY=1
    break
  fi
  sleep 1
done

if [[ "$ROS_READY" -ne 1 ]]; then
  echo "90秒内未发现 /inspection/status，未启动上传程序" >&2
  exit 1
fi

# 避免重复节点导致“new node registered with same name”。
if command -v rosnode >/dev/null 2>&1 && \
   rosnode list 2>/dev/null | grep -qx '/mqtt_yolo_point_uploader'; then
  echo "停止旧的 /mqtt_yolo_point_uploader 节点..."
  rosnode kill /mqtt_yolo_point_uploader >/dev/null 2>&1 || true
  sleep 1
fi

echo "启动已验证的jieguo上传程序: robot=$ROBOT_CODE root=$RESULTS_ROOT"
echo "启动后补传已封存图片/TXT；未结束任务等待底盘状态续采，任务结束后统一上传。"
"${UPLOAD_ENV[@]}" python3 -u "$UPLOADER" \
  --robot-code "$ROBOT_CODE" \
  --mqtt-host "$ROBOT_MQTT_HOST" \
  --mqtt-port "$ROBOT_MQTT_PORT" \
  --file-topic "thing/robot/$ROBOT_CODE/file" \
  --yolo-env "$YOLO_ENV" \
  --detector-file "$DETECTOR_FILE" \
  --class-profile "$UPLOAD_CLASS_PROFILE" \
  --results-root "$RESULTS_ROOT" &
UPLOADER_PID=$!

CRUISE_ARGS=()
if [[ -n "${QUICK_CRUISE_TASK:-}" ]]; then
  CRUISE_ARGS+=(--task-name "$QUICK_CRUISE_TASK")
fi
echo "启动中台MQTT一键巡检接收程序: $ROBOT_MQTT_HOST:$ROBOT_MQTT_PORT"
echo "下发: thing/robot/$ROBOT_CODE/services；回执: thing/robot/$ROBOT_CODE/services_reply"
"${UPLOAD_ENV[@]}" python3 -u "$CRUISE_RECEIVER" \
  --host "$ROBOT_MQTT_HOST" --port "$ROBOT_MQTT_PORT" \
  --robot-code "$ROBOT_CODE" "${CRUISE_ARGS[@]}" &
CRUISE_PID=$!

echo "一键启动完成。按 Ctrl+C 可统一停止。"

# 任一关键进程退出，就停止另一进程并返回退出状态。
set +e
wait -n "$MAIN_PID" "$UPLOADER_PID" "$CRUISE_PID"
STATUS=$?
set -e
exit "$STATUS"
