#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA_ENV_NAME="${NEIMENG_CONDA_ENV:-rknn_yolo}"

find_conda_sh() {
  local candidate
  for candidate in \
    "${NEIMENG_CONDA_SH:-}" \
    "${HOME}/anaconda3/etc/profile.d/conda.sh" \
    "${HOME}/miniconda3/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh"; do
    if [[ -n "$candidate" && -r "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

activate_yolo_env() {
  local conda_sh
  conda_sh="$(find_conda_sh)" || {
    echo "未找到conda.sh；可设置 NEIMENG_CONDA_SH=/绝对路径/conda.sh" >&2
    return 1
  }
  # shellcheck disable=SC1090
  source "$conda_sh"
  conda activate "$CONDA_ENV_NAME"
  python3 -c 'import cv2, numpy, zmq; from rknnlite.api import RKNNLite' || {
    echo "Conda环境 $CONDA_ENV_NAME 缺少YOLO/RKNN依赖" >&2
    return 1
  }
}

# 子终端入口：只启动YOLO，确保模型进程使用Conda环境。
if [[ "${1:-}" == "--yolo-child" ]]; then
  YOLO_BACKEND="${2:-rknn}"
  activate_yolo_env
  echo "启动YOLO: backend=$YOLO_BACKEND, conda=$CONDA_ENV_NAME"
  exec bash "$WORKSPACE_DIR/rknn_yolo/run_yolo.sh" "$YOLO_BACKEND"
fi

# 子终端入口：ROS仍使用系统Python，避免和RKNN环境混用。
if [[ "${1:-}" == "--ros-child" ]]; then
  shift
  exec bash "$WORKSPACE_DIR/run_ubuntu20_full_auto.sh" "$@"
fi

YOLO_BACKEND="${1:-rknn}"
shift || true
LAUNCH_MODE="${NEIMENG_LAUNCH_MODE:-terminals}"
if [[ "${1:-}" == "--inline" ]]; then
  LAUNCH_MODE="inline"
  shift
fi

case "$YOLO_BACKEND" in
  yolov8|rknn|rknn_truck|custom) ;;
  *)
    echo "未知YOLO后端: $YOLO_BACKEND；可选 yolov8 或 rknn" >&2
    exit 2
    ;;
esac

open_terminal_pair() {
  local launcher_q backend_q arg_q ros_args=""
  local yolo_command ros_command
  printf -v launcher_q '%q' "$WORKSPACE_DIR/run_ubuntu20_all.sh"
  printf -v backend_q '%q' "$YOLO_BACKEND"
  for arg in "$@"; do
    printf -v arg_q '%q' "$arg"
    ros_args+=" $arg_q"
  done

  yolo_command="bash $launcher_q --yolo-child $backend_q; status=\$?; echo; echo YOLO进程已退出，状态码=\$status; exec bash"
  ros_command="bash $launcher_q --ros-child$ros_args; status=\$?; echo; echo ROS全流程已退出，状态码=\$status; exec bash"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="内蒙古巡检 - YOLO" -- bash -lc "$yolo_command"
    sleep 1
    gnome-terminal --title="内蒙古巡检 - ROS全流程" -- bash -lc "$ros_command"
    return 0
  fi
  if command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -T "内蒙古巡检 - YOLO" -e bash -lc "$yolo_command"
    sleep 1
    x-terminal-emulator -T "内蒙古巡检 - ROS全流程" -e bash -lc "$ros_command"
    return 0
  fi
  return 1
}

# Ubuntu桌面默认打开两个窗口；SSH或无桌面环境自动退回同一终端。
if [[ "$LAUNCH_MODE" == "terminals" && -n "${DISPLAY:-}" ]]; then
  if open_terminal_pair "$@"; then
    echo "已打开两个独立终端：YOLO检测、ROS全流程"
    exit 0
  fi
  echo "未找到可用终端程序，改为当前终端一体运行" >&2
fi

YOLO_PID=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$YOLO_PID" ]] && kill -0 "$YOLO_PID" 2>/dev/null; then
    echo "停止YOLO进程(pid=$YOLO_PID)..."
    kill -TERM "$YOLO_PID" 2>/dev/null || true
    wait "$YOLO_PID" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

activate_yolo_env
echo "启动YOLO: backend=$YOLO_BACKEND, conda=$CONDA_ENV_NAME"
bash "$WORKSPACE_DIR/rknn_yolo/run_yolo.sh" "$YOLO_BACKEND" &
YOLO_PID=$!

# YOLO后台进程保留激活后的环境；ROS恢复系统Python环境。
conda deactivate

sleep 2
if ! kill -0 "$YOLO_PID" 2>/dev/null; then
  wait "$YOLO_PID" || true
  echo "YOLO启动失败，请检查上方模型、RTSP或ZMQ端口报错" >&2
  exit 1
fi

echo "启动ROS全流程：底盘、云台、编排器、结果存储、轻量UI"
bash "$WORKSPACE_DIR/run_ubuntu20_full_auto.sh" "$@"
