#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${NEIMENG_BUILD_DIR:-${WORKSPACE_DIR}/build_ubuntu20}"
DEVEL_DIR="${NEIMENG_DEVEL_DIR:-${WORKSPACE_DIR}/devel_ubuntu20}"

if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
  echo "未找到 ROS Noetic。Ubuntu 20.04 请先安装 ros-noetic-desktop-full。" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash

if ! command -v catkin_make >/dev/null 2>&1; then
  echo "未找到 catkin_make，请安装 ros-noetic-catkin。" >&2
  exit 1
fi

if ! python3 -c 'import rospy, requests, PyQt5, yaml, zmq' >/dev/null 2>&1; then
  echo "Python 依赖缺失，请安装 python3-requests、python3-pyqt5、python3-yaml 和 python3-zmq。" >&2
  exit 1
fi

python3 -m compileall -q "${WORKSPACE_DIR}/src"
catkin_make   -C "${WORKSPACE_DIR}"   --source "${WORKSPACE_DIR}/src"   --build "${BUILD_DIR}"   -DCATKIN_DEVEL_PREFIX="${DEVEL_DIR}"   -DPYTHON_EXECUTABLE=/usr/bin/python3

source "${DEVEL_DIR}/setup.bash"
exec roslaunch neimeng_bringup full_auto_test.launch enable_ui:=true "$@"
