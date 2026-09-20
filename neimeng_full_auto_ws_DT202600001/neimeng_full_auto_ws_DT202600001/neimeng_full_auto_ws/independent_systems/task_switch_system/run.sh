#!/usr/bin/env bash
set -Eeuo pipefail

MODULE_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$MODULE_DIR/../.." && pwd)"
BUILD_DIR="$WORKSPACE_DIR/build_ubuntu20"
DEVEL_DIR="$WORKSPACE_DIR/devel_ubuntu20"

source /opt/ros/noetic/setup.bash
if [[ ! -r "$DEVEL_DIR/setup.bash" ]]; then
  catkin_make -C "$WORKSPACE_DIR" --source "$WORKSPACE_DIR/src" --build "$BUILD_DIR" \
    -DCATKIN_DEVEL_PREFIX="$DEVEL_DIR" -DPYTHON_EXECUTABLE=/usr/bin/python3
fi
source "$DEVEL_DIR/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/independent_systems:$ROS_PACKAGE_PATH"
chmod +x "$MODULE_DIR"/scripts/*.py
python3 -m compileall -q "$MODULE_DIR/scripts"
exec roslaunch task_switch_system task_switch.launch "$@"
