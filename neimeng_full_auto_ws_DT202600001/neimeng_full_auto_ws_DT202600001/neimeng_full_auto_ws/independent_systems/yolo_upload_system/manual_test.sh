#!/usr/bin/env bash
set -Eeuo pipefail
MODULE_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$MODULE_DIR/../.." && pwd)"
DEVEL_DIR="$WORKSPACE_DIR/devel_ubuntu20"
source /opt/ros/noetic/setup.bash
source "$DEVEL_DIR/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/independent_systems:$ROS_PACKAGE_PATH"
chmod +x "$MODULE_DIR/scripts/manual_detection_trigger.py"
exec rosrun yolo_upload_system manual_detection_trigger.py "$@"
