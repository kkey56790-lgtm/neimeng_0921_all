#!/usr/bin/env bash
# Run in Linux with rknn-toolkit2 and onnx installed.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERTER="$ROOT/neimeng_full_auto_ws_DT202600001/neimeng_full_auto_ws_DT202600001/neimeng_full_auto_ws/rknn_yolo/onnx_to_rknn.py"
exec "${RKNN_CONVERT_PYTHON:-python3}" "$CONVERTER" "$ROOT/0919.onnx" \
  --output "$ROOT/0919.rknn" --target rk3588 --size 640 "$@"
