#!/usr/bin/env bash
set -Eeuo pipefail
MODULE_DIR="$(cd "$(dirname "$0")" && pwd)"
exec bash "$MODULE_DIR/run.sh" rknn "$@"
