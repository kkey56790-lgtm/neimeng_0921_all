#!/usr/bin/env bash
set -Eeuo pipefail
MODULE_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$MODULE_DIR/../.." && pwd)"
exec bash "$WORKSPACE_DIR/run_ubuntu20_all.sh" "$@"
