#!/usr/bin/env bash
# Shared paths: no dependency on the author's drive, username or checkout name.
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)
WS=${COURSE_WS:-${XDG_DATA_HOME:-$HOME/.local/share}/swarm-exploration-course/ws}
DISPLAY_DIR="$ROOT/.runtime/display"
export COURSE_ROOT="$ROOT" COURSE_WS="$WS" COURSE_DISPLAY_DIR="$DISPLAY_DIR"
mkdir -p "$ROOT/setup/ros/logs" "$DISPLAY_DIR"
