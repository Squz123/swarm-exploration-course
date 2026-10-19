#!/usr/bin/env bash
set -euo pipefail
export COURSE_PLANNER=online
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_trial.sh" ONLINE "${1:-$(date +%Y%m%d_%H%M%S)_ONLINE}"
