#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/fast-defaults.sh"
export COURSE_PLANNER=fast
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_trial.sh" B "${1:-$(date +%Y%m%d_%H%M%S)_FAST}"
