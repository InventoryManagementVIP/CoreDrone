#!/usr/bin/env bash
# Check that every parameter name in the committed .parm files exists in the pinned ArduPilot.
# Catches parameters renamed by an ArduPilot bump (Copter 4.7 renamed several, e.g.
# ARMING_CHECK -> ARMING_SKIPCHK). Runs SITL with its built-in physics, no Gazebo needed.
# Usage: just params-check   (CI runs it on every PR that touches params or deps)
set -euo pipefail

ROOT="${INV_ROOT:-$(cd "$(dirname "$0")/../.." && pwd)}"
PARAMS=(
  "$ROOT/real/flight_controller/params/common.parm"
  "$ROOT/sim/params/sim.parm"
  "$ROOT/real/flight_controller/params/real.parm"
)
COPTER_PARM="$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm"
RUN="$(mktemp -d)"
cd "$RUN"

# Load our files as defaults too: feature-gated groups (RNGFND1_*, VISO_*) only exist once their
# TYPE parameter is set.
DEFAULTS="$COPTER_PARM,$(IFS=,; echo "${PARAMS[*]}")"
arducopter --model quad --speedup 5 -I0 --wipe --home 40.4237,-86.9212,190,0 --defaults "$DEFAULTS" > sitl.log 2>&1 &
SITL=$!
trap 'kill "$SITL" 2>/dev/null || true; rm -rf "$RUN"' EXIT

ros2 run inv_tools params_diff --connection tcp:127.0.0.1:5760 --check-names --files "${PARAMS[@]}"
