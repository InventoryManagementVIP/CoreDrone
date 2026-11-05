# CoreDrone task runner. `just` lists every recipe.
#
# On the host, recipes run inside the dev container (docker compose exec), so the only things
# you need installed are Docker and just. Inside the container (`just shell`) the same recipes
# run directly. Windows: run just from a WSL2 Ubuntu shell (docs/onboarding/windows-wsl2.md).

set shell := ["bash", "-euo", "pipefail", "-c"]
set export := true

# Git Bash would rewrite /ws-style paths; harmless everywhere else.
MSYS_NO_PATHCONV := "1"

in_container := env("INV_IN_CONTAINER", "0")

# Linux and WSLg hosts get Gazebo/RViz windows; macOS is always headless (use Foxglove).
has_display := if os() == "macos" { "false" } else if env("DISPLAY", "") != "" { "true" } else if env("WAYLAND_DISPLAY", "") != "" { "true" } else { "false" }
gui_files := if has_display == "true" { " -f docker/compose.gui.yaml" } else { "" }
wslg_file := if path_exists("/mnt/wslg") == "true" { " -f docker/compose.wslg.yaml" } else { "" }
dri_file := if path_exists("/dev/dri") == "true" { " -f docker/compose.dri.yaml" } else { "" }
compose := "docker compose -f docker/compose.yaml" + gui_files + wslg_file + dri_file
headless := if has_display == "true" { "false" } else { "true" }

# List recipes
default:
    @just --list --unsorted

# First-time setup: get the dev image (pull, or build if not published), start it, add git hooks
setup:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ "{{in_container}}" = "1" ]; then echo "run 'just setup' on the host, not in the container"; exit 1; fi
    {{compose}} pull dev || { echo "no published image yet: building it locally (20-40 min, once)"; {{compose}} build dev; }
    {{compose}} up -d dev
    if command -v pre-commit >/dev/null; then pre-commit install; else echo "tip: 'pipx install pre-commit' on the host to lint on every commit ('just lint' works without it)"; fi
    echo "ready: 'just build' then 'just test'"

# Start the dev container (idempotent)
up:
    {{compose}} up -d dev

# Stop every CoreDrone container
down:
    {{compose}} --profile sim --profile foxglove down

# Shell in the dev container
shell: up
    {{compose}} exec dev bash

# Build the dev image locally instead of pulling it
image:
    {{compose}} build dev

# colcon build the overlay (drone/) with --symlink-install
build:
    just _run "cd drone && colcon build --symlink-install"

# Run every test: package tests (colcon) + world generator + evaluation + contracts
test: test-packages test-sim

# colcon test for every package in drone/src, with a summary
test-packages:
    just _run "cd drone && colcon test --return-code-on-test-failure; rc=\$?; colcon test-result --verbose; exit \$rc"

# World generator, evaluation and contract tests (plain pytest, no simulator)
test-sim:
    just _run "python3 -m pytest -q sim/worldgen/tests sim/evaluation/tests && python3 contracts/validate.py"

# Lint everything (pre-commit: ruff, clang-format, yamllint, xmllint, large files, secrets)
lint:
    just _run "pre-commit run --all-files --show-diff-on-failure"

# Generate a warehouse world from its layout: just worldgen W=sim_small [SENSORS=label|none|full]
worldgen W="sim_small" SENSORS="label":
    just _run "cd sim && python3 -m worldgen --warehouse {{trim_start_match(W, 'W=')}} --sensors {{trim_start_match(SENSORS, 'SENSORS=')}}"

# Full simulation + stack (GUI where available, else headless + Foxglove on ws://localhost:8765)
sim W="sim_small" SENSORS="label":
    just worldgen {{W}} {{SENSORS}}
    just _run "ros2 launch inv_bringup drone.launch.py platform:=sim localization:=groundtruth mapping:=none warehouse:={{trim_start_match(W, 'W=')}} sensors:={{trim_start_match(SENSORS, 'SENSORS=')}} headless:={{headless}} foxglove:=true"

# Same as sim, never opens a window (CI, macOS, SSH)
sim-headless W="sim_small" SENSORS="label":
    just worldgen {{W}} {{SENSORS}}
    just _run "ros2 launch inv_bringup drone.launch.py platform:=sim localization:=groundtruth mapping:=none warehouse:={{trim_start_match(W, 'W=')}} sensors:={{trim_start_match(SENSORS, 'SENSORS=')}} headless:=true foxglove:=true"

# SITL smoke test (CI): arm -> take off 1.5 m -> hover 10 s -> land, through inv_flight_interface
sim-smoke:
    just _run "timeout 900 launch_test sim/tests/test_sim_smoke.py"

# Fly the scan mission on a generated world, then score it against ground truth
sim-mission W="sim_small":
    just _run "timeout 6000 launch_test sim/tests/test_sim_mission.py warehouse:={{trim_start_match(W, 'W=')}}"

# Check that every parameter in the committed .parm files exists in the pinned ArduPilot (SITL)
params-check:
    just _run "bash sim/tests/check_param_names.sh"

# Build ArduCopter with DDS for the flight controller; flash runs/firmware/arducopter.apj
fc-firmware:
    just _run "bash real/flight_controller/build_firmware.sh"

# Compare a live flight controller with the committed .parm files: just params-diff CONNECTION=udpin:0.0.0.0:14551
params-diff CONNECTION="udpin:0.0.0.0:14551" PLATFORM="real":
    just _run "ros2 run inv_tools params_diff --connection {{trim_start_match(CONNECTION, 'CONNECTION=')}} --platform {{trim_start_match(PLATFORM, 'PLATFORM=')}}"

# Start foxglove_bridge (ws://localhost:8765) for an already running stack
foxglove:
    just _run "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0"

# Record an MCAP bag with a topic profile: minimal | scan | sensors | sim
record PROFILE="minimal":
    just _run "ros2 run inv_tools record --profile {{trim_start_match(PROFILE, 'PROFILE=')}}"

# Delete colcon build/install/log volumes (next `just build` starts clean)
clean:
    {{compose}} down
    docker volume rm -f coredrone_colcon_build coredrone_colcon_install coredrone_colcon_log

# Run a command in the dev environment (container from the host, or directly inside it)
[private]
_run cmd:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ "{{in_container}}" = "1" ]; then
        cd "${INV_ROOT:-/ws}"
        exec inv-entrypoint bash -c {{quote(cmd)}}
    fi
    {{compose}} up -d dev >/dev/null
    tty=-T
    if [ -t 0 ] && [ -t 1 ]; then tty=""; fi
    exec {{compose}} exec $tty dev inv-entrypoint bash -c {{quote(cmd)}}
