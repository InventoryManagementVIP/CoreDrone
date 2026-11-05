#!/usr/bin/env bash
# Entrypoint for the CoreDrone images: set up the ROS/underlay/overlay environment, then run the command.
set -e
source /opt/inv/env.sh
exec "$@"
