# Environment for every shell and process in the CoreDrone images.
# Sourced by /usr/local/bin/inv-entrypoint and by ~/.bashrc. Not executable on purpose.
# shellcheck shell=bash disable=SC1090,SC1091

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Underlay: ArduPilot SITL, ardupilot_gazebo, micro-ROS agent (built into the image).
if [ -f /opt/underlay/install/setup.bash ]; then
  source /opt/underlay/install/setup.bash
fi

# Overlay: this repo's drone/ workspace, once `just build` has run.
if [ -f "${INV_ROOT:-/ws}/drone/install/setup.bash" ]; then
  source "${INV_ROOT:-/ws}/drone/install/setup.bash"
fi

export PATH="/opt/underlay/src/Micro-XRCE-DDS-Gen/scripts:${HOME}/.local/bin:${PATH}"
