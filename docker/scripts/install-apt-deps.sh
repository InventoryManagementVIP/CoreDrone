#!/usr/bin/env bash
# System, ROS 2 and Gazebo Harmonic packages for docker/dev.Dockerfile.
#
# Only the platform/tooling layer lives here. Packages that the overlay (drone/src) needs
# are declared in each package.xml and installed by rosdep in a later Dockerfile step.
set -euo pipefail
: "${ROS_DISTRO:?ROS_DISTRO must be set (see docker/.env)}"

apt-get update

common=(
  # Build tooling
  build-essential ccache cmake git git-lfs curl wget gawk unzip sudo less vim-tiny gnupg lsb-release
  # Python tooling
  python3-pip python3-dev python3-venv python3-pexpect python3-ptyprocess python3-serial
  python3-lxml python3-numpy python3-pyparsing python3-psutil python3-yaml
  python3-pytest python3-pytest-timeout python3-jsonschema python3-opencv python3-pil python3-qrcode
  # ArduPilot SITL build, from Tools/environment_install/install-prereqs-ubuntu.sh at the pinned tag
  libtool libxml2-dev libxslt1-dev libxml2-utils python3-empy
  # Micro-XRCE-DDS-Gen is built with gradle and needs a JDK
  openjdk-17-jdk-headless
  # ardupilot_gazebo plugin build (its package.xml build_depends)
  rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
  # Headless rendering for Gazebo camera sensors: Mesa llvmpipe through EGL, no GPU needed
  libegl1 libgl1-mesa-dri libegl-mesa0 mesa-utils
  # Network debugging
  iproute2 iputils-ping netcat-openbsd
  # ROS 2 tooling used by the platform layer and by developers
  "ros-${ROS_DISTRO}-xacro"
  "ros-${ROS_DISTRO}-robot-state-publisher"
  "ros-${ROS_DISTRO}-tf2-ros"
  "ros-${ROS_DISTRO}-tf2-tools"
  "ros-${ROS_DISTRO}-foxglove-bridge"
  "ros-${ROS_DISTRO}-rosbag2-storage-mcap"
  "ros-${ROS_DISTRO}-rviz2"
  "ros-${ROS_DISTRO}-launch-testing-ros"
  "ros-${ROS_DISTRO}-launch-testing-ament-cmake"
  "ros-${ROS_DISTRO}-ament-cmake-pytest"
  "ros-${ROS_DISTRO}-geographic-msgs"
)

case "${ROS_DISTRO}" in
  jazzy)
    # Gazebo Harmonic is Jazzy's default pairing and ships as ROS vendor packages.
    distro_pkgs=(ros-jazzy-ros-gz python3-zxing-cpp)
    ;;
  humble)
    # Non-default pairing: Harmonic comes from packages.osrfoundation.org.
    # This branch is kept so ROS_DISTRO stays a one-line switch; CI does not exercise it.
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list
    apt-get update
    # python3-zxing-cpp is not packaged for Ubuntu 22.04; the Dockerfile pip-installs it instead.
    distro_pkgs=(gz-harmonic ros-humble-ros-gzharmonic)
    ;;
  *)
    echo "Unsupported ROS_DISTRO='${ROS_DISTRO}'. See docs/adr/0002-ros-distro.md." >&2
    exit 1
    ;;
esac

apt-get install -y --no-install-recommends "${common[@]}" "${distro_pkgs[@]}"
rm -rf /var/lib/apt/lists/*
