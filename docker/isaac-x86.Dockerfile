# syntax=docker/dockerfile:1.7
#
# Isaac ROS image for x86_64 + NVIDIA RTX: Isaac ROS Visual SLAM, nvblox and AprilTag (GPU) plus
# this repo's overlay dependencies. Runs next to the dev container (compose `isaac` service shares
# its network), so Gazebo/SITL stay in the dev image and VSLAM consumes the simulated cameras.
#
# UNTESTED in this repo: needs CUDA, never built in GitHub-hosted CI. Manual GPU check:
# docs/onboarding/linux-nvidia.md. Base image tag: docker/.env (ISAAC_ROS_BASE_X86), UNVERIFIED.

ARG ROS_DISTRO
ARG ISAAC_ROS_BASE
FROM ${ISAAC_ROS_BASE}

ARG ROS_DISTRO
ARG ISAAC_ROS_RELEASE=4.6
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive PIP_BREAK_SYSTEM_PACKAGES=1

# The single distro pin (docker/.env) must match what this Isaac ROS release ships.
RUN test -f "/opt/ros/${ROS_DISTRO}/setup.bash" \
 || { echo "Isaac ROS base image is not ROS ${ROS_DISTRO}: fix docker/.env (docs/adr/0002)"; exit 1; }

# Isaac ROS packages, pinned to the release line (the base image tracks release-4 otherwise).
RUN echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-${ISAAC_ROS_RELEASE} noble main" \
      > /etc/apt/sources.list.d/nvidia-isaac-ros.list \
 && (test -f /usr/share/keyrings/nvidia-isaac-ros.gpg \
     || curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg) \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-isaac-ros-visual-slam" \
      "ros-${ROS_DISTRO}-isaac-ros-nvblox" \
      "ros-${ROS_DISTRO}-isaac-ros-apriltag" \
      "ros-${ROS_DISTRO}-foxglove-bridge" \
 && rm -rf /var/lib/apt/lists/*

# Overlay dependencies from package.xml (same rosdep path as the dev image).
COPY deps/rosdep.yaml /opt/inv/rosdep.yaml
COPY drone/src /tmp/overlay-src
RUN echo "yaml file:///opt/inv/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/10-coredrone.list \
 && apt-get update && (rosdep init 2>/dev/null || true) && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install -y --rosdistro "${ROS_DISTRO}" --ignore-src --from-paths /tmp/overlay-src \
      --skip-keys "python3-pymavlink ardupilot_msgs" \
 && pip install pymavlink \
 && rm -rf /var/lib/apt/lists/* /tmp/overlay-src

ENV INV_ROOT=/ws
WORKDIR /ws
