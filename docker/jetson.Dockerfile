# syntax=docker/dockerfile:1.7
#
# Onboard runtime image for the Jetson (aarch64, JetPack 7.2): Isaac ROS (VSLAM, nvblox, AprilTag),
# the micro-ROS agent, ardupilot_msgs, the OAK camera driver, and this repo's stack built in.
# Started by real/jetson/compose.yaml under systemd.
#
# UNTESTED SKELETON: no Jetson yet. Build on the Jetson itself (or an arm64 builder with the
# Jetson base image): docker build -f docker/jetson.Dockerfile --build-arg ROS_DISTRO=jazzy \
#   --build-arg ISAAC_ROS_BASE=<ISAAC_ROS_BASE_JETSON from docker/.env> -t coredrone-jetson .

ARG ROS_DISTRO
ARG ISAAC_ROS_BASE
FROM ${ISAAC_ROS_BASE}

ARG ROS_DISTRO
ARG ISAAC_ROS_RELEASE=4.6
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive PIP_BREAK_SYSTEM_PACKAGES=1

RUN test -f "/opt/ros/${ROS_DISTRO}/setup.bash" \
 || { echo "Isaac ROS base image is not ROS ${ROS_DISTRO}: fix docker/.env (docs/adr/0002)"; exit 1; }

# Isaac ROS for JetPack (the Jetson apt component is noble-jetpack), pinned to the release line.
RUN echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-${ISAAC_ROS_RELEASE} noble-jetpack main" \
      > /etc/apt/sources.list.d/nvidia-isaac-ros.list \
 && (test -f /usr/share/keyrings/nvidia-isaac-ros.gpg \
     || curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg) \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-isaac-ros-visual-slam" \
      "ros-${ROS_DISTRO}-isaac-ros-nvblox" \
      "ros-${ROS_DISTRO}-isaac-ros-apriltag" \
      "ros-${ROS_DISTRO}-depthai-ros-driver" \
      "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
      python3-vcstool git \
 && rm -rf /var/lib/apt/lists/*

# Underlay: only what the vehicle needs from deps/underlay.repos (no SITL, no Gazebo).
WORKDIR /opt/underlay
COPY deps/underlay.repos /opt/underlay/underlay.repos
RUN mkdir -p src && vcs import --shallow --input underlay.repos src \
 && touch src/ardupilot_gazebo/COLCON_IGNORE src/Micro-XRCE-DDS-Gen/COLCON_IGNORE \
 && apt-get update && (rosdep init 2>/dev/null || true) && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install -y -r --rosdistro "${ROS_DISTRO}" --ignore-src \
      --from-paths src/ardupilot/Tools/ros2/ardupilot_msgs src/micro_ros_agent --skip-keys microxrcedds_agent \
 && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --merge-install --packages-up-to ardupilot_msgs micro_ros_agent --cmake-args -DBUILD_TESTING=OFF \
 && rm -rf build log src/ardupilot src/ardupilot_gazebo src/Micro-XRCE-DDS-Gen \
 && rm -rf /var/lib/apt/lists/*

# The stack itself (built, not bind-mounted) plus the repo files it reads at runtime.
WORKDIR /opt/inv
COPY deps/rosdep.yaml /opt/inv/rosdep.yaml
COPY drone/src /opt/inv/drone/src
COPY config /opt/inv/config
COPY contracts /opt/inv/contracts
COPY real /opt/inv/real
COPY sim/launch /opt/inv/sim/launch
RUN echo "yaml file:///opt/inv/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/10-coredrone.list \
 && apt-get update && rosdep update --rosdistro "${ROS_DISTRO}" \
 && source /opt/underlay/install/setup.bash \
 && rosdep install -y --rosdistro "${ROS_DISTRO}" --ignore-src --from-paths drone/src \
      --skip-keys "python3-pymavlink ardupilot_msgs ros_gz_bridge ros_gz_sim" \
 && pip install pymavlink \
 && cd drone && colcon build --cmake-args -DBUILD_TESTING=OFF \
 && rm -rf build log /var/lib/apt/lists/*

ENV INV_ROOT=/opt/inv
COPY docker/scripts/env.sh /opt/inv/env.sh
COPY docker/scripts/entrypoint.sh /usr/local/bin/inv-entrypoint
RUN chmod 0755 /usr/local/bin/inv-entrypoint
ENTRYPOINT ["/usr/local/bin/inv-entrypoint"]
CMD ["bash"]
