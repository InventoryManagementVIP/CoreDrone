# syntax=docker/dockerfile:1.7
#
# CoreDrone `dev` image: the one image for development, simulation and CI.
#
#   ROS 2 (${ROS_DISTRO}) + Gazebo Harmonic
#   + ArduPilot Copter SITL built with AP_DDS + ardupilot_gazebo plugin + micro-ROS agent
#   + every apt/rosdep dependency of the overlay in drone/src
#
# Multi-arch: linux/amd64 and linux/arm64 (Apple Silicon). CI publishes it to GHCR
# (.github/workflows/docker-publish.yml), so students pull it instead of compiling ArduPilot.
#
# The underlay is built once, here, from deps/underlay.repos. This repo is the overlay and is
# bind-mounted at /ws at runtime (docker/compose.yaml); its colcon build/install/log
# directories live in named volumes.
#
# ROS_DISTRO has no default on purpose. Its single source of truth is docker/.env, which
# `just`, `docker compose` and CI all read. See docs/adr/0002-ros-distro.md.

ARG ROS_DISTRO

# =============================================================================================
# Stage `underlay`: platform packages + ArduPilot SITL + ardupilot_gazebo + micro-ROS agent.
# Depends only on docker/ and deps/, so overlay edits never trigger an ArduPilot rebuild.
FROM ros:${ROS_DISTRO}-ros-base AS underlay

ARG ROS_DISTRO
ARG TARGETARCH
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV ROS_DISTRO=${ROS_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    GZ_VERSION=harmonic

# 1. Platform packages: build tooling, ROS 2 tooling, Gazebo Harmonic.
COPY docker/scripts/install-apt-deps.sh /tmp/install-apt-deps.sh
RUN bash /tmp/install-apt-deps.sh && rm /tmp/install-apt-deps.sh

# 2. just, so the same recipes work inside the container.
ARG JUST_VERSION=1.59.0
RUN arch="$([ "${TARGETARCH}" = arm64 ] && echo aarch64 || echo x86_64)" \
 && curl -fsSL "https://github.com/casey/just/releases/download/${JUST_VERSION}/just-${JUST_VERSION}-${arch}-unknown-linux-musl.tar.gz" \
    | tar -xz -C /usr/local/bin just \
 && just --version

# 3. Underlay sources, pinned in deps/underlay.repos.
#    Only the ArduPilot submodules a SITL build needs (not ChibiOS & co., which are large and
#    only used for flight-controller firmware). waf is told not to fetch more (step 6).
WORKDIR /opt/underlay
COPY deps/underlay.repos /opt/underlay/underlay.repos
RUN mkdir -p src \
 && vcs import --shallow --input underlay.repos src \
 && git -C src/Micro-XRCE-DDS-Gen submodule update --init --recursive --depth 1 \
 && touch src/Micro-XRCE-DDS-Gen/COLCON_IGNORE \
 && git -C src/ardupilot submodule update --init --recursive --depth 1 \
      modules/waf modules/mavlink modules/gtest modules/lwip modules/littlefs \
      modules/Micro-XRCE-DDS-Client modules/Micro-CDR \
      modules/DroneCAN/DSDL modules/DroneCAN/pydronecan modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard

# 4. Micro-XRCE-DDS-Gen: ArduPilot's waf calls `microxrceddsgen` when DDS is enabled.
RUN cd src/Micro-XRCE-DDS-Gen \
 && ./gradlew assemble --no-daemon --quiet \
 && rm -rf /root/.gradle
ENV PATH=/opt/underlay/src/Micro-XRCE-DDS-Gen/scripts:${PATH}

# 5. Python tools with no apt package. Pinned; see docs/adr/0003-ardupilot-dds.md.
#    MAVProxy matches ArduPilot's own CI at the pinned tag. pymavlink is installed from the
#    ArduPilot submodule so its MAVLink dialect matches the firmware we run.
#    pre-commit is a standalone tool: it gets its own venv so it cannot replace Debian's Python
#    packages (pip refuses to uninstall them, and colcon/rosdep depend on them).
#    MAVProxy imports `future` without declaring it, hence the explicit pin.
ARG MAVPROXY_VERSION=1.8.71
ARG FUTURE_VERSION=1.0.0
ARG DRONECAN_VERSION=1.0.27
ARG PRE_COMMIT_VERSION=4.7.0
ARG ZXING_CPP_VERSION=2.2.0
RUN pip install "MAVProxy==${MAVPROXY_VERSION}" "future==${FUTURE_VERSION}" "dronecan==${DRONECAN_VERSION}" \
 && (cd src/ardupilot/modules/mavlink/pymavlink && pip install --upgrade .) \
 && if [ "${ROS_DISTRO}" = humble ]; then pip install "zxing-cpp==${ZXING_CPP_VERSION}"; fi \
 && python3 -m venv /opt/pre-commit \
 && /opt/pre-commit/bin/pip install "pre-commit==${PRE_COMMIT_VERSION}" \
 && ln -s /opt/pre-commit/bin/pre-commit /usr/local/bin/pre-commit \
 && mavproxy.py --version && pre-commit --version

# 6. Build the underlay: ArduPilot SITL (all vehicles, DDS on), ardupilot_gazebo, micro-ROS agent.
#    On Jazzy, Gazebo Harmonic comes from ROS vendor packages, so the gz-* rosdep keys are skipped.
ARG BUILD_JOBS=8
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install -y -r --rosdistro "${ROS_DISTRO}" --ignore-src \
      --from-paths src/ardupilot/Tools/ros2/ardupilot_msgs src/ardupilot/Tools/ros2/ardupilot_sitl \
                   src/ardupilot_gazebo src/micro_ros_agent \
      --skip-keys "gz-cmake3 gz-sim8 gz-common5 gz-plugin2 microxrcedds_agent" \
 && rm -rf /var/lib/apt/lists/*
# Backported upstream fixes for the pinned sources (each patch says why; deps/README.md).
COPY deps/patches /opt/underlay/patches
RUN for p in /opt/underlay/patches/ardupilot/*.patch; do git -C src/ardupilot apply --verbose "$p"; done
RUN --mount=type=cache,target=/root/.ccache,id=coredrone-ccache-${TARGETARCH} \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && export MAKEFLAGS="-j${BUILD_JOBS}" CCACHE_DIR=/root/.ccache \
 && colcon build --merge-install \
      --packages-up-to ardupilot_sitl ardupilot_gazebo micro_ros_agent \
      --parallel-workers 2 \
      --event-handlers console_cohesion- console_direct- status- \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                   -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
                   "-DARDUPILOT_WAF_CONFIGURE_ARGS=--no-submodule-update" \
                   "-DARDUPILOT_WAF_BUILD_ARGS=-j${BUILD_JOBS}" \
 && rm -rf build log src/ardupilot/build \
 && test -x install/bin/arducopter

# =============================================================================================
# Stage `manifests`: only the overlay's package.xml files, so the rosdep layer below is
# invalidated by dependency changes but not by ordinary source edits.
FROM ros:${ROS_DISTRO}-ros-base AS manifests
COPY drone/src /src
RUN mkdir /manifests \
 && cd /src \
 && find . -name package.xml -exec sh -c 'mkdir -p "/manifests/$(dirname "$1")" && cp "$1" "/manifests/$1"' _ {} \;

# =============================================================================================
# Stage `dev` (the published image): underlay + overlay dependencies + developer user.
FROM underlay AS dev
ARG ROS_DISTRO

# 7. Overlay dependencies, resolved from drone/src/*/package.xml by rosdep.
#    Project-local keys live in deps/rosdep.yaml. pymavlink is skipped: step 5 installed it.
COPY deps/rosdep.yaml /opt/inv/rosdep.yaml
RUN echo "yaml file:///opt/inv/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/10-coredrone.list
COPY --from=manifests /manifests /tmp/overlay-manifests
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && source /opt/underlay/install/setup.bash \
 && rosdep install -y --rosdistro "${ROS_DISTRO}" --ignore-src \
      --from-paths /tmp/overlay-manifests \
      --skip-keys "python3-pymavlink ardupilot_msgs" \
 && rm -rf /var/lib/apt/lists/* /tmp/overlay-manifests

# 8. Non-root developer user (UID 1000, matching most Linux hosts) and the workspace mount points.
#    Ubuntu 24.04 images already ship a UID-1000 user called `ubuntu`; rename it.
ARG USERNAME=dev
RUN if id -u ubuntu >/dev/null 2>&1; then \
      usermod -l "${USERNAME}" -d "/home/${USERNAME}" -m ubuntu && groupmod -n "${USERNAME}" ubuntu; \
    else \
      useradd -m -s /bin/bash -u 1000 "${USERNAME}"; \
    fi \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
 && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
 && git config --system --add safe.directory '*' \
 && mkdir -p /ws/drone/build /ws/drone/install /ws/drone/log \
             "/home/${USERNAME}/.cache/ccache" "/home/${USERNAME}/.cache/pre-commit" \
 && chown -R "${USERNAME}:${USERNAME}" /ws "/home/${USERNAME}"

COPY docker/scripts/env.sh /opt/inv/env.sh
COPY docker/scripts/entrypoint.sh /usr/local/bin/inv-entrypoint
RUN chmod 0755 /usr/local/bin/inv-entrypoint \
 && echo 'source /opt/inv/env.sh' >> "/home/${USERNAME}/.bashrc"

ENV INV_ROOT=/ws \
    INV_IN_CONTAINER=1 \
    CCACHE_DIR=/home/${USERNAME}/.cache/ccache
USER ${USERNAME}
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/inv-entrypoint"]
CMD ["bash"]
