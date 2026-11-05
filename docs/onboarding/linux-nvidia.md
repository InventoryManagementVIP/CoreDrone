# Onboarding: Linux + NVIDIA RTX

Everything in [linux.md](linux.md), plus Isaac ROS Visual SLAM and nvblox against the simulated
cameras. This is the only place besides the Jetson where the `isaac` localization and `nvblox`
mapping backends run, and GitHub-hosted CI cannot run them.

## Requirements (Isaac ROS 4.6)

- Ampere or newer NVIDIA GPU with 8 GB+ memory, Ubuntu 24.04, NVIDIA driver 595+.
- NVIDIA Container Toolkit (`nvidia-ctk runtime configure --runtime=docker`).
- An NGC account if the Isaac ROS base image needs a login (`docker login nvcr.io`).

## Run

The simulator stays in the `dev` container; the `isaac` container shares its network, so both
are one ROS graph.

```bash
just worldgen W=sim_small SENSORS=full
docker compose -f docker/compose.yaml --profile isaac build isaac   # derives from the Isaac ROS 4.6 base
docker compose -f docker/compose.yaml --profile isaac up -d isaac

# terminal 1 (dev): platform only (Gazebo, SITL, agent, bridges, fc_bootstrap)
just _run "ros2 launch sim/launch/sim_platform.launch.py world:=sim_small sensors:=full"

# terminal 2 (isaac): the stack with Isaac VSLAM and nvblox
docker compose -f docker/compose.yaml exec isaac bash
cd /ws/drone && colcon build --symlink-install --build-base build-isaac --install-base install-isaac
source install-isaac/setup.bash
ros2 launch inv_bringup drone.launch.py platform:=sim with_platform:=false localization:=isaac mapping:=nvblox
```

(The separate build/install directories keep the Isaac build apart from the dev container's.)

## Manual GPU check (before merging changes to Isaac configs)

There is no GPU runner yet, so whoever changes `inv_localization/config/isaac_vslam.yaml`,
`inv_mapping/config/nvblox.yaml` or `docker/isaac-x86.Dockerfile` runs this and pastes the result
in the PR:

1. `just worldgen W=sim_small SENSORS=full` then the launch command above.
2. `ros2 topic hz /localization/odom` shows ~30 Hz and `/localization/status` says TRACKING.
3. Fly `mission:=smoke` and check that `/localization/odom` stays within 0.1 m of
   `/sim/ground_truth/odom` (Foxglove plot).

UNVERIFIED: the Isaac ROS 4.6 base image tags in `docker/.env` were inferred from NGC push dates;
confirm them with `isaac-ros activate` on this machine and pin by digest.
