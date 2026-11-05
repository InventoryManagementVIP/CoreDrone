# Onboarding: Linux (no NVIDIA GPU)

What works: the full simulation with the Gazebo GUI, RViz inside the container, Foxglove.

## Install (once)

1. Docker Engine + compose plugin (docs.docker.com/engine/install), and add yourself to the
   `docker` group.
2. `sudo apt install -y git git-lfs pipx && pipx install pre-commit && git lfs install`
3. `just`: `curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin`
4. Allow the container to open windows: `xhost +local:` (per login session; or use a more
   restrictive `xhost +SI:localuser:$(id -un)`).

## First run

```bash
git clone https://github.com/InventoryManagementVIP/CoreDrone.git && cd CoreDrone
just setup && just build && just test
just sim          # Gazebo GUI; Foxglove also on ws://localhost:8765
```

The justfile adds `docker/compose.gui.yaml` when `$DISPLAY` is set and `docker/compose.dri.yaml`
when `/dev/dri` exists (hardware OpenGL on Intel/AMD).

## Native install (optional)

Ubuntu 24.04 + ROS 2 Jazzy natively works too: install the packages from
`docker/scripts/install-apt-deps.sh`, build the underlay from `deps/underlay.repos` the way
`docker/dev.Dockerfile` does (including `deps/patches/`), then `colcon build` in `drone/`. The
container is the supported path; if native and container disagree, the container wins.

## Tips

- The container user has UID 1000. If yours differs, files it creates in the checkout
  (`sim/worlds/generated`, `runs/`) will belong to UID 1000.
- Use a unique `ROS_DOMAIN_ID` (e.g. in `.env`) if several people run ROS on the same network.
