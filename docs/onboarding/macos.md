# Onboarding: macOS (Apple Silicon)

What works: the full simulation with `groundtruth` localization, headless Gazebo, visualisation in
Foxglove. What does not: Isaac ROS (needs an NVIDIA GPU), Gazebo GUI windows.

## Install (once)

1. Docker Desktop for Mac. In *Settings → Resources* give it at least 8 GB RAM and 4 CPUs.
2. `brew install just pre-commit git-lfs` and `git lfs install`.
3. Foxglove (desktop app) from foxglove.dev.
4. Optional ground station: QGroundControl.

## First run

```bash
git clone https://github.com/InventoryManagementVIP/CoreDrone.git && cd CoreDrone
just setup        # pulls the arm64 dev image (~4 GB), starts it, installs git hooks
just build
just test
just sim          # headless on macOS; open Foxglove -> ws://localhost:8765
```

`just setup` builds the image locally if it is not published yet; that takes 30-60 minutes once
(ArduPilot is compiled inside it).

## Tips

- `just shell` gives you a shell in the container; the repo is at `/ws`.
- QGroundControl: it listens on UDP 14550 by default and sees SITL automatically.
- Keep the repo out of iCloud Drive-synced folders (Desktop/Documents with iCloud on): the
  container writes generated worlds and run outputs into it.
- If Docker is slow, check that *Use Rosetta for x86/amd64 emulation* is not forcing the amd64
  image: `docker image inspect ghcr.io/inventorymanagementvip/coredrone-dev:jazzy --format '{{.Architecture}}'`
  should print `arm64`.
