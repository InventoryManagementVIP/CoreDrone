# CoreDrone: autonomous indoor inventory drone

A quadcopter that flies a predefined route through warehouse aisles, stops at every shelf slot,
reads barcodes / QR codes / AprilTags, ties each read to a rack slot, lands, and syncs the results to
the backend behind the team's Companion App.

ArduPilot Copter on a Pixhawk 6C · NVIDIA Jetson Orin Nano companion · OAK-D Lite ([parts](real/BOM.md)) · ROS 2 Jazzy ·
Isaac ROS VSLAM (or simulated ground truth) · Gazebo Harmonic + ArduPilot SITL for everything that
can be tested before the hardware arrives.

## Quickstart

You need Docker and [just](https://just.systems). Everything else runs in the `dev` container.

**macOS (Apple Silicon)** · details: [docs/onboarding/macos.md](docs/onboarding/macos.md)
```bash
brew install just pre-commit git-lfs && git lfs install
git clone https://github.com/InventoryManagementVIP/CoreDrone.git && cd CoreDrone
just setup && just build && just test
just sim                      # headless; open Foxglove at ws://localhost:8765
```

**Windows** (WSL2 Ubuntu shell + Docker Desktop) · [docs/onboarding/windows-wsl2.md](docs/onboarding/windows-wsl2.md)
```bash
git clone https://github.com/InventoryManagementVIP/CoreDrone.git ~/src/CoreDrone && cd ~/src/CoreDrone
just setup && just build && just test
just sim                      # Gazebo opens through WSLg
```

**Linux** · [docs/onboarding/linux.md](docs/onboarding/linux.md) · with an NVIDIA RTX GPU also
[linux-nvidia.md](docs/onboarding/linux-nvidia.md)
```bash
git clone https://github.com/InventoryManagementVIP/CoreDrone.git && cd CoreDrone
xhost +local: && just setup && just build && just test
just sim                      # Gazebo GUI + Foxglove
```

**Jetson** (the vehicle): [docs/onboarding/jetson.md](docs/onboarding/jetson.md). Not flown yet.

## Everyday commands

| | |
|---|---|
| `just shell` | shell in the dev container (repo at `/ws`) |
| `just build` / `just test` / `just lint` | colcon build, all tests, all linters |
| `just worldgen W=sim_small` | generate a warehouse world from `config/warehouses/sim_small.yaml` |
| `just sim W=sim_small` | Gazebo + ArduPilot SITL + the full stack |
| `just sim-smoke` | take off, hover, land in SITL (what CI runs) |
| `just sim-mission W=sim_small` | full scan mission + evaluation report |
| `just params-diff CONNECTION=...` | compare a flight controller with the committed parameters |
| `just params-check` | every committed parameter name exists in the pinned ArduPilot (run after a bump) |

`just --list` shows the rest (Foxglove, bag recording, image builds, clean-up).

## Repository map

| Path | What |
|---|---|
| `drone/src/` | the ROS 2 stack: one copy, used in sim and on the vehicle ([architecture](docs/architecture.md)) |
| `sim/` | simulation platform: world generator, SITL params, evaluation, sim tests |
| `real/` | vehicle platform: Jetson deployment, sensors, flight controller, flight-test checklists |
| `config/warehouses/` | warehouse layouts: drive the sim world, the route and slot attribution |
| `contracts/` | drone → backend record format and the layout schema |
| `docker/`, `deps/` | the images and the pinned upstream sources ([why each pin](deps/README.md)) |
| `docs/` | [architecture](docs/architecture.md), [interfaces](docs/interfaces.md), [ArduPilot](docs/ardupilot.md), [simulation](docs/simulation.md), [decisions (ADRs)](docs/adr/README.md) |
| `app/` | reserved; the Companion App lives in its own repository |

## Contributing

Branch, open a PR, fill in the template; CI runs lint, builds and tests on amd64 and arm64, and the
SITL smoke test when relevant paths change. Decisions that others will ask "why?" about get an ADR.
Owners per subsystem are in [.github/CODEOWNERS](.github/CODEOWNERS).
