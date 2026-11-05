# Agent Brief: Set Up the Drone Inventory Monorepo

> **Who this is for:** an autonomous coding agent setting up a brand-new repository.
> **Your job right now:** build the scaffolding: layout, environments, simulation, interfaces, CI, and docs. Do **not** build the full autonomy features yet. Every package should exist, build, launch, and have a defined interface. Most should be stubs. The one exception is the SITL loop, which must work end to end (see §13).
> **Ask the human before:** pinning the ROS 2 distro (§3.1), moving the companion app into this repo (§3.9), or adding any heavy dependency not listed here.

---

## 1. Project context

A university team is building an autonomous indoor drone for warehouse inventory. The flight sequence:

1. The drone takes off from the end of an aisle.
2. It flies a predefined route along the racks, covering every shelf level.
3. It reads barcodes, QR codes, and/or AprilTags.
4. It ties each read to a rack and slot location.
5. It lands.
6. The results go to a backend: a Supabase/Postgres database behind a web endpoint.
7. A separate team's React **Companion App**, with an LLM "inventory agent," shows the results and sends push notifications.

**Hardware (some parts not yet delivered):**
- Custom Quad-X frame
- Pixhawk-class flight controller running **ArduPilot Copter**
- **NVIDIA Jetson** companion computer (model not confirmed)
- **Stereo/depth camera with an integrated IMU** (model not confirmed)

The team is waiting on the Jetson and sensors, so simulation (SITL) comes first.

**Target onboard architecture** (the team's diagram, which this brief refines in §4):

```
Stereo camera + IMU ─┬─> Isaac ROS Visual SLAM (pose + saved map) ─┬─> Mission planner (behavior tree) ─┐
                     ├─> nvblox (3D obstacle map) ─────────────────┘                                    │
                     └─> Tag detector (barcodes, AprilTags) ──> Inventory node (items in map frame)      │
                                                                                                         v
                                Micro XRCE-DDS agent  (/ap/tf in, /ap/cmd_vel out) <─────────────────────┘
                                          │
                                   ArduPilot (EKF3, external nav) <-·-·-> Ground station (params, logs)
```

**Team constraints that shape the repo:**
- Students rotate in and out each semester. Onboarding has to take minutes, and the reasons behind decisions have to be written down.
- Most members use **macOS (Apple Silicon)**. Some use Windows. Few have NVIDIA GPUs. The previous attempt (ORB-SLAM3, then Stella VSLAM) stalled largely because it was hard to install on Macs.
- The old repo is being abandoned. Do not port its code. Carry over only its lessons (§2).

---

## 2. Problems from the old setup this repo must fix

| Old problem | What the new repo does about it |
|---|---|
| SLAM libraries built from source, broken on macOS | No source-built SLAM. Use binary/containerized NVIDIA Isaac ROS on NVIDIA hardware, and a **swappable localization interface** with a ground-truth backend so anyone can develop without a GPU (§6, §8). |
| Consumer drone (Tello): rolling-shutter jello, EIS, closed API | Done: moved to ArduPilot + Jetson. The repo encodes that platform. |
| No simulation, so nothing could be tested before hardware arrived | Gazebo Harmonic + ArduPilot SITL as a first-class `sim/` tree, runnable headless in CI. |
| "Works on my machine" | Every workflow runs in pinned Docker images through a single task runner (`just`). Native installs are optional. |
| Knowledge lost between semesters | ADRs (`docs/adr/`), an architecture doc, per-platform onboarding, and CODEOWNERS per subsystem. |
| Sim code and real code drifting apart | One autonomy stack. Only the platform layer (drivers, launch args, params) differs between `sim/` and `real/` (§5). |

---

## 3. Decisions: locked, recommended (with challenges), and open

### 3.1 ROS 2 distro: **open, ask the human, but present this analysis**

- Today is October 2026. **ROS 2 Humble reaches end of life in May 2027.** **Jazzy** is supported until May 2029. Starting a new repo on Humble means a forced migration within a year.
- **Point against Jazzy:** ArduPilot's ROS 2 docs still say "Humble is the only version supported." AP_DDS is reported to work on Jazzy, but verify it.
- **Point for Jazzy:** Gazebo Harmonic's *default* pairing is Jazzy. ArduPilot's docs describe Humble + Harmonic as a "non-default pairing."
- **The deciding constraint is Isaac ROS on the Jetson we actually get.** Isaac ROS 3.x targets Humble on JetPack 6. Isaac ROS 4.x moved to Jazzy, with JetPack 7. **Check which Isaac ROS release supports the delivered Jetson's JetPack.** That answer decides the distro.
- **Default if unanswered:** set the distro as a **single build argument** (`ROS_DISTRO`) used by every Dockerfile and CI job. Start on whichever distro Isaac ROS supports for the target Jetson. Write an ADR that records the decision and the Humble EOL date.

### 3.2 Flight stack: **locked: ArduPilot Copter + AP_DDS (Micro XRCE-DDS)**

- Pin the **latest stable ArduPilot Copter release**. Verify the tag.
- ROS 2 talks to ArduPilot through the native DDS interface:
  - **Published by ArduPilot:** `/ap/pose/filtered`, `/ap/twist/filtered`, `/ap/imu/experimental/data`, `/ap/battery`, `/ap/clock`, `/ap/time`, …
  - **Subscribed by ArduPilot:** `/ap/tf` (external odometry), `/ap/cmd_vel` (TwistStamped), `/ap/cmd_gps_pose`, `/ap/joy`
  - **Services:** `/ap/arm_motors`, `/ap/mode_switch`, `/ap/experimental/takeoff`, `/ap/prearm_check`, `/ap/get_parameters`, `/ap/set_parameters`
- **Challenge 1: the flight controller may not support DDS.** AP_DDS needs enough flash memory and a spare serial port (or Ethernet). Some FC boards need a custom firmware build with DDS enabled. Document this in `real/flight_controller/README.md`. List MAVROS as the fallback.
- **Challenge 2: don't let this decision leak.** Mission code must **never** use `/ap/*` directly. Everything goes through one `flight_interface` package (§6), so switching to MAVROS later is a change inside one package.
- Keep a **MAVLink link to the ground station** (Mission Planner or QGroundControl) at the same time, for parameters, logs, and the safety pilot. On the Jetson, use `mavlink-router` or a second telemetry port.

### 3.3 Localization / VSLAM: **recommended: Isaac ROS Visual SLAM (cuVSLAM), behind a swappable interface**

- **The team's documents contradict each other.** The midterm deck and the process doc say *Stella VSLAM* (ORB features). The latest meeting notes and the architecture diagram say *Isaac ROS VSLAM + nvblox*. **Go with Isaac ROS.** It is GPU-accelerated on the Jetson, shipped as containers, and fused with stereo + IMU. **Do not vendor ORB-SLAM3 or Stella VSLAM.** That is the old repo's failure mode.
- **Challenge:** Isaac ROS needs CUDA. It **cannot run on Macs, on most laptops, or in GitHub-hosted CI.** So the repo must define localization as an interface, not an implementation. **Contract:** the backend publishes TF `map → odom → base_link` (REP-105) plus `nav_msgs/Odometry` on `/localization/odom`.
- **Backends to scaffold:**
  - `isaac`: real Jetson, or an x86 machine with an RTX GPU, in the Isaac ROS container.
  - `groundtruth`: sim only. Gazebo's true pose, with **optional configurable noise and drift** to mimic VSLAM. This is what Mac/Windows/CI developers use.
  - `rtabmap` (optional, later): CPU fallback, installable from apt, supports stereo + IMU. Better than ORB-SLAM3/Stella if a non-GPU real-sensor path is ever needed.
- **Selection:** a launch argument, `localization:=isaac|groundtruth|rtabmap`.

### 3.4 Obstacle mapping: **recommended: nvblox, scoped down**

- nvblox also needs CUDA. Same pattern as §3.3: `mapping:=nvblox|none`.
- **Challenge: don't build a 3D path planner now.** The route is predefined (§3.7). In phase 1, nvblox's job is a **"stop and hold if the corridor ahead is blocked"** check: query the ESDF/costmap along the next route segment. Full replanning comes later, if ever.

### 3.5 Camera: **challenge the "4K" spec**

- The deck sells the camera as "full color, 4K." **4K is the wrong metric for this system.** What matters:
  - **VSLAM:** global-shutter stereo, hardware-synced IMU, roughly 640–1280 px width at 30–60 fps. That is what cuVSLAM is designed for.
  - **Reading labels from a moving drone:** enough pixels *on the label*. That depends on resolution, field of view, distance, and motion blur.
  - **Jetson load:** a 4K stream will eat the Jetson's bandwidth and compute budget.
- **Recommendation:** confirm the camera is on Isaac ROS's supported list (RealSense D435i/D455, ZED family, and so on). On a RealSense, run VSLAM on the **IR stereo pair** (global shutter, with the IR emitter off or interleaved) and run label decoding on the **RGB stream** (rolling shutter, acceptable while hovering). If label reads fail at real distances, add a narrow-FOV, high-resolution camera only for scanning.
- **Repo implication:** the sensor driver config lives in `real/`. Camera topics are remapped to canonical names (§4.2), so swapping cameras never touches autonomy code.

### 3.6 Inventory location attribution: **challenge: don't rely on SLAM alone**

- The plan says to map each barcode to a rack/slot using the SLAM pose at scan time. **VSLAM drifts**, warehouse aisles are repetitive and low-texture, and slots are often under 1 m apart. Pose-only attribution will misassign slots.
- **Recommendation:** use the labels themselves.
  - Real warehouses already put **location labels on rack beams**.
  - Where they don't, put **AprilTags on the rack uprights and beams**.
  - Assign each item to a slot from the location label/tag seen **in the same frame or a nearby frame**. Use the SLAM pose only as a tiebreaker and a sanity check.
  - Rack AprilTags can also serve as absolute localization fixes later.
- The warehouse layout file (§3.8) defines which tag ID or location code belongs to which rack/slot.

### 3.7 Mission / behavior: **recommended: stop-and-scan waypoint routes, py_trees**

- **Routes:** routes are **generated from the warehouse layout file**: aisle → rack face → shelf levels → hover points. They are not hand-flown and not free-planned.
- **Scanning:** **stop and hover** at each scan point. Don't scan while moving. Hovering removes most motion blur and timing problems.
- **Behavior tree library:** **`py_trees` / `py_trees_ros`**.
  - Reason: the mission layer isn't performance-critical, and rotating students ramp up much faster in Python.
  - The alternative, BehaviorTree.CPP v4 + Groot2, is what Nav2 uses. Record this choice in an ADR.

### 3.8 Simulation: **locked: Gazebo Harmonic + ArduPilot SITL (`ardupilot_gazebo` / `ardupilot_gz`)**

- This is ArduPilot's officially supported ROS 2 simulation path. Reference launch: `ardupilot_gz_bringup iris_maze.launch.py`. It runs without an NVIDIA GPU and can run headless in CI.
- **Isaac Sim** is **not** in scope now. It is photoreal and better for testing perception, but it needs an RTX GPU and is heavy. Leave a `sim/isaac/README.md` placeholder that explains when you would add it.
- **Key idea: one warehouse layout file drives everything.** `config/warehouses/<name>.yaml` (racks, aisles, levels, slot codes, tag IDs, and for sim, items per slot) feeds:
  1. the **sim world generator** (SDF with racks plus boxes textured with generated QR/barcode/AprilTag images)
  2. **mission route generation**
  3. **slot attribution** in the inventory node
  4. the **sim ground-truth inventory**, used to score a mission run automatically (recall, precision, correct-slot rate)

### 3.9 Monorepo scope: **recommended: yes, with hard boundaries**

```
repo/
  drone/      ROS 2 overlay workspace (this brief's focus)
  sim/        simulation assets and tools
  real/       hardware-specific config and deployment
  contracts/  shared data schemas between drone and backend
  app/        companion app: reserved, see below
  docs/
```

- **Ask the human** whether the companion app (React + Supabase) moves into `app/` now. **Default:** create `app/README.md` that points to the existing app repo, and don't migrate anything. That team has its own momentum.
- Whichever way that goes, the **drone → backend contract** lives in `contracts/`. That keeps the two teams from breaking each other.

### 3.10 Drone → backend uplink: **challenge: make it store-and-forward**

- The process doc has the drone "continuously sending scans wirelessly to a web endpoint." Warehouse Wi-Fi is unreliable (metal racks), and **flight must never block on the network.**
- **Recommendation:** the inventory node writes to a **local SQLite queue on the Jetson**. A separate `uplink` node syncs to the backend when it can connect (idempotent upserts keyed by `scan_id`), and exports the CSV the process doc mentions.
- **Credentials:** never commit them. Load them from env/secret files ignored by git.

### 3.11 Altitude and EKF fallback: **challenge: VSLAM-only is fragile indoors**

- Indoors, the barometer is noisy (HVAC, prop wash near shelves). If VSLAM loses tracking, there is no fallback.
- **Recommend to the hardware team:**
  - a **downward rangefinder** as the height source
  - an **optical flow sensor** as `EK3_SRC2_*`, with an RC switch to change EKF source sets
- **Repo implication:** parameter files (§9) carry both source sets, and the sim models both sensors where Gazebo supports them.

---

## 4. Architecture: refinements to the team diagram

### 4.1 Gaps in the current diagram (fix these in `docs/architecture.md`)

1. **Inventory node has no pose input.** It needs TF (`map → camera`) at detection time, and ideally the location-label detections (§3.6).
2. **Nothing gets data off the drone.** Add an **uplink** node (§3.10).
3. **Isaac VSLAM's output doesn't go straight to `/ap/tf`.** It goes through a **localization adapter**. The adapter:
   - re-publishes only `odom → base_link` to `/ap/tf` (the only transform AP_VisualOdom accepts) at a steady rate
   - handles the base_link vs. camera-frame offset (set the `VISO_POS_*` parameters accordingly, to zero if already in base_link)
   - monitors tracking health, and if tracking is lost, raises a failsafe signal to the mission layer
4. **The mission planner should not command `/ap/cmd_vel` directly.** It goes through the **flight interface** node. That node owns arm, mode, takeoff, velocity/position commands, a command watchdog, and geofence/limits.
5. **Add a safety/health monitor:**
   - inputs: battery, localization health, DDS link heartbeat, nvblox "blocked" signal
   - output: a mission abort → hold → land sequence
   - the RC safety pilot override always wins
6. **Time sync:** the Jetson and FC clocks must agree for EKF fusion. Use chrony on the Jetson, and note that `/ap/time` is available. Sim uses `/clock` with `use_sim_time`.
7. **The EKF origin must be set when flying without GPS.** Verify the current mechanism. It is typically MAVLink `SET_GPS_GLOBAL_ORIGIN`, sent by a small helper at startup. Script it for both sim and real.

### 4.2 Canonical topic and frame contract (put in `docs/interfaces.md`, enforced by remaps)

| Purpose | Canonical name | Type | Producer |
|---|---|---|---|
| Stereo left/right images + info | `/camera/left/image_raw`, `/camera/right/image_raw`, `/camera/*/camera_info` | sensor_msgs | driver (real) / gz bridge (sim) |
| Depth | `/camera/depth/image_raw` (+ info) | sensor_msgs | driver / gz bridge |
| Label camera (RGB) | `/camera/color/image_raw` (+ info) | sensor_msgs | driver / gz bridge |
| Camera IMU | `/camera/imu` | sensor_msgs/Imu | driver / gz bridge |
| Localization | TF `map→odom→base_link`, `/localization/odom`, `/localization/status` | TF, Odometry, custom | localization backend |
| Obstacles | `/mapping/blocked` (+ nvblox native topics) | custom | mapping backend |
| Detections | `/perception/detections` | `inv_interfaces/LabelDetectionArray` | perception |
| Inventory | `/inventory/items` | `inv_interfaces/InventoryItemArray` | inventory |
| Mission state | `/mission/status` | `inv_interfaces/MissionStatus` | mission |
| Flight commands | action/service API defined by `flight_interface` (GoTo, Hover, Takeoff, Land, Abort) | `inv_interfaces` | flight_interface |

**Frames:**
- ENU / REP-103 / REP-105 throughout.
- Sensor extrinsics have **one source of truth**: the URDF/xacro in the description package. Sim sensor poses must be generated from or checked against it. Don't hand-copy numbers.

### 4.3 Revised diagram (mermaid, for `docs/architecture.md`)

```mermaid
flowchart TB
  subgraph Platform["Platform layer (differs: sim vs real)"]
    CAM[Stereo + depth + IMU\nreal driver OR gz bridge]
    FC[ArduPilot\nreal FC OR SITL]
  end
  subgraph Autonomy["Autonomy stack (identical in sim and real)"]
    LOC[Localization backend\nisaac | groundtruth | rtabmap]
    ADP[Localization adapter\n→ /ap/tf, health]
    MAP[Mapping\nnvblox | none]
    PER[Perception\nbarcode / QR / AprilTag]
    INV[Inventory\nslot attribution + SQLite queue]
    MIS[Mission\npy_trees, routes from layout]
    FLT[Flight interface\narm/mode/goto/watchdog]
    SAF[Safety monitor]
    UPL[Uplink\nstore-and-forward]
  end
  CAM --> LOC --> ADP --> FC
  CAM --> MAP --> MIS
  CAM --> PER --> INV
  LOC --> INV
  INV --> UPL --> BE[(Backend / Supabase)]
  MIS --> FLT --> FC
  FC --> FLT
  SAF --> MIS
  ADP --> SAF
  FC -. MAVLink .- GCS[Ground station]
```

---

## 5. Repository layout

**Principle:** `sim/` and `real/` hold **only platform-specific things**: worlds, models, drivers, deployment, and parameter overrides. **All autonomy code lives once, in `drone/src/`.** The top-level launch picks the platform with `platform:=sim|real`. If you find yourself copying a node into `sim/` or `real/`, stop.

Use the name `real/`, not `prod/`. "Prod" suggests a deployed product. This directory is the physical-vehicle platform.

```
drone-inventory/                       # name TBD by team
├── README.md                          # 1-page: what this is, quickstart per OS, links
├── justfile                           # single entrypoint for all workflows (see §7.3)
├── .devcontainer/                     # VS Code / Codespaces devcontainer → docker/dev image
├── .github/
│   ├── workflows/                     # ci.yml, sim-smoke.yml, docker-publish.yml
│   ├── CODEOWNERS                     # per subsystem, for rotating owners
│   └── pull_request_template.md
├── .pre-commit-config.yaml            # ruff, ruff-format, clang-format, yamllint, xmllint, check-added-large-files
├── .gitattributes                     # LFS for meshes/textures (*.dae, *.stl, *.png under sim/models)
├── docker/
│   ├── dev.Dockerfile                 # ROS 2 + Gazebo Harmonic + ArduPilot SITL + micro-ROS agent; multi-arch (amd64, arm64)
│   ├── isaac-x86.Dockerfile           # FROM Isaac ROS base (x86 + NVIDIA GPU); adds our overlay
│   ├── jetson.Dockerfile              # FROM Isaac ROS base (aarch64/JetPack); onboard runtime image
│   └── compose.yaml                   # services: dev, sim (headless), foxglove
├── deps/
│   ├── underlay.repos                 # vcs: ardupilot (pinned tag), ardupilot_gz, ardupilot_gazebo, micro-ROS-agent, Micro-XRCE-DDS-Gen
│   └── README.md                      # why each pin; how to bump
├── contracts/
│   ├── scan_record.schema.json        # drone → backend record (JSON Schema), versioned
│   ├── warehouse_layout.schema.json   # schema for config/warehouses/*.yaml
│   └── README.md
├── config/
│   └── warehouses/
│       ├── sim_small.yaml             # 2 aisles × 2 racks × 3 levels, for CI
│       └── sim_large.yaml
├── drone/                             # ROS 2 overlay workspace root (colcon build here)
│   └── src/
│       ├── inv_interfaces/            # ament_cmake: msgs/srvs/actions
│       ├── inv_description/           # URDF/xacro of airframe + sensor extrinsics (source of truth)
│       ├── inv_bringup/               # top-level launch: drone.launch.py platform:=sim|real localization:=... mapping:=...
│       ├── inv_localization/          # backends (groundtruth, isaac config, rtabmap stub) + adapter → /ap/tf + health
│       ├── inv_mapping/               # nvblox config + "corridor blocked" checker; `none` backend
│       ├── inv_perception/            # barcode/QR decode (zxing-cpp), AprilTag (isaac_ros_apriltag on GPU / apriltag_ros on CPU)
│       ├── inv_inventory/             # detection + TF → items; slot attribution from layout; SQLite queue
│       ├── inv_mission/               # py_trees_ros BT; route generator from layout
│       ├── inv_flight_interface/      # the ONLY package that touches /ap/*; arm/mode/takeoff/goto/land, watchdog, limits
│       ├── inv_safety/                # health aggregation → abort/hold/land
│       ├── inv_uplink/                # SQLite → backend sync, CSV export
│       └── inv_tools/                 # bag helpers, EKF-origin setter, param diff, dev utilities
├── sim/
│   ├── README.md
│   ├── worldgen/                      # Python: layout YAML → SDF world + label textures + ground_truth.json
│   ├── models/                        # drone SDF (iris-derived + stereo/depth/IMU/rangefinder), rack/box models
│   ├── worlds/                        # generated (gitignored) + a few hand-made
│   ├── launch/                        # sim_platform.launch.py: Gazebo + SITL + micro-ROS agent + gz↔ROS bridges (canonical topic names)
│   ├── params/                        # ArduPilot SITL overrides (layered on real/flight_controller/params/common.parm)
│   ├── evaluation/                    # score a run: inventory output vs ground_truth.json → report
│   ├── tests/                         # launch_testing smoke tests (takeoff/hover/land; scan mission)
│   └── isaac/README.md                # placeholder: when/why to add Isaac Sim
├── real/
│   ├── README.md
│   ├── jetson/                        # JetPack version, setup script, docker run/compose for onboard, systemd units, chrony
│   ├── sensors/                       # camera driver launch/config, udev rules, calibration results (Kalibr outputs)
│   ├── flight_controller/
│   │   ├── README.md                  # FC model, firmware build (DDS enabled), serial/Ethernet wiring for XRCE-DDS
│   │   └── params/                    # common.parm, real.parm, per-airframe tuning; diffed in PRs
│   ├── launch/                        # real_platform.launch.py: drivers + micro-ROS agent (serial) + remaps to canonical topics
│   └── flight_test/                   # pre-flight checklists, test cards, log template
├── app/README.md                      # pointer to companion app repo (unless migration approved)
└── docs/
    ├── architecture.md                # §4 content + diagram
    ├── interfaces.md                  # §4.2 topic/frame contract
    ├── onboarding/                    # macos.md, windows-wsl2.md, linux.md, linux-nvidia.md, jetson.md
    ├── simulation.md
    ├── ardupilot.md                   # DDS, EKF sources, params, EKF origin, GCS
    └── adr/                           # 0001-monorepo.md, 0002-ros-distro.md, 0003-ardupilot-dds.md,
                                       # 0004-isaac-vslam-swappable-localization.md, 0005-gazebo-harmonic.md,
                                       # 0006-label-based-slot-attribution.md, 0007-py-trees.md, 0008-store-and-forward-uplink.md
```

**Package conventions:**
- Python packages use `ament_python`. Messages use `ament_cmake`. Use C++ only where performance requires it.
- `inv_` is a placeholder prefix. Let the team rename it.
- Every node:
  - declares its parameters with defaults in a YAML file under its package's `config/`
  - supports `use_sim_time`
  - has at least one unit test plus a `launch_testing` "starts and publishes" test
- Dependencies are declared in `package.xml` and resolved by `rosdep`. Avoid ad-hoc `pip install`. When pip is unavoidable, pin it in the Dockerfile and note it in the ADR.

---

## 6. Package specs for this phase (scaffold depth)

| Package | Must work now | Can be stubbed |
|---|---|---|
| `inv_interfaces` | All messages/actions from §4.2 defined and building | — |
| `inv_description` | Airframe + sensor frames, valid URDF, publishes TF via `robot_state_publisher` | Accurate meshes |
| `inv_bringup` | `drone.launch.py` composes all packages; platform/localization/mapping arguments work | — |
| `inv_localization` | **`groundtruth` backend** (from Gazebo pose, optional noise/drift parameters) + **adapter publishing `odom→base_link` on `/ap/tf`** + status topic | `isaac`: config + launch only, untested without GPU; `rtabmap` stub |
| `inv_flight_interface` | Arm, GUIDED mode, takeoff, go-to-local-pose / velocity, land, cmd watchdog, all against SITL | Geofence beyond simple box limits |
| `inv_mission` | Route generator from layout YAML; a BT that runs takeoff → visit N hover points → land | Scan logic, retries, failsafe branches beyond abort |
| `inv_perception` | Decode QR/AprilTag from the sim RGB camera on a test world (CPU path) | Isaac GPU AprilTag path, 1D barcodes tuning |
| `inv_inventory` | Message plumbing, SQLite queue, slot attribution from layout (simple nearest-label rule) | Deduplication, confidence fusion |
| `inv_safety` | Aggregates battery + localization status + link heartbeat → triggers abort | Smarter policies |
| `inv_uplink` | CSV export; backend client behind an interface with a **mock** backend for tests | Real Supabase sync (needs contract sign-off with app team) |
| `inv_mapping` | `none` backend; nvblox launch/config committed | Corridor-blocked check |

---

## 7. Environments and tooling

### 7.1 Platform support matrix (document in `docs/onboarding/`)

| Developer machine | What works | How |
|---|---|---|
| macOS Apple Silicon | Full sim with `groundtruth` localization; headless Gazebo; view in Foxglove | `dev` image built for **arm64**; Foxglove desktop app connects to `foxglove_bridge` |
| Windows | Full sim, GUI via WSLg | WSL2 + Docker or devcontainer |
| Linux (no NVIDIA) | Full sim, Gazebo GUI | `dev` image or native |
| Linux + NVIDIA RTX | Above + **Isaac VSLAM/nvblox against sim cameras** | `isaac-x86` image |
| Jetson | Real flight | `jetson` image, run by systemd/compose in `real/jetson/` |

- Use **Foxglove** (with `foxglove_bridge` in the images) as the primary visualization tool. It runs natively on Mac/Windows without X forwarding. RViz stays available inside Linux containers.
- **Note:** Gazebo needs hardware OpenGL to perform well. On Mac, Gazebo in Docker runs headless (server only). Document that `just sim` on Mac means headless + Foxglove.

### 7.2 Docker images
- The **underlay** (ArduPilot SITL built at the pinned tag, `ardupilot_gz`, `ardupilot_gazebo`, micro-ROS agent, Micro-XRCE-DDS-Gen at the version matching the ArduPilot release) is built **once inside the image** from `deps/underlay.repos`. Our repo is the **overlay**, bind-mounted for development.
- Publish images to GHCR from CI (multi-arch for `dev`), so students pull them instead of building ArduPilot themselves.
- The Isaac images derive from NVIDIA's Isaac ROS base images, pinned to the Isaac ROS release that matches the Jetson's JetPack.

### 7.3 Task runner (`justfile`): commands every student needs
```
just setup        # pull images, install pre-commit hooks
just shell        # shell in dev container
just build        # colcon build (overlay) with symlink-install
just test         # colcon test + result summary
just lint         # pre-commit run --all-files
just worldgen W=sim_small     # layout → world + ground truth
just sim W=sim_small          # Gazebo + SITL + micro-ROS agent + full stack, localization:=groundtruth
just sim-headless W=sim_small
just sim-smoke    # launch_testing: arm → takeoff 1.5 m → hover 10 s → land (CI)
just sim-mission  # run scan mission on sim_small, then evaluate vs ground truth
just foxglove     # start foxglove_bridge
just params-diff  # diff FC params vs committed real/flight_controller/params
```

### 7.4 Repo hygiene
- **pre-commit:** ruff (lint + format), clang-format, yamllint, xmllint for `package.xml`, `check-added-large-files`, no secrets (gitleaks or detect-secrets).
- **Git LFS** for sim meshes/textures.
- **Never commit rosbags.** Use MCAP, store them externally, and reference them from `docs/datasets.md`.
- `.env.example` for backend URL/keys. The real `.env` is gitignored.
- CODEOWNERS split: `sim/`, `real/`, `drone/src/inv_perception`, `inv_mission`, `inv_flight_interface`, `contracts/`.

---

## 8. Simulation design details

1. **Indoor, GPS-denied, from day one.**
   - Load SITL with parameters that **disable GPS and use ExternalNAV**, so sim exercises the *same* EKF path as the real drone: `AHRS_EKF_TYPE=3`, `EK3_SRC1_POSXY=6`, `EK3_SRC1_VELXY=6`, `EK3_SRC1_VELZ=6`, `EK3_SRC1_YAW=6`, `VISO_TYPE=1`, with POSZ from the rangefinder (or baro to start).
   - Prove the drone holds position with `groundtruth` → adapter → `/ap/tf`.
   - Do **not** test with SITL's default GPS. It hides exactly the integration we need to validate.
2. **Drone model:** start from the `ardupilot_gazebo` iris model and add:
   - a stereo camera pair matching the real baseline and resolution
   - a depth camera
   - an RGB label camera
   - an IMU
   - a downward rangefinder

   Bridge everything to the canonical topics from §4.2 with `ros_gz_bridge`.
3. **World generator** (`sim/worldgen`):
   - **Input:** layout YAML.
   - **Output:** SDF world (floor, racks with beams, boxes per slot, textured faces with generated QR / AprilTag images, rack location labels), plus `ground_truth.json` mapping slot → item code and tag ID → pose.
   - **Determinism:** the generator is seeded, so CI is reproducible.
4. **Evaluation:** `sim/evaluation` compares the inventory node's output (SQLite/CSV) to `ground_truth.json` and reports read recall, false reads, and correct-slot rate. Later this becomes the team's regression metric.
5. **CI-friendly:** headless, `speedup` configurable, deterministic seeds, hard timeouts.

---

## 9. ArduPilot integration specifics (put in `docs/ardupilot.md`)

- **DDS:** `DDS_ENABLE=1`; `DDS_DOMAIN_ID` must equal `ROS_DOMAIN_ID`.
  - **SITL** uses the UDP transport (`ardupilot_sitl` `sitl_dds_udp.launch.py`).
  - **Real** uses a serial connection (or Ethernet on supported FCs) to the micro-ROS agent on the Jetson. Document the port and baud rate.
- **External nav:** the only TF that `AP_VisualOdom` accepts on `/ap/tf` is parent `odom`, child `base_link`. Publish it at a steady rate (target 30 Hz or more). Set `VISO_DELAY_MS` from measured latency.
- **EKF source sets:**
  - **SRC1:** ExternalNAV, height from rangefinder or baro.
  - **SRC2:** optical flow + rangefinder, for when VSLAM is lost.
  - Use an RC switch to select the source set. Document `EK3_SRC_OPTIONS`.
- **EKF origin:** must be set without GPS. Implement `inv_tools/set_ekf_origin` and call it from both platform launches. Verify the current method in ArduPilot docs.
- **Parameter files are code:**
  - `real/flight_controller/params/common.parm` is the shared base.
  - `real.parm` and `sim/params/sim.parm` layer on top.
  - Every parameter change goes through a PR. `just params-diff` compares a live FC against the committed files.
- **Commanding:**
  - The flight interface uses `/ap/mode_switch` → GUIDED, `/ap/arm_motors`, `/ap/experimental/takeoff`, then `/ap/cmd_vel` (verify body vs. earth frame for Copter) or position targets.
  - Include a **command watchdog**: if the mission stops sending, command zero velocity, then hold.
- **Safety pilot:** an RC transmitter with a mode switch (LOITER/ALT_HOLD) and a motor kill switch is mandatory for real flights. Put this in `real/flight_test/checklist.md`.

---

## 10. Drone → backend contract (`contracts/`)

- `scan_record.schema.json` (versioned, `schema_version` field). Draft fields:
  - `scan_id` (UUID, idempotency key)
  - `mission_id`, `drone_id`, `warehouse_id`
  - `timestamp`
  - `code`, `symbology` (qr / ean13 / code128 / apriltag)
  - `slot_code`, `rack_id`, `level`
  - `position_map` {x, y, z}
  - `confidence`
  - `attribution_method` (label | pose)
  - `image_ref` (optional)
- **Ownership:** the app team owns the database. This repo owns the record format. Changes require approval from both CODEOWNERS groups.
- Validate records against the schema in unit tests on the uplink side.

---

## 11. CI (GitHub Actions)

- **On every PR:**
  1. `lint` (pre-commit)
  2. `build-test`: `colcon build` + `colcon test` in the `dev` image (amd64)
  3. `worldgen` unit tests
  4. `contracts`: schema validation
- **On every PR, path-filtered to `drone/**`, `sim/**`, `deps/**`:** `sim-smoke` (headless SITL takeoff/hover/land with `groundtruth` localization). Hard timeout: about 15 minutes.
- **Nightly:** `sim-mission` on `sim_small` plus the evaluation report uploaded as an artifact; multi-arch `dev` image build.
- **Not in CI:** anything that needs CUDA (Isaac VSLAM, nvblox). Document a manual "GPU check" procedure, or a self-hosted runner later.

---

## 12. Docs to write now

- **README:** 10-line quickstart per OS.
- **`docs/architecture.md`:** §4 content.
- **`docs/interfaces.md`:** §4.2 content.
- **`docs/ardupilot.md`:** §9 content.
- **`docs/simulation.md`**
- **`docs/onboarding/*`**
- **ADRs 0001–0008:** each with context, decision, alternatives considered, consequences, and a "revisit when" line. Include the rejected options with reasons:
  - ORB-SLAM3 / Stella (rejected)
  - MAVROS (fallback)
  - Isaac Sim (deferred)
  - BehaviorTree.CPP (alternative)
  - Humble vs. Jazzy (pending)
  - pose-only slot attribution (rejected)

---

## 13. Definition of done for this setup task

1. A fresh clone plus `just setup && just build && just test` passes on Linux amd64 and in the arm64 `dev` image. Verify arm64 at least in CI with QEMU or an arm runner.
2. `just sim W=sim_small` brings up:
   - Gazebo Harmonic with the generated warehouse
   - ArduPilot SITL (Copter, GPS disabled, ExternalNAV)
   - the micro-ROS agent
   - the full `drone.launch.py platform:=sim localization:=groundtruth mapping:=none`

   Then `ros2 topic echo /ap/pose/filtered` shows a stable pose.
3. `just sim-smoke` passes headless and in CI: arm → takeoff → hover → land, all through `inv_flight_interface` (no direct `/ap/*` use outside that package).
4. `just sim-mission` flies the generated route on `sim_small`, decodes at least some QR/AprilTags with the CPU path, writes the SQLite/CSV, and the evaluation script prints a report. Low scores are fine. The pipeline must run.
5. All packages from §5 exist, build, launch, and have tests. Stubs are clearly marked `TODO(phase-2)`.
6. Parameter files, contracts, the layout schema, docs, and ADRs are committed. CI is green.
7. `real/` contains a documented skeleton (Jetson setup steps, FC DDS wiring notes, sensor launch placeholders, flight test checklist). **No hardware claims are presented as verified.**

---

## 14. Verify before pinning (do not trust this brief's memory)

| Item | Check |
|---|---|
| ArduPilot Copter latest stable tag + matching Micro-XRCE-DDS-Gen branch | ardupilot.org / ArduPilot GitHub releases; `Tools/ros2/ros2.repos` |
| AP_DDS support on the chosen ROS 2 distro | ArduPilot dev docs (ros2.html), ardupilot_gz README |
| `ardupilot_gz` / `ardupilot_gazebo` branch for Gazebo Harmonic + chosen distro | their READMEs |
| Isaac ROS release ↔ JetPack ↔ ROS distro ↔ Jetson model | NVIDIA Isaac ROS release notes |
| Camera model supported by Isaac ROS VSLAM (stereo + IMU, global shutter) | Isaac ROS docs; ask the hardware team for the model |
| FC board supports DDS (flash) + available UART/Ethernet | ArduPilot board docs; ask the hardware team |
| How to set EKF origin without GPS in the current Copter version | ArduPilot non-GPS navigation docs |
| `/ap/cmd_vel` frame semantics for Copter | ArduPilot ROS 2 interfaces / source |

---

## 15. Non-goals right now

- Full 3D path planning or exploration.
- Object detection on unmarked boxes (deferred; the AprilTag/label approach comes first).
- Companion app features, the LLM agent, or migrating the app.
- Isaac Sim.
- Multi-drone operation.
- Real flight. The `real/` tree is scaffolded only, until hardware arrives.
