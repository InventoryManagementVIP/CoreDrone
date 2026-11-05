# Simulation

Gazebo Harmonic + ArduPilot SITL, the officially supported ArduPilot ROS 2 path, running the same
autonomy stack as the vehicle. Indoor and GPS-denied from day one: SITL has GPS disabled and the
EKF navigates on external navigation, exactly like the real drone.

## Commands

| Command | What it does |
|---|---|
| `just worldgen W=sim_small [SENSORS=label]` | layout YAML → world, label textures, drone model, `ground_truth.json` |
| `just sim W=sim_small` | Gazebo + SITL + micro-ROS agent + full stack (`localization:=groundtruth mapping:=none`); GUI on Linux/WSLg, headless + Foxglove on macOS |
| `just sim-headless W=sim_small` | same, never opens a window |
| `just sim-smoke` | CI smoke test: arm → take off 1.5 m → hover 10 s → land |
| `just sim-mission W=sim_small` | full scan mission + evaluation report (`runs/latest/evaluation.json`) |
| `just params-check` | every name in the `.parm` files exists in the pinned ArduPilot |

While `just sim` runs: Foxglove → *Open connection* → `ws://localhost:8765`; a GCS on the host
listens on UDP 14550; `ros2 topic echo /ap/pose/filtered` (in `just shell`) shows the FC's pose.

## One layout file drives everything

`config/warehouses/<name>.yaml` (schema: `contracts/warehouse_layout.schema.json`) defines aisles,
racks (side of the aisle, offset, geometry), shelf levels, slot code format, label kinds and sizes,
scan parameters and, for sim only, what is on the shelves (seeded). It feeds:

1. the world generator (`sim/worldgen`): racks, boxes, QR labels on boxes, AprilTags on beams
2. route generation (`inv_mission/route.py`): hover points per slot, serpentine per level
3. slot attribution (`inv_inventory`): location tag/label → slot
4. ground truth (`ground_truth.json`) for `sim/evaluation`

`sim_small` (CI): 2 aisles × 2 racks × 3 levels × 3 bays = 36 slots. `sim_large`: 256 slots.

## World generator

`sim/worldgen` (plain Python; `inv_layout` and `inv_description` provide geometry and extrinsics).
Output in `sim/worlds/generated/<warehouse>/` (gitignored):

- `world.sdf`: floor, lights, `model://warehouse_<id>`, `model://inv_quad`
- `models/warehouse_<id>/`: static racks (uprights, beams, decks), boxes, one textured quad per label
- `models/inv_quad/`: the drone. Airframe, rotors, LiftDrag and ArduPilotPlugin are copied from
  ardupilot_gazebo's pinned `iris_with_ardupilot`; sensor poses come from the URDF; intrinsics from
  `sim/models/inv_quad/sensors.yaml`; an OdometryPublisher provides ground truth
- `bridge.yaml`: ros_gz_bridge config onto the canonical topics (docs/interfaces.md)
- `ground_truth.json`, `info.json`

Deterministic: same layout + same seed → same world. Sensor sets: `none` (smoke test: no cameras),
`label` (RGB label camera), `full` (stereo, depth, RGB, camera IMU). The downward rangefinder is
always there: it is the EKF's height source and is wired into ArduPilotPlugin.

## Evaluation

`python3 -m evaluation --run-dir runs/latest --world sim_small` (from `sim/`, done by
`just sim-mission`) compares the inventory queue with `ground_truth.json`:

- **read recall**: items read / items on the shelves
- **false reads**: decoded item codes that are not on the shelves
- **correct-slot rate**: items attributed to the right slot / items read

The nightly CI job uploads the report; treat drops as regressions.

## Performance notes

- No GPU needed: Gazebo renders cameras with Mesa llvmpipe (`--headless-rendering`). It is slow, so
  CI renders only what it needs (`sensors:=none` for smoke, `label` for the mission at 5 Hz).
- SITL and Gazebo run in lock-step: a slow machine makes the sim slower, never wrong. Measured on
  Docker Desktop (WSL2, 16 cores, llvmpipe): the coupled loop runs at 0.3-0.45x real time with or
  without the rangefinder's render pass, and about 0.2x with the full ROS stack and the label
  camera. The stock empty world alone runs at 1.0x, so the cost is the 1 kHz lock-step exchange plus
  the drone model, not rendering.
- Every timeout inside the stack (`takeoff_timeout_s`, `goto_timeout_s`, ...) is in sim time. The
  launch tests' own budgets (`sim/tests/`) are wall-clock, sized for ~0.2x.
- Gazebo publishes `/clock` on every 1 kHz physics step, unthrottled, and every `use_sim_time`
  Python node handles each message: about 30% of a core per node during the scan mission (eleven
  nodes, ~3 cores). Harmless on a 16-core machine; on 4-vCPU CI runners it competes with Gazebo.
  TODO(phase-2): bridge Gazebo's clock to another topic and republish `/clock` at ~100 Hz with a
  C++ throttle (`topic_tools`), if CI timing gets tight.
- On macOS, Gazebo in Docker is headless only; use Foxglove for visualisation.
- Make it harder: `inv_localization/config/groundtruth.yaml` has noise, drift and dropout
  parameters to rehearse VSLAM error and the localization-lost failsafe.

## Isaac Sim

Not in scope now: see `sim/isaac/README.md`.
