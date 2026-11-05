# Interfaces: topics, actions and frames

This is the contract between packages. Drivers (real) and the Gazebo bridge (sim) remap onto these
names in `real/launch/` and `sim/launch/` only, so swapping a camera or a simulator never touches
the autonomy code. Message definitions live in `drone/src/inv_interfaces`.

## Topics

| Purpose | Name | Type | Producer |
|---|---|---|---|
| Stereo pair (VSLAM) | `/camera/left/image_raw`, `/camera/right/image_raw`, `/camera/{left,right}/camera_info` | `sensor_msgs/Image`, `CameraInfo` | driver (real) / gz bridge (sim, `sensors:=full`) |
| Depth | `/camera/depth/image_raw`, `/camera/depth/camera_info` | `sensor_msgs/Image`, `CameraInfo` | driver / gz bridge (`full`) |
| Label camera (RGB) | `/camera/color/image_raw`, `/camera/color/camera_info` | `sensor_msgs/Image`, `CameraInfo` | driver / gz bridge (`label`, `full`) |
| Camera IMU | `/camera/imu` | `sensor_msgs/Imu` | driver / gz bridge (`full`) |
| Downward range | `/rangefinder/scan` | `sensor_msgs/LaserScan` | gz bridge (sim; the FC reads its own rangefinder) |
| Sim ground truth | `/sim/ground_truth/odom` | `nav_msgs/Odometry` | gz OdometryPublisher (sim only) |
| Localization (backend output) | TF `map→odom→base_link`, `/localization/odom` | TF, `nav_msgs/Odometry` | localization backend |
| Localization health | `/localization/status` | `inv_interfaces/LocalizationStatus` | localization adapter |
| External nav to the FC | `/localization/external_nav` | `nav_msgs/Odometry` (odom→base_link, best effort) | localization adapter → `inv_flight_interface` |
| Obstacles | `/mapping/blocked` (+ nvblox native topics) | `inv_interfaces/CorridorStatus` | mapping |
| Detections | `/perception/detections` | `inv_interfaces/LabelDetectionArray` | perception (one array per detector per image, image stamp) |
| Inventory | `/inventory/items` | `inv_interfaces/InventoryItemArray` | inventory |
| Mission state + heartbeat | `/mission/status` | `inv_interfaces/MissionStatus` | mission (2 Hz) |
| Flight controller state | `/flight/state` | `inv_interfaces/FlightState` | flight interface (20 Hz) |
| Velocity passthrough | `/flight/cmd_vel` | `geometry_msgs/TwistStamped` (`map` or `base_link`) | teleop/tools → flight interface (watchdog: stop after 0.5 s, hold after 2 s) |
| Safety | `/safety/status` | `inv_interfaces/SafetyStatus` | safety monitor |
| Uplink health | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | uplink |

## Flight commands (`inv_flight_interface`)

| Name | Kind | Type | Notes |
|---|---|---|---|
| `/flight/takeoff` | action | `inv_interfaces/Takeoff` | GUIDED, arm, take off, climb; waits for FC link, EKF origin, external nav, frame alignment |
| `/flight/goto` | action | `inv_interfaces/GoTo` | target in `map`; position + yaw; keeps holding the target afterwards |
| `/flight/hover` | action | `inv_interfaces/Hover` | hold position for `duration_s` |
| `/flight/land` | action | `inv_interfaces/Land` | LAND mode; succeeds when disarmed |
| `/flight/abort` | service | `inv_interfaces/Abort` | hold 2 s, then land |
| `/uplink/export_csv` | service | `inv_interfaces/ExportCsv` | CSV of queued scan records |

**Rule:** only `inv_flight_interface` names ArduPilot's `/ap/*` interfaces. CI enforces it
(`drone/src/inv_flight_interface/test/test_ap_topics_confined.py`). Platform code in `sim/` and
`real/` may observe `/ap/*` for diagnostics; the autonomy stack may not.

## Frames

- ENU world frames, REP-103 / REP-105: `map` (warehouse layout coordinates, z = 0 on the floor),
  `odom` (drift-free locally, continuous), `base_link` (x forward, y left, z up; the FC body frame).
- Optical frames (`*_optical_frame`) are z forward, x right, y down.
- Sensor extrinsics have one source of truth: `drone/src/inv_description/urdf/sensors.xacro`. The
  sim drone model is generated from it (`sim/worldgen/drone_model.py`); never copy numbers by hand.

| Frame | Parent | Published by |
|---|---|---|
| `map` → `odom` | — | localization backend (identity for `groundtruth`) |
| `odom` → `base_link` | `odom` | localization backend |
| `base_link` → `camera_link`, `camera_{left,right,depth,color,imu}`, their `*_optical_frame`s, `rangefinder_link`, `flow_link`, `fc_imu_link` | `base_link` | `robot_state_publisher` (URDF, static) |
| `tag36h11:<id>` | camera optical frame | `apriltag_ros` |

## Labels

- Location labels: one per slot, on the beam under it. Sim: AprilTag tag36h11, IDs assigned in
  slot order from the layout (`first_id`). Detected codes look like `tag36h11:5`.
- Item labels: QR (sim); 1D barcodes are supported by the reader but not generated in sim yet.
- `size_m` in a layout is the code itself without the white quiet zone (AprilTag: black square;
  QR: symbol). Detectors use it for pose estimation.
