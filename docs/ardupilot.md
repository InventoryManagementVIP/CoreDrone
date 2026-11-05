# ArduPilot integration

Pinned: **ArduPilot Copter-4.7.2** (stable since 11 Oct 2026, commit `fb77356`), with AP_DDS
(Micro XRCE-DDS) as the ROS 2 link. Everything below was checked against the source at that tag;
re-check it when bumping (deps/README.md).

## DDS link (AP_DDS)

- `DDS_ENABLE 1`. `DDS_DOMAIN_ID` must equal `ROS_DOMAIN_ID` (sim sets it at launch from the
  environment; on the vehicle set both per drone).
- **Sim:** UDP to the micro-ROS agent in the same container, port `DDS_UDP_PORT 2019`
  (`micro_ros_agent udp4 -p 2019`).
- **Real:** a serial port to the agent on the Jetson, `SERIALn_PROTOCOL 45` and a matching baud
  (`micro_ros_agent serial --dev /dev/ttyTHS1 -b 921600`). Some boards need a custom build with DDS
  enabled (flash); see real/flight_controller/README.md. Fallback if DDS is not possible: MAVROS,
  implemented as a second backend inside `inv_flight_interface` only.
- Topics/services the stack uses (QoS from `AP_DDS_Topic_Table.h`): `/ap/status` (reliable,
  transient local), `/ap/time` (reliable), `/ap/battery`, `/ap/pose/filtered`,
  `/ap/gps_global_origin/filtered` (best effort); publishes `/ap/tf` and `/ap/cmd_vel` (best
  effort); services `/ap/mode_switch`, `/ap/arm_motors`, `/ap/experimental/takeoff`.

### DDS start-up race (affects sim and real)

`AP_DDS_Client::create()` gives the agent a hard-coded **500 ms** to create its DDS participant. On a
busy machine the agent's first participant can take longer (we measured ~570 ms in the dev
container while Gazebo was loading). AP_DDS then reports `DDS: Creation Requests failed` and its
DDS thread exits until the next reboot: no `/ap/*` data at all. `inv_tools/fc_bootstrap` watches the
FC's STATUSTEXT for that and reboots the FC (only ever while disarmed); the agent is warm by then.
On the vehicle it also reboots once at start-up if it could not observe the outcome
(`--reboot-if-unknown`), because the FC usually boots before the Jetson.

Two MAVLink details the MAVLink tools (`fc_bootstrap`, `set_ekf_origin`) handle:

- ArduPilot forwards every peer's HEARTBEAT to all its other links (MAVProxy's GCS heartbeat shows
  up on SERIAL1), and pymavlink re-targets `master.target_system` on every heartbeat it receives.
  The tools lock onto the first heartbeat from an actual autopilot and address commands to it.
- A SITL reboot drops its TCP links and SITL can take several seconds to listen again; pymavlink
  re-dials a TCP link only 3 times by default, so the tools allow 60. (On the vehicle they use
  mavlink-router's UDP output, which has no connection to drop.)

## External navigation (VSLAM → EKF3)

- `AHRS_EKF_TYPE 3`, `GPS1_TYPE 0`, `VISO_TYPE 1`.
- Source set 1: `EK3_SRC1_POSXY 6` (ExternalNav), `EK3_SRC1_YAW 6`, `EK3_SRC1_POSZ 2`
  (rangefinder), `EK3_SRC1_VELXY 0` and `EK3_SRC1_VELZ 0` because `/ap/tf` carries a pose only
  (AP_DDS calls `handle_pose_estimate`; there is no velocity path over DDS).
- Source set 2 (RC switch, `RCx_OPTION 90`): optical flow + rangefinder (`EK3_SRC2_VELXY 5`,
  `EK3_SRC2_POSZ 2`, yaw from compass). Indoors the compass is weak; revisit once the flow sensor is
  fitted. The `ahrs-source-extnav-optflow.lua` applet can switch automatically.
  Arming fails with `EK3 sources require OpticalFlow` until a flow sensor is configured: intended.
- **Only `odom → base_link` is accepted** on `/ap/tf` (`AP_DDS_External_Odom::is_odometry_frame`).
  base_link is the FC body frame, so `VISO_POS_*` stay 0.
- **External navigation timestamps.** AP_DDS hands the transform's header stamp straight to the EKF
  as a time on the FC's *boot clock* (`time_ms = remote_time_us * 1E-3`, with a TODO for jitter
  correction, also still on master). ROS stamps are UTC on the Jetson and `/clock` in sim, so
  unconverted stamps are never fused. `inv_flight_interface` learns the offset from `/ap/time`
  (the FC's boot clock while its RTC is unset) and restamps every external-nav sample.
  **Keep `BRD_RTC_TYPES 1`** (GPS only): if the RTC is set from MAVLink `SYSTEM_TIME`, `/ap/time`
  becomes UTC and this mapping breaks; the flight interface then refuses to forward.
- `VISO_DELAY_MS 10` covers only residual latency, since stamps are capture times. Re-measure on
  hardware (compare the `VISP` and `IMU` log messages).
- **Frame convention.** AP_DDS treats the external-nav odom frame as x-forward/y-left and maps its
  +x to North (`convert_transform`). Its `/ap/pose/filtered` and its `map`-frame velocity commands
  are ENU built from that North, so the FC's "map" frame is our odom frame **rotated +90°**. The
  flight interface measures this rotation at start-up (FC yaw vs. the yaw we feed it) and refuses to
  take off until it has (log: `FC map frame = odom rotated +90 deg`), so a future change in
  ArduPilot cannot make the drone fly sideways.

## Commanding

- GUIDED (`/ap/mode_switch` mode 4) → `/ap/arm_motors` → `/ap/experimental/takeoff` (altitude
  above home) → velocity setpoints on `/ap/cmd_vel`.
- `/ap/cmd_vel` frame semantics at 4.7.2 (`AP_DDS_External_Control::handle_velocity_control`):
  `frame_id: map` is ENU in the FC's map frame (see above); `frame_id: base_link` converts
  `(x, y, -z)` with `body_to_earth`, i.e. **y is treated as right, not left** (REP-103 says left).
  The flight interface therefore always uses `map` plus the measured rotation. Any other frame id
  is ignored.
- GoTo/Hover are closed-loop on our own localization (TF `map→base_link`) with speed, acceleration
  and geofence limits; the FC's position controller does the inner loops. If setpoints stop,
  `GUID_TIMEOUT 3` makes the FC stop by itself (second line of defence after our watchdogs).
- LAND (mode 9) to land; the FC disarms on touchdown.

## EKF origin (no GPS)

Without GPS the EKF has no origin and Copter will not arm in GUIDED. `fc_bootstrap` (and the
standalone `set_ekf_origin`) send **`MAV_CMD_DO_SET_GLOBAL_ORIGIN` (611) as COMMAND_INT**, which is
acknowledged at 4.7.2, falling back to the legacy `SET_GPS_GLOBAL_ORIGIN` message, and confirm via
`GPS_GLOBAL_ORIGIN`. The origin is the layout's `geodetic_origin`, so sim (`SITL --home`) and the
vehicle agree. Alternative: the `ahrs-set-origin.lua` applet (`AHRS_ORIG_LAT/LON/ALT`) if scripting
fits on the FC. Every reboot clears the origin; `fc_bootstrap` sets it after any reboot it causes.

## Parameter files are code

```
real/flight_controller/params/common.parm   shared base (EKF sources, VISO, DDS, limits, failsafes)
├── sim/params/sim.parm                      SITL: no GPS sim, SITL rangefinder/flow, no RC failsafe
└── real/flight_controller/params/real.parm  vehicle (real/BOM.md): serial DDS, VSLAM height, battery
```

- Every change goes through a PR. `just params-diff CONNECTION=...` compares a live FC with the
  files; `just params-check` (CI) verifies every name exists in the pinned firmware. Copter 4.7
  renamed many parameters, mostly to SI units. The ones these files use:

  | Copter 4.6 | Copter 4.7 |
  |---|---|
  | `ARMING_CHECK` | `ARMING_SKIPCHK` (inverted: a bitmask of checks to skip) |
  | `GPS_TYPE` | `GPS1_TYPE` |
  | `SIM_GPS_DISABLE` | `SIM_GPS1_ENABLE` |
  | `RNGFND1_MAX_CM` | `RNGFND1_MAX` (m) |
  | `WPNAV_SPEED`, `WPNAV_SPEED_UP`, `WPNAV_SPEED_DN` | `WP_SPD`, `WP_SPD_UP`, `WP_SPD_DN` (m/s) |
  | `LAND_SPEED` | `LAND_SPD_MS` (m/s) |
  | `PILOT_SPEED_UP` | `PILOT_SPD_UP` (m/s) |
  | `RTL_ALT` | `RTL_ALT_M` (m) |

  The firmware skips unknown names in a defaults file without a word (the warning in
  `AP_Param::read_param_defaults_file` is compiled only with `ENABLE_DEBUG`), which is why the CI check
  exists. Parameters that register late (the `DDS_*` group appears only after
  "ArduPilot Ready") are missing from the bulk list, so the checker asks for each missing name
  directly before calling it unknown.
- `FS_EKF_ACTION 1` lands if the EKF loses its position (e.g. VSLAM lost and no switch to set 2).

## SITL specifics

- Gazebo physics via `--model JSON` and ardupilot_gazebo's ArduPilotPlugin, lock-step.
- GPS is disabled in SITL too (`SIM_GPS1_ENABLE 0`): a simulated GPS would hide exactly the
  external-nav path we need to test.
- The rangefinder is a Gazebo ray sensor forwarded as JSON `rng_1`. Copter-4.7.2's JSON backend
  mis-indexes those fields; `deps/patches/ardupilot/0001-sitl-json-rangefinder-bitmask.patch`
  backports the fix from master (SITL code only).
- SITL must be started by absolute path: a reboot re-execs `argv[0]`.
- `SIM_IMU_COUNT 1`: with SITL's second IMU, arming waited minutes for "Gyros/Accels inconsistent"
  to clear after spawn. A single IMU also needs accel 2's calibration cleared (SITL's
  `copter.parm` sets it), or arming fails with "3D Accel calibration needed".
- MAVProxy prints the FC's messages (`AP: PreArm: ...`) into the launch log: look there first when
  the vehicle will not arm. `sim_platform.launch.py` refuses to start while another simulation
  still holds the SITL, JSON or DDS ports.

## Ground station

MAVProxy (sim) or mavlink-router (Jetson) forwards MAVLink to Mission Planner / QGroundControl for
parameters, logs and the safety pilot's situational awareness. In the dev container MAVProxy sends
to `udpout:host.docker.internal:14550`, so a GCS on the host just listens on UDP 14550.
