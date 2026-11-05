# Sensors

**The camera is the Luxonis OAK-D Lite (real/BOM.md). Nothing here has been run on one.**

| | Stereo (VSLAM, depth) | RGB (labels) | IMU |
|---|---|---|---|
| Sensor | OV7251 pair, global shutter | IMX214, rolling shutter | BMI270, 6-axis |
| Resolution used | 640x480 at 30 fps | 1920x1080 at 15 fps | |
| Horizontal FOV | 73° | 69° | |
| Other | 75 mm baseline; ideal depth 0.4-8 m | fixed focus: 50 cm and beyond; auto-focus: 8 cm and beyond | absent on early Kickstarter units |

## Test on the bench, before flying

1. **VSLAM on this camera.** Isaac ROS documents RealSense, ZED and Hawk/Eagle, not OAK. Luxonis ships
   an Isaac VSLAM config in its driver, which this setup follows. Carry the running drone along a
   real aisle and back, props off: `/localization/status` must stay TRACKING and the pose must
   return close to the start. Aisles are repetitive and low-texture, and this is a low-resolution
   pair, so an office test does not settle it.
2. **Camera-IMU sync.** `tracking_mode: 1` (visual-inertial) needs well-synced timestamps. Check them
   with Kalibr. If they are poor, try `tracking_mode: 0` (stereo only).
3. **Topics.** `i_publish_synced_rect_pair` comes from Luxonis's config for the `depth` pipeline; here
   it runs in `RGBD`. Confirm the left, right, RGB and IMU topics all publish, and that the frame
   names match `OAK_FRAMES` in `real_platform.launch.py`.
4. **Label reading at real sizes.** Sim labels are large: 180 mm QR codes. Pixels per module at
   1080p and 1.0 m are about 1400 x (module size in m), so a 30 mm QR with 25 modules gets about
   1.7 px per module, which is too few. Measure with real labels, then shorten `scan.standoff_m`
   or print larger labels. Fixed focus blurs under 0.5 m.

## Where things go

- Driver config: `oak_d_lite.yaml`. The canonical topic remaps and frame aliases are in
  `real/launch/real_platform.launch.py`; autonomy code never sees driver topic names.
- udev rules: `udev/99-coredrone.rules`, installed by `real/jetson/setup.sh`.
- Calibration: `calibration/` (Kalibr camera-IMU results). Extrinsics then go into
  `drone/src/inv_description/urdf/sensors.xacro` (the single source of truth) and the IMU noise
  model into `drone/src/inv_localization/config/isaac_vslam.yaml`.
