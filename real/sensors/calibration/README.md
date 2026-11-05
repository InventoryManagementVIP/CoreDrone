# Calibration results

Kalibr camera-IMU calibration results, one directory per camera unit (by serial number), e.g.
`OAK-D-Lite_<serial>/2026-11-05/{camchain-imucam.yaml, imu.yaml, report.pdf}`.

After each calibration:
1. Copy the camera-to-camera and camera-to-IMU extrinsics into
   `drone/src/inv_description/urdf/sensors.xacro` (the single source of truth; the sim model is
   generated from it).
2. Copy the IMU noise densities and random walks into
   `drone/src/inv_localization/config/isaac_vslam.yaml`.
3. Open one PR with both changes and the calibration report, reviewed by the perception and sim
   CODEOWNERS.
