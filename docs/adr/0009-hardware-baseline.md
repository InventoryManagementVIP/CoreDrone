# 0009: Hardware baseline

- Status: Accepted
- Date: 2026-11-05

## Context

The team chose its parts (real/BOM.md): Holybro Pixhawk 6C with PM02, Jetson Orin Nano Super
Developer Kit 8 GB, Luxonis OAK-D Lite, Holybro X500 V2 ARF kit, a SiK telemetry radio and a battery.
The repo had been written against a generic Pixhawk, an unknown Jetson and a RealSense-shaped camera.

## Decision

- **Firmware**: Copter-4.7.2 built with `--enable-DDS` (`just fc-firmware`), because the board's
  hwdef does not enable DDS. The build fits with 333 KB of flash to spare.
- **Camera**: `depthai_ros_driver` replaces the RealSense driver, remapped to the canonical topics
  in `real_platform.launch.py`. The OV7251 global-shutter pair at 640x480 feeds VSLAM; RGB at 1080p
  reads labels. The URDF and the sim's camera model match the OAK-D Lite's geometry.
- **Height**: no rangefinder is on the BOM, so `real.parm` takes EKF height from VSLAM. Simulation
  keeps the rangefinder until the team decides (see Consequences).
- **VSLAM stays Isaac ROS**, on a camera NVIDIA does not document, pending the bench test in
  `real/sensors/README.md`. ROS 2 Jazzy, JetPack 7.2 and Isaac ROS 4.6 are unchanged (ADR 0002).

## Alternatives considered

- **Keep a RealSense for VSLAM**: on NVIDIA's tutorial list, but not the part the team chose.
- **Barometer for height**: indoors it suffers from HVAC and prop wash near shelves.

## Consequences

- Luxonis ships a driver config for Isaac VSLAM, which is encouraging but not proof. The bench
  test comes before any flight.
- 8 GB shared between CPU and GPU may not fit cuVSLAM, nvblox and label decoding together, so the
  onboard default stays `mapping:=none`. Measure before enabling nvblox.
- VSLAM height fails together with VSLAM. A rangefinder is recommended; with one, `real.parm` loses
  its height override and the vehicle matches simulation again.
- Without optical flow there is no position fallback: losing VSLAM means landing.

## Revisit when

The VSLAM bench test fails on this camera, or a rangefinder or flow sensor is added.
