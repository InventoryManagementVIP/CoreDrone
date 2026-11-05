# Jetson (onboard computer)

**SKELETON: Jetson Orin Nano Super Developer Kit 8 GB (real/BOM.md). Nothing here has run on one.**

- **OS:** JetPack 7.2 (Jetson Linux 39.2, Ubuntu 24.04, CUDA 13.2). Required for Isaac ROS 4.6 on
  both Orin and Thor (docs/adr/0002-ros-distro.md). Use an NVMe root filesystem
  (not on the parts list yet). NVIDIA lists "Jetson Orin" without naming the Nano 8 GB: confirm on
  first boot. CPU and GPU share the 8 GB, so measure before enabling nvblox.
- **Runtime:** everything runs in the onboard image (`docker/jetson.Dockerfile`, derived from the
  Isaac ROS 4.6 `arm64-jetpack` base) started by systemd through `compose.yaml` here.

## Files

| File | Purpose |
|---|---|
| `setup.sh` | one-time host setup: Docker + NVIDIA runtime check, chrony, udev rules, mavlink-router, systemd unit, env file |
| `compose.yaml` | the onboard container: host network, FC serial port, camera USB, GPU |
| `systemd/coredrone.service` | starts the stack at boot, restarts it if it dies |
| `chrony.conf` | time sync (EKF fusion needs one consistent clock; see below) |
| `mavlink-router.conf` | FC telemetry → GCS (UDP) and the local endpoint `fc_bootstrap` uses |

## Time

The stack timestamps with the Jetson clock (UTC via chrony). ArduPilot runs on its own boot clock;
`inv_flight_interface` converts external-nav stamps into it using `/ap/time`
(docs/ardupilot.md). chrony keeps the Jetson's clock steady (no jumps mid-flight); keep the FC's
`BRD_RTC_TYPES 1` so `/ap/time` stays the boot clock.

## Networking

ROS 2 uses the host network on the vehicle. Give every drone and every developer laptop on the same
network its own `ROS_DOMAIN_ID` (and the same value in `DDS_DOMAIN_ID` on that drone's FC).
