# Onboarding: Jetson (the vehicle)

**Nothing here has run on hardware yet.** The computer is the Orin Nano Super Developer Kit 8 GB
(real/BOM.md); this is the plan.

1. Flash **JetPack 7.2** (Jetson Linux 39.2, Ubuntu 24.04). ROS 2 Jazzy + Isaac ROS 4.6 needs it
   (docs/adr/0002-ros-distro.md). Fit an NVMe SSD first.
2. Run `real/jetson/setup.sh` (Docker, NVIDIA container runtime, chrony, udev rules, mavlink-router,
   the systemd unit).
3. Build or pull the onboard image (`docker/jetson.Dockerfile`).
4. Build and flash the FC firmware (`just fc-firmware`), wire the FC
   (real/flight_controller/README.md), set parameters, `just params-diff`.
5. Follow `real/flight_test/checklist.md` for every flight. A safety pilot with an RC transmitter
   (mode switch + motor kill) is mandatory.

See `real/README.md` for the details and the open hardware questions.
