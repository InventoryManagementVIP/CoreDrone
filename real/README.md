# real/: the physical vehicle

Platform-specific configuration and deployment for the real drone. **Skeleton: the parts are chosen ([BOM.md](BOM.md)), but nothing
here has run on them.** Values still waiting on delivered hardware are marked `TODO(hardware)`.

The autonomy stack is the same as in simulation (`drone/`); only these things differ:

| Path | What | Status |
|---|---|---|
| `launch/real_platform.launch.py` | camera driver (remapped to canonical topics), micro-ROS agent on the FC serial port, `fc_bootstrap` | skeleton |
| `flight_controller/` | Pixhawk 6C firmware build, wiring, parameter files | params are code; firmware builds |
| `jetson/` | JetPack, setup script, onboard compose + systemd, chrony, mavlink-router | skeleton |
| `sensors/` | OAK-D Lite bench tests, driver config, udev rules, calibration results | skeleton |
| `flight_test/` | pre-flight checklist, test card and log templates | ready to use |

Start the vehicle stack (once set up): `systemctl start coredrone` on the Jetson, which runs
`ros2 launch inv_bringup drone.launch.py platform:=real ...` in the onboard container.

## Open hardware questions

[BOM.md](BOM.md) has the full list. Flight is blocked by three things not on the parts list: an **RC
transmitter and receiver**, **NVMe storage**, and **Jetson power from the battery**. Two more are
choices: a **downward rangefinder** (recommended) and **which OAK-D Lite variant** (fixed focus
recommended). The first bench test is **Isaac ROS VSLAM on this camera** (`sensors/README.md`).
