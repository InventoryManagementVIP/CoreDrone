# Flight controller

**Holybro Pixhawk 6C with PM02 (real/BOM.md), ArduPilot Copter-4.7.2. Not yet flashed or bench-tested.**

## Firmware

The board ships with PX4, and its ArduPilot hwdef does not enable DDS, so build the firmware:

```bash
just fc-firmware      # output: runs/firmware/arducopter.apj
```

Flash it as custom firmware from Mission Planner or QGroundControl, and record the hash here. A
trial build used 1,632,972 bytes of flash with 333,100 free. If DDS proves impossible: MAVROS
fallback, implemented inside `inv_flight_interface` only (docs/adr/0003-ardupilot-dds.md).

## Wiring

| FC port | Parameter prefix | Protocol | To | Baud |
|---|---|---|---|---|
| USB | `SERIAL0_*` | MAVLink | Jetson USB (`/dev/flight_controller`), mavlink-router | n/a |
| TELEM1 | `SERIAL1_*` | 2 (MAVLink2) | SiK telemetry radio | 57600 |
| TELEM2 | `SERIAL2_*` | 45 (DDS) | Jetson 40-pin UART (TODO(hardware): confirm the device, default `/dev/ttyTHS1`) | 921600 |

Cross TX/RX and share ground. Check that the logic levels match before connecting. The micro-ROS
agent side is `real/launch/real_platform.launch.py` (`dds_device`, `dds_baud`). PM02 powers the FC
only: the Jetson needs its own feed (real/BOM.md).

## Parameters

`params/common.parm` (shared with SITL) + `params/real.parm` (vehicle) + per-airframe tuning files.
Every change goes through a PR; `just params-diff CONNECTION=udpin:0.0.0.0:14551` shows what differs
on a live FC. See docs/ardupilot.md for what each block does and why.

## Bring-up checklist (first time on the bench, props off)

1. Flash the DDS build; load `common.parm` + `real.parm`; reboot.
2. Accelerometer calibration. Calibrate the kit's ESCs; check motor order and direction.
3. Radio calibration; check the mode switch and the motor kill switch.
4. Start the agent and `fc_bootstrap`; `/flight/state` on the Jetson shows `connected: true`.
5. With VSLAM running, `EKF3 IMU0 is using external nav data` appears in the GCS messages and
   `/flight/state.external_nav_ok` is true.
6. Measure `VISO_DELAY_MS` (docs/ardupilot.md) and update `real.parm`.
