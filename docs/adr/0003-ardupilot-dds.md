# 0003: ArduPilot Copter + AP_DDS, behind one flight-interface package

- Status: Accepted
- Date: 2026-10-20

## Context

The airframe is a Holybro X500 V2 Quad-X with a Pixhawk 6C running ArduPilot Copter (real/BOM.md).
ArduPilot ships a native ROS 2 interface (AP_DDS over Micro XRCE-DDS) next to MAVLink. Pinned:
Copter-4.7.2, the latest stable release (11 Oct 2026). Details in docs/ardupilot.md.

## Decision

- ROS 2 talks to ArduPilot through AP_DDS: the micro-ROS agent on the Jetson, UDP in sim, a serial
  port on the vehicle. A MAVLink link to the ground station is kept in parallel (MAVProxy in sim,
  mavlink-router on the Jetson) for parameters, logs and the safety pilot.
- **Only `inv_flight_interface` names `/ap/*`.** Mission and everything else use its
  `/flight/*` actions and topics. CI enforces this. The FC-specific code sits behind a
  `FlightBackend` interface (`ap_dds_backend.py`), so switching to MAVROS is a change inside one
  package.
- Platform tools that need MAVLink (EKF origin, DDS recovery, parameter diff) live in `inv_tools`
  and use MAVLink only.

## Alternatives considered

- **MAVROS** as the primary link: mature and widely used, but adds a MAVLink↔ROS translation layer
  and its own parameter plugins. Kept as the fallback for flight controllers that cannot run DDS
  (flash space or no spare UART/Ethernet).
- **PX4**: different team experience and parameters; not on the table.

## Consequences

Verified in SITL; each of these needed code in the flight interface or tools:

- AP_DDS feeds external-nav stamps to the EKF as FC boot-clock times without conversion: the
  flight interface restamps them from `/ap/time`.
- The FC's map frame is our odom rotated +90°, and `base_link` velocity commands treat y as
  right: we command in `map` with a rotation learned at runtime.
- AP_DDS gives up for good if the agent is slower than 500 ms creating its participant:
  `fc_bootstrap` reboots the FC (disarmed only) when that happens.
- SITL's JSON backend at 4.7.2 drops rangefinder data: patched (SITL only) in `deps/patches/`.

Some flight controllers need a custom firmware build with DDS enabled. MAVLink and DDS run at the
same time, so the GCS keeps working.

## Revisit when

The chosen flight controller cannot run DDS (switch the backend to MAVROS inside
`inv_flight_interface`), or a new ArduPilot release fixes the timestamp/frame/race issues above
(then simplify the workarounds and drop the patch).
