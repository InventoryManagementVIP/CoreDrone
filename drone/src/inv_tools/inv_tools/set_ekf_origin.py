"""Set ArduPilot's EKF origin over MAVLink. Needed for every GPS-denied flight.

Without GPS the EKF has no global origin, and ArduPilot will not arm in GUIDED with external
navigation until one is set. Called by both platform launches (sim/launch/sim_platform.launch.py,
real/launch/real_platform.launch.py) once the flight controller is up.

Mechanisms (verified against Copter-4.7.2, libraries/GCS_MAVLink/GCS_Common.cpp):
  1. COMMAND_INT MAV_CMD_DO_SET_GLOBAL_ORIGIN (611): acknowledged, so we know it worked. Preferred.
  2. SET_GPS_GLOBAL_ORIGIN message: legacy, no acknowledgement. Fallback for older firmware.
Either way we confirm by waiting for the GPS_GLOBAL_ORIGIN message the vehicle sends back.
A third option, the ahrs-set-origin.lua applet, needs scripting on the FC (docs/ardupilot.md).

The origin itself comes from the warehouse layout (geodetic_origin), so sim and real agree.

Usage:
  ros2 run inv_tools set_ekf_origin --connection tcp:127.0.0.1:5762 --warehouse sim_small
  ros2 run inv_tools set_ekf_origin --connection udpin:0.0.0.0:14551 --lat 40.4 --lon -86.9 --alt 190
"""

from __future__ import annotations

import argparse
import sys
import time

MAV_CMD_DO_SET_GLOBAL_ORIGIN = 611
# TCP only: pymavlink re-dials a dropped link this many times, 1 s apart, before raising. SITL
# drops the link on every reboot and can take several seconds to listen again.
TCP_RECONNECT_RETRIES = 60


def _origin_from_args(args) -> tuple[float, float, float]:
    if args.warehouse:
        from inv_layout import load_layout

        o = load_layout(args.warehouse).geodetic_origin
        return o.latitude_deg, o.longitude_deg, o.altitude_m
    if None in (args.lat, args.lon, args.alt):
        raise SystemExit("give --warehouse, or all of --lat --lon --alt")
    return args.lat, args.lon, args.alt


def wait_autopilot_heartbeat(master, timeout_s: float):
    """Wait for the autopilot's own heartbeat and aim pymavlink's target at it.

    pymavlink re-targets on every HEARTBEAT it receives, and ArduPilot forwards other peers'
    heartbeats (a GCS on another port, MAVProxy) to every channel, so `master.target_system` alone
    can point at the wrong system. Commands are therefore sent to the IDs returned here.
    """
    from pymavlink import mavutil

    deadline = time.monotonic() + timeout_s
    while (remaining := deadline - time.monotonic()) > 0:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=min(remaining, 1.0))
        if (
            msg is not None
            and msg.get_srcSystem() != 0
            and msg.type != mavutil.mavlink.MAV_TYPE_GCS
            and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            master.target_system, master.target_component = msg.get_srcSystem(), msg.get_srcComponent()
            return msg
    return None


def _wait_origin(master, lat: float, lon: float, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = master.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=1.0)
        if msg is not None and abs(msg.latitude * 1e-7 - lat) < 1e-6 and abs(msg.longitude * 1e-7 - lon) < 1e-6:
            return True
    return False


def set_origin(
    connection: str,
    lat: float,
    lon: float,
    alt_m: float,
    timeout_s: float = 120.0,
    source_system: int = 245,
    log=print,
    master=None,
) -> bool:
    """Set and confirm the EKF origin. Pass `master` to reuse an open pymavlink connection."""
    from pymavlink import mavutil

    deadline = time.monotonic() + timeout_s
    if master is None:
        log(f"connecting to {connection}")
    while master is None:
        try:
            master = mavutil.mavlink_connection(
                connection, source_system=source_system, autoreconnect=True, retries=TCP_RECONNECT_RETRIES
            )
        except OSError as exc:  # e.g. SITL has not opened its TCP port yet
            if time.monotonic() > deadline:
                log(f"cannot connect: {exc}")
                return False
            time.sleep(2.0)
    while (heartbeat := wait_autopilot_heartbeat(master, timeout_s=5.0)) is None:
        if time.monotonic() > deadline:
            log("no heartbeat from the flight controller")
            return False
        log("waiting for heartbeat...")
    target = (heartbeat.get_srcSystem(), heartbeat.get_srcComponent())
    log(f"heartbeat from system {target[0]}; setting EKF origin to {lat:.7f}, {lon:.7f}, {alt_m:.1f} m")

    lat_e7, lon_e7 = round(lat * 1e7), round(lon * 1e7)
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        # The EKF only accepts an origin once it is initialised, so keep trying for a while.
        master.mav.command_int_send(
            *target,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            MAV_CMD_DO_SET_GLOBAL_ORIGIN,
            0,
            0,
            0,
            0,
            0,
            0,
            lat_e7,
            lon_e7,
            float(alt_m),
        )
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=2.0)
        accepted = ack is not None and ack.command == MAV_CMD_DO_SET_GLOBAL_ORIGIN and ack.result == 0
        if not accepted:
            master.mav.set_gps_global_origin_send(target[0], lat_e7, lon_e7, round(alt_m * 1000))
        # Ask for GPS_GLOBAL_ORIGIN explicitly so we do not depend on stream rates.
        master.mav.command_long_send(
            *target,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        if _wait_origin(master, lat, lon, timeout_s=3.0):
            log(f"EKF origin set (attempt {attempt}, {'COMMAND_INT' if accepted else 'SET_GPS_GLOBAL_ORIGIN'})")
            return True
        log(f"origin not confirmed yet (attempt {attempt}), retrying")
        time.sleep(2.0)
    log("gave up: EKF origin not confirmed")
    return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="tcp:127.0.0.1:5762", help="pymavlink connection string")
    parser.add_argument("--warehouse", help="warehouse layout name or path (uses its geodetic_origin)")
    parser.add_argument("--lat", type=float)
    parser.add_argument("--lon", type=float)
    parser.add_argument("--alt", type=float, help="metres above mean sea level")
    parser.add_argument("--timeout", type=float, default=120.0)
    # ros2 launch passes --ros-args ...; ignore anything after it.
    argv = sys.argv[1:] if argv is None else argv
    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]
    args = parser.parse_args(argv)
    lat, lon, alt = _origin_from_args(args)
    return (
        0
        if set_origin(
            args.connection, lat, lon, alt, args.timeout, log=lambda m: print(f"[set_ekf_origin] {m}", flush=True)
        )
        else 1
    )


if __name__ == "__main__":
    sys.exit(main())
