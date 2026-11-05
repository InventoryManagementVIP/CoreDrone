"""Bring the flight controller to a known-good state before the stack flies. MAVLink only.

  1. wait for the FC's heartbeat
  2. make sure its DDS link to the micro-ROS agent came up
  3. set the EKF origin (needed without GPS; lost on every reboot)

Why step 2 exists (docs/ardupilot.md, "DDS start-up race"): at Copter-4.7.2 AP_DDS gives the XRCE
agent a hard-coded 500 ms to create its DDS participant (AP_DDS_Client::create). The agent's first
participant can take longer (Fast DDS start-up on a busy machine: we measured ~570 ms in the dev
container while Gazebo was loading). AP_DDS then reports "Creation Requests failed" and its DDS
thread exits until the next reboot. The agent is warm by then, so one reboot fixes it. We only
ever reboot while disarmed.

Called by sim/launch/sim_platform.launch.py and real/launch/real_platform.launch.py.
  ros2 run inv_tools fc_bootstrap --connection tcp:127.0.0.1:5762 --warehouse sim_small
"""

from __future__ import annotations

import argparse
import sys
import time

from inv_tools.set_ekf_origin import TCP_RECONNECT_RETRIES, set_origin, wait_autopilot_heartbeat

DDS_OK = ("DDS: Initialization passed",)
DDS_FAILED = (
    "DDS: Creation Requests failed",
    "DDS: No ping response, exiting",
    "DDS: transport invalid, exiting",
    "DDS: Transport initialization failed",
)


def dds_outcome(text: str) -> bool | None:
    """True / False when a STATUSTEXT reports DDS success / permanent failure, else None."""
    if any(marker in text for marker in DDS_OK):
        return True
    if any(marker in text for marker in DDS_FAILED):
        return False
    return None


def _connect(connection: str, deadline: float, log):
    from pymavlink import mavutil

    while True:
        try:
            return mavutil.mavlink_connection(
                connection, source_system=244, autoreconnect=True, retries=TCP_RECONNECT_RETRIES
            )
        except OSError as exc:
            if time.monotonic() > deadline:
                log(f"cannot connect to {connection}: {exc}")
                return None
            time.sleep(2.0)


def _armed(heartbeat) -> bool:
    from pymavlink import mavutil

    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def _send_gcs_heartbeat(master) -> None:
    from pymavlink import mavutil

    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def wait_for_dds(master, target: tuple[int, int], timeout_s: float, log) -> tuple[bool | None, object]:
    """Read STATUSTEXT until DDS reports success or failure. Returns (outcome, last FC heartbeat)."""
    deadline = time.monotonic() + timeout_s
    heartbeat = None
    last_sent = 0.0
    while time.monotonic() < deadline:
        # ArduPilot only sends STATUSTEXT on MAVLink channels that have heard from a peer.
        if time.monotonic() - last_sent >= 1.0:
            _send_gcs_heartbeat(master)
            last_sent = time.monotonic()
        msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "HEARTBEAT":
            if (msg.get_srcSystem(), msg.get_srcComponent()) == target:  # the FC, not a forwarded GCS
                heartbeat = msg
            continue
        outcome = dds_outcome(msg.text)
        if outcome is not None:
            log(f"FC says: {msg.text}")
            return outcome, heartbeat
    return None, heartbeat


def reboot(master, target: tuple[int, int], log) -> None:
    from pymavlink import mavutil

    log("rebooting the flight controller (disarmed) to retry DDS")
    master.mav.command_long_send(
        *target,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def bootstrap(
    connection: str,
    lat: float,
    lon: float,
    alt_m: float,
    *,
    max_reboots: int = 3,
    dds_timeout_s: float = 60.0,
    reboot_if_unknown: bool = False,
    timeout_s: float = 300.0,
    log=print,
) -> bool:
    deadline = time.monotonic() + timeout_s
    master = _connect(connection, deadline, log)
    if master is None:
        return False
    _send_gcs_heartbeat(master)
    heartbeat = wait_autopilot_heartbeat(master, timeout_s=max(1.0, deadline - time.monotonic()))
    if heartbeat is None:
        log("no heartbeat from the flight controller")
        return False
    target = (heartbeat.get_srcSystem(), heartbeat.get_srcComponent())
    log(f"heartbeat from system {target[0]}; waiting for the DDS link")

    reboots = 0
    while True:
        outcome, latest = wait_for_dds(master, target, dds_timeout_s, log)
        if outcome:
            break
        heartbeat = latest or heartbeat  # judge "armed" on the newest heartbeat we have seen
        armed = _armed(heartbeat)
        if outcome is None and not reboot_if_unknown:
            log("DDS status not seen (FC booted before we connected); assuming it is up")
            break
        if armed:
            log("DDS is not up but the vehicle is ARMED: not rebooting")
            return False
        if reboots >= max_reboots:
            log(f"DDS still failing after {reboots} reboots; check the agent and DDS_* parameters")
            return False
        reboot(master, target, log)
        reboots += 1
        reboot_if_unknown = False  # after our own reboot we will see the boot messages
        time.sleep(3.0)
        heartbeat = wait_autopilot_heartbeat(master, timeout_s=60.0)
        if heartbeat is None:
            log("no heartbeat after reboot")
            return False

    return set_origin(connection, lat, lon, alt_m, max(30.0, deadline - time.monotonic()), log=log, master=master)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="tcp:127.0.0.1:5762")
    parser.add_argument("--warehouse", required=True, help="layout whose geodetic_origin becomes the EKF origin")
    parser.add_argument("--max-reboots", type=int, default=3)
    parser.add_argument("--dds-timeout", type=float, default=60.0)
    parser.add_argument(
        "--reboot-if-unknown", action="store_true", help="reboot (disarmed only) when the DDS outcome was not observed"
    )
    argv = sys.argv[1:] if argv is None else argv
    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]
    args = parser.parse_args(argv)

    from inv_layout import load_layout

    o = load_layout(args.warehouse).geodetic_origin
    ok = bootstrap(
        args.connection,
        o.latitude_deg,
        o.longitude_deg,
        o.altitude_m,
        max_reboots=args.max_reboots,
        dds_timeout_s=args.dds_timeout,
        reboot_if_unknown=args.reboot_if_unknown,
        log=lambda m: print(f"[fc_bootstrap] {m}", flush=True),
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
