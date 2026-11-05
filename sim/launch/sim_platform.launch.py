"""Simulation platform: Gazebo Harmonic + ArduPilot SITL (JSON physics, AP_DDS) + micro-ROS agent
+ ros_gz bridges to the canonical topics + EKF origin setter.

Only platform things live here. The autonomy stack (inv_bringup drone.launch.py) is the same in
sim and on the real vehicle; drone.launch.py platform:=sim includes this file.

  world:=sim_small        warehouse layout; generated on first use (just worldgen W=...)
  sensors:=label          sensor set from sim/models/inv_quad/sensors.yaml (none | label | full)
  headless:=true          Gazebo server only (macOS, CI); false opens the Gazebo GUI (Linux, WSLg)
  speedup:=1              SITL speedup; Gazebo runs in lock-step with SITL
  wipe:=true              reload parameters from the .parm files on every start
  gcs_out:=               extra MAVProxy output for a ground station, e.g. udpout:host.docker.internal:14550
  run_dir:=/tmp/coredrone/sitl   SITL working directory (eeprom.bin, dataflash logs)
"""

from __future__ import annotations

import json
import os
import shutil
import socket
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_DIR = Path(__file__).resolve().parents[1]
ROOT = Path(os.environ.get("INV_ROOT", SIM_DIR.parent))
GENERATED = SIM_DIR / "worlds" / "generated"
COMMON_PARM = ROOT / "real" / "flight_controller" / "params" / "common.parm"
SIM_PARM = SIM_DIR / "params" / "sim.parm"
DDS_UDP_PORT = 2019
SITL_MAVLINK_SERIAL1 = "tcp:127.0.0.1:5762"
# SITL serial ports 0-2 (TCP), the ArduPilotPlugin's JSON port and the micro-ROS agent (UDP).
SIM_PORTS = (
    (socket.SOCK_STREAM, 5760),
    (socket.SOCK_STREAM, 5762),
    (socket.SOCK_STREAM, 5763),
    (socket.SOCK_DGRAM, 9002),
    (socket.SOCK_DGRAM, DDS_UDP_PORT),
)


def _ensure_world(warehouse: str, sensors: str) -> Path:
    out = GENERATED / warehouse
    info = out / "info.json"
    if info.is_file() and json.loads(info.read_text()).get("sensor_set") == sensors:
        return out
    sys.path.insert(0, str(SIM_DIR))
    from worldgen.__main__ import generate

    return generate(warehouse, sensors, GENERATED)


def _check_ports_free() -> None:
    """Fail fast if a previous simulation still holds our ports: otherwise SITL exits with "bind
    failed" deep in the log and everything else times out waiting for it."""
    busy = []
    for kind, port in SIM_PORTS:
        with socket.socket(socket.AF_INET, kind) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # ignore TIME_WAIT leftovers
            try:
                sock.bind(("0.0.0.0", port))
            except OSError:
                busy.append(f"{'tcp' if kind == socket.SOCK_STREAM else 'udp'}/{port}")
    if busy:
        raise RuntimeError(
            f"ports {', '.join(busy)} are in use: another simulation is still running. Stop it "
            "(Ctrl-C its launch, or `just down` to restart the containers) and try again."
        )


def _arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch(context, *args, **kwargs):
    _check_ports_free()
    warehouse, sensors = _arg(context, "world"), _arg(context, "sensors")
    headless = _arg(context, "headless").lower() == "true"
    world_dir = _ensure_world(warehouse, sensors)
    info = json.loads((world_dir / "info.json").read_text())

    run_dir = Path(_arg(context, "run_dir"))
    run_dir.mkdir(parents=True, exist_ok=True)
    runtime_parm = run_dir / "runtime.parm"
    runtime_parm.write_text(f"DDS_DOMAIN_ID {int(os.environ.get('ROS_DOMAIN_ID', '0'))}\n")
    copter_parm = Path(get_package_share_directory("ardupilot_sitl")) / "config" / "default_params" / "copter.parm"
    defaults = ",".join(str(p) for p in (copter_parm, COMMON_PARM, SIM_PARM, runtime_parm))

    # Generated models first, then every ROS package's share/ directory so package:// URIs resolve
    # (ardupilot_gazebo's iris meshes use them; ros_gz_sim's own launch file does the same).
    resource_paths = [str(world_dir / p) for p in info["resource_paths"]]
    shares = [str(Path(p) / "share") for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]
    existing = [p for p in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":") if p]
    gz_resource_path = ":".join(dict.fromkeys(resource_paths + existing + shares))

    gz_cmd = ["gz", "sim", "-r", "-v", "2"]
    if headless:
        gz_cmd += ["-s", "--headless-rendering"]
    gz_cmd.append(str(world_dir / info["world_file"]))

    # Absolute path: SITL reboots by re-exec'ing argv[0], which fails for a bare PATH name.
    sitl_cmd = [
        shutil.which("arducopter") or "arducopter",
        "--model",
        "JSON",
        "--speedup",
        _arg(context, "speedup"),
        "--slave",
        "0",
        "--sim-address=127.0.0.1",
        "-I0",
        "--home",
        info["sitl_home"],
        "--defaults",
        defaults,
    ]
    if _arg(context, "wipe").lower() == "true":
        sitl_cmd.append("--wipe")

    mavproxy_cmd = [
        "mavproxy.py",
        "--master",
        "tcp:127.0.0.1:5760",
        "--out",
        "udp:127.0.0.1:14550",
        "--non-interactive",
    ]
    gcs_out = _arg(context, "gcs_out")
    if gcs_out:
        mavproxy_cmd += ["--out", gcs_out]

    return [
        LogInfo(msg=f"sim: world {world_dir} (sensors={info['sensor_set']}, headless={headless})"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        ExecuteProcess(cmd=gz_cmd, name="gazebo", output="screen"),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            parameters=[{"config_file": str(world_dir / info["bridge_file"])}],
        ),
        ExecuteProcess(cmd=sitl_cmd, name="ardupilot_sitl", cwd=str(run_dir), output="screen"),
        # On screen so the FC's own messages ("AP: PreArm: ...", arming, mode changes) land in the
        # launch log and in CI artifacts; unbuffered or Python holds them until exit. Its tlogs go
        # to the run directory.
        ExecuteProcess(
            cmd=mavproxy_cmd,
            name="mavproxy",
            cwd=str(run_dir),
            output="screen",
            additional_env={"PYTHONUNBUFFERED": "1"},
        ),
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            arguments=["udp4", "-p", str(DDS_UDP_PORT)],
            output="log",
        ),
        # Make sure AP_DDS connected (reboot SITL once if it lost the start-up race), then set the
        # EKF origin: no GPS indoors, so the EKF needs one before it will navigate.
        Node(
            package="inv_tools",
            executable="fc_bootstrap",
            name="fc_bootstrap",
            output="screen",
            arguments=["--connection", SITL_MAVLINK_SERIAL1, "--warehouse", warehouse],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="sim_small"),
            DeclareLaunchArgument("sensors", default_value="label"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("speedup", default_value="1"),
            DeclareLaunchArgument("wipe", default_value="true"),
            DeclareLaunchArgument("gcs_out", default_value=os.environ.get("INV_GCS_OUT", "")),
            DeclareLaunchArgument("run_dir", default_value="/tmp/coredrone/sitl"),
            OpaqueFunction(function=_launch),
        ]
    )
