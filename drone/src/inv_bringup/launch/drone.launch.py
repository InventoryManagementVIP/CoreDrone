"""Top-level launch: the whole autonomy stack, identical in simulation and on the vehicle.

  ros2 launch inv_bringup drone.launch.py platform:=sim localization:=groundtruth mapping:=none

  platform:=sim|real          picks the platform launch (sim/launch or real/launch) and its params
  localization:=groundtruth|isaac|rtabmap   default: groundtruth in sim, isaac on the vehicle
  mapping:=none|nvblox
  perception:=cpu|gpu|none
  mission:=none|smoke|scan    none = stack only; smoke = takeoff/hover/land; scan = full route
  warehouse:=sim_small        config/warehouses/<name>.yaml
  with_platform:=true         false = autonomy stack only (platform started separately, e.g. systemd)
  run_dir:=                   where the inventory DB and CSV exports go (default: $INV_ROOT/runs/latest)
  sim only:  sensors:=label|none|full   headless:=true|false   speedup:=1
  foxglove:=false             start foxglove_bridge on port 8765 (loopback only on the vehicle)
  params_file:=               extra overrides (YAML, keyed by node name) for the nodes started in this
                              file; localization, mapping and perception take their own

Parameters layer as: package defaults, platform overrides, values computed here, params_file.
Later files win, so a package's defaults must come first.

Only the platform differs between sim and real; if you find yourself adding a node here that
only exists for one platform, it belongs in sim/launch or real/launch instead.
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PLATFORMS = ("sim", "real")


def _share(pkg: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(pkg), *parts)


def repo_root() -> Path:
    env = os.environ.get("INV_ROOT")
    if env:
        return Path(env)
    for parent in Path(__file__).resolve().parents:
        if (parent / "sim" / "launch").is_dir() and (parent / "real" / "launch").is_dir():
            return parent
    raise RuntimeError("cannot find the repository (sim/ and real/); set INV_ROOT")


def _launch(context, *args, **kwargs):
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    platform = arg("platform")
    if platform not in PLATFORMS:
        raise ValueError(f"platform:={platform!r}; choose sim or real")
    sim = platform == "sim"
    use_sim_time = sim
    localization = arg("localization") or ("groundtruth" if sim else "isaac")
    warehouse, mission = arg("warehouse"), arg("mission")
    root = repo_root()
    run_dir = Path(arg("run_dir") or root / "runs" / "latest")
    run_dir.mkdir(parents=True, exist_ok=True)

    from inv_layout import load_layout

    layout = load_layout(warehouse)
    (lo, hi) = layout.bounds(margin_m=1.0)

    platform_params = _share("inv_bringup", "config", f"{platform}.yaml")
    user_params = arg("params_file")

    def params(package_defaults, computed=None):
        out = [package_defaults, platform_params]
        if computed:
            out.append(computed)
        if user_params:
            out.append(user_params)
        return [*out, {"use_sim_time": use_sim_time}]

    actions = [
        LogInfo(
            msg=f"drone: platform={platform} localization={localization} mapping={arg('mapping')} "
            f"perception={arg('perception')} mission={mission} warehouse={warehouse} run_dir={run_dir}"
        )
    ]

    if arg("with_platform").lower() == "true":
        launch_file = root / platform / "launch" / f"{platform}_platform.launch.py"
        platform_args = {"world": warehouse}
        if sim:
            platform_args.update(sensors=arg("sensors"), headless=arg("headless"), speedup=arg("speedup"))
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_file)), launch_arguments=platform_args.items()
            )
        )

    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_share("inv_description", "launch", "description.launch.py")),
            launch_arguments={"use_sim_time": str(use_sim_time).lower()}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_share("inv_localization", "launch", "localization.launch.py")),
            launch_arguments={
                "localization": localization,
                "use_sim_time": str(use_sim_time).lower(),
                "params_file": platform_params,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_share("inv_mapping", "launch", "mapping.launch.py")),
            launch_arguments={"mapping": arg("mapping"), "use_sim_time": str(use_sim_time).lower()}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_share("inv_perception", "launch", "perception.launch.py")),
            launch_arguments={
                "perception": arg("perception"),
                "use_sim_time": str(use_sim_time).lower(),
                "tag_size_m": str(layout.location_label.size_m),
                "qr_symbol_size_m": str(layout.item_label.size_m if layout.item_label.kind == "qr" else 0.0),
            }.items(),
        ),
        Node(
            package="inv_flight_interface",
            executable="flight_interface",
            name="flight_interface",
            output="screen",
            parameters=params(
                _share("inv_flight_interface", "config", "flight_interface.yaml"),
                {
                    "geofence_min_xyz": [float(v) for v in lo],
                    "geofence_max_xyz": [float(v) for v in hi],
                    # Hold and then land if the mission node dies mid-flight.
                    "mission_heartbeat_timeout_s": 5.0 if mission != "none" else 0.0,
                    # The safety monitor is always part of this stack: land if it dies.
                    "safety_heartbeat_timeout_s": 3.0,
                },
            ),
        ),
        Node(
            package="inv_inventory",
            executable="inventory",
            name="inventory",
            output="screen",
            parameters=params(
                _share("inv_inventory", "config", "inventory.yaml"),
                {"warehouse": warehouse, "db_path": str(run_dir / "inventory.sqlite")},
            ),
        ),
        Node(
            package="inv_uplink",
            executable="uplink",
            name="uplink",
            output="screen",
            parameters=params(
                _share("inv_uplink", "config", "uplink.yaml"),
                {
                    "db_path": str(run_dir / "inventory.sqlite"),
                    "export_dir": str(run_dir / "exports"),
                    "mock_output": str(run_dir / "mock_backend.jsonl"),
                },
            ),
        ),
        Node(
            package="inv_safety",
            executable="safety_monitor",
            name="safety_monitor",
            output="screen",
            parameters=params(_share("inv_safety", "config", "safety.yaml")),
        ),
    ]
    if mission != "none":
        actions.append(
            Node(
                package="inv_mission",
                executable="mission",
                name="mission",
                output="screen",
                parameters=params(
                    _share("inv_mission", "config", "mission.yaml"),
                    {"mission": mission, "warehouse": warehouse, "mission_id": arg("mission_id")},
                ),
            )
        )
    if arg("foxglove").lower() == "true":
        actions.append(
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="log",
                # Sim: inside the container, which publishes the port to the host's loopback only
                # (docker/compose.yaml). The vehicle uses host networking: reach it over an SSH tunnel.
                parameters=[{"port": 8765, "address": "0.0.0.0" if sim else "127.0.0.1", "use_sim_time": use_sim_time}],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("platform", default_value="sim", description="sim | real"),
            DeclareLaunchArgument("localization", default_value="", description="groundtruth | isaac | rtabmap"),
            DeclareLaunchArgument("mapping", default_value="none", description="none | nvblox"),
            DeclareLaunchArgument("perception", default_value="cpu", description="cpu | gpu | none"),
            DeclareLaunchArgument("mission", default_value="none", description="none | smoke | scan"),
            DeclareLaunchArgument("mission_id", default_value=""),
            DeclareLaunchArgument("warehouse", default_value="sim_small"),
            DeclareLaunchArgument("with_platform", default_value="true"),
            DeclareLaunchArgument("run_dir", default_value=""),
            DeclareLaunchArgument("sensors", default_value="label", description="sim: none | label | full"),
            DeclareLaunchArgument("headless", default_value="true", description="sim: Gazebo without GUI"),
            DeclareLaunchArgument("speedup", default_value="1", description="sim: SITL speedup"),
            DeclareLaunchArgument("foxglove", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=""),
            OpaqueFunction(function=_launch),
        ]
    )
