"""Localization: one backend (localization:=groundtruth|isaac|rtabmap) plus the adapter.

Contract for every backend (docs/interfaces.md): TF map->odom->base_link (REP-105) and
nav_msgs/Odometry on /localization/odom; optionally LocalizationStatus on
/localization/backend_status.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BACKENDS = ("groundtruth", "isaac", "rtabmap")


def _launch(context, *args, **kwargs):
    share = get_package_share_directory("inv_localization")
    backend = LaunchConfiguration("localization").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    extra = LaunchConfiguration("params_file").perform(context)
    if backend not in BACKENDS:
        raise ValueError(f"localization:={backend!r}; choose one of {', '.join(BACKENDS)}")

    def params(name):
        p = [os.path.join(share, "config", name)]
        if extra:
            p.append(extra)
        p.append({"use_sim_time": use_sim_time})
        return p

    actions = []
    if backend == "groundtruth":
        if not use_sim_time:
            raise ValueError("localization:=groundtruth only exists in simulation (platform:=sim)")
        actions.append(
            Node(
                package="inv_localization",
                executable="groundtruth",
                name="localization_groundtruth",
                output="screen",
                parameters=params("groundtruth.yaml"),
            )
        )
    else:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", f"{backend}.launch.py")),
                launch_arguments={"use_sim_time": str(use_sim_time).lower()}.items(),
            )
        )
    actions.append(
        Node(
            package="inv_localization",
            executable="adapter",
            name="localization_adapter",
            output="screen",
            parameters=[*params("adapter.yaml"), {"backend": backend}],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("localization", default_value="groundtruth", description="|".join(BACKENDS)),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value="", description="extra parameter overrides"),
            OpaqueFunction(function=_launch),
        ]
    )
