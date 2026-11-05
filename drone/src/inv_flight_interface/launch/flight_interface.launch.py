"""Start inv_flight_interface with its default parameters plus optional overrides."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context, *args, **kwargs):
    defaults = os.path.join(get_package_share_directory("inv_flight_interface"), "config", "flight_interface.yaml")
    params = [defaults]
    extra = LaunchConfiguration("params_file").perform(context)
    if extra:
        params.append(extra)
    params.append({"use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true"})
    return [
        Node(
            package="inv_flight_interface",
            executable="flight_interface",
            name="flight_interface",
            output="screen",
            parameters=params,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value="", description="extra parameter overrides"),
            OpaqueFunction(function=_node),
        ]
    )
