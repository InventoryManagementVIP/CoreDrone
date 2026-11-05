"""Start the uplink node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("inv_uplink"), "config", "uplink.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("db_path", default_value="/tmp/coredrone/inventory.sqlite"),
            DeclareLaunchArgument("export_dir", default_value="/tmp/coredrone/exports"),
            Node(
                package="inv_uplink",
                executable="uplink",
                name="uplink",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                        "db_path": LaunchConfiguration("db_path"),
                        "export_dir": LaunchConfiguration("export_dir"),
                    },
                ],
            ),
        ]
    )
