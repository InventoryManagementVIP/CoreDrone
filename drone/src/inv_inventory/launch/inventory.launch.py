"""Start the inventory node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("inv_inventory"), "config", "inventory.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("warehouse", default_value="sim_small"),
            DeclareLaunchArgument("db_path", default_value="/tmp/coredrone/inventory.sqlite"),
            Node(
                package="inv_inventory",
                executable="inventory",
                name="inventory",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                        "warehouse": LaunchConfiguration("warehouse"),
                        "db_path": LaunchConfiguration("db_path"),
                    },
                ],
            ),
        ]
    )
