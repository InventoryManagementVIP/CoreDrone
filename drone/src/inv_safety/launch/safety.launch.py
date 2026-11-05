"""Start the safety monitor with its default thresholds."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("inv_safety"), "config", "safety.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="inv_safety",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
                parameters=[
                    config,
                    {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
                ],
            ),
        ]
    )
