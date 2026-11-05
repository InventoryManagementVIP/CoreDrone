"""Start the mission node (mission:=scan|smoke)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("inv_mission"), "config", "mission.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("mission", default_value="scan"),
            DeclareLaunchArgument("warehouse", default_value="sim_small"),
            DeclareLaunchArgument("mission_id", default_value=""),
            Node(
                package="inv_mission",
                executable="mission",
                name="mission",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                        "mission": LaunchConfiguration("mission"),
                        "warehouse": LaunchConfiguration("warehouse"),
                        "mission_id": LaunchConfiguration("mission_id"),
                    },
                ],
            ),
        ]
    )
