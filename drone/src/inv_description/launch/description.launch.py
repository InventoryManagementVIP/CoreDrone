"""robot_state_publisher for the CoreDrone URDF (all joints are fixed, so it only publishes /tf_static)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _rsp(context, *args, **kwargs):
    from inv_description.extrinsics import default_xacro_path, expand_xacro

    xacro_path = LaunchConfiguration("model").perform(context) or default_xacro_path()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": ParameterValue(expand_xacro(xacro_path), value_type=str),
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("model", default_value="", description="xacro file; default: inv_quad"),
            OpaqueFunction(function=_rsp),
        ]
    )
