"""Obstacle mapping: mapping:=none (default; no GPU) or mapping:=nvblox (Isaac ROS images only).

Both start the corridor checker, so /mapping/blocked always exists.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _launch(context, *args, **kwargs):
    backend = LaunchConfiguration("mapping").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    share = get_package_share_directory("inv_mapping")
    actions = [
        Node(
            package="inv_mapping",
            executable="corridor_checker",
            name="corridor_checker",
            output="screen",
            parameters=[
                os.path.join(share, "config", "corridor_checker.yaml"),
                {"backend": backend, "use_sim_time": use_sim_time},
            ],
        )
    ]
    if backend == "none":
        return actions
    if backend != "nvblox":
        raise ValueError(f"mapping:={backend!r}; choose none or nvblox")
    try:
        get_package_share_directory("nvblox_ros")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "mapping:=nvblox needs Isaac ROS nvblox (CUDA). Run inside docker/isaac-x86.Dockerfile or the "
            "Jetson image, or use mapping:=none."
        ) from exc
    # UNTESTED in this repo (needs CUDA). Inputs are the canonical depth topics (docs/interfaces.md).
    nvblox = ComposableNode(
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        name="nvblox_node",
        parameters=[os.path.join(share, "config", "nvblox.yaml"), {"use_sim_time": use_sim_time}],
        remappings=[
            ("camera_0/depth/image", "/camera/depth/image_raw"),
            ("camera_0/depth/camera_info", "/camera/depth/camera_info"),
            ("camera_0/color/image", "/camera/color/image_raw"),
            ("camera_0/color/camera_info", "/camera/color/camera_info"),
        ],
    )
    actions.append(
        ComposableNodeContainer(
            name="nvblox_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[nvblox],
            output="screen",
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mapping", default_value="none", description="none | nvblox"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch),
        ]
    )
