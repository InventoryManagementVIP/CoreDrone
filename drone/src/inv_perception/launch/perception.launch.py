"""Perception: perception:=cpu (zxing-cpp + apriltag_ros) | gpu (TODO phase-2) | none.

Every detector publishes inv_interfaces/LabelDetectionArray on /perception/detections, stamped
with the source image's stamp.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    mode = LaunchConfiguration("perception").perform(context)
    if mode == "none":
        return []
    if mode == "gpu":
        raise RuntimeError("perception:=gpu (isaac_ros_apriltag) is TODO(phase-2); use perception:=cpu")
    if mode != "cpu":
        raise ValueError(f"perception:={mode!r}; choose cpu, gpu or none")
    share = get_package_share_directory("inv_perception")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    image = LaunchConfiguration("image_topic").perform(context)
    info = LaunchConfiguration("camera_info_topic").perform(context)
    tag_size = float(LaunchConfiguration("tag_size_m").perform(context))
    qr_size = float(LaunchConfiguration("qr_symbol_size_m").perform(context))
    common = {"use_sim_time": use_sim_time}
    return [
        Node(
            package="inv_perception",
            executable="code_reader",
            name="code_reader",
            output="screen",
            parameters=[
                os.path.join(share, "config", "code_reader.yaml"),
                common,
                {"image_topic": image, "camera_info_topic": info, "qr_symbol_size_m": qr_size},
            ],
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[os.path.join(share, "config", "apriltag_ros.yaml"), common, {"size": tag_size}],
            remappings=[
                ("image_rect", image),
                ("camera_info", info),
                ("detections", "/perception/apriltag/detections"),
            ],
        ),
        Node(
            package="inv_perception",
            executable="tag_adapter",
            name="tag_adapter",
            output="screen",
            parameters=[os.path.join(share, "config", "tag_adapter.yaml"), common, {"tag_size_m": tag_size}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("perception", default_value="cpu", description="cpu | gpu | none"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("tag_size_m", default_value="0.09"),
            DeclareLaunchArgument("qr_symbol_size_m", default_value="0.0"),
            OpaqueFunction(function=_launch),
        ]
    )
