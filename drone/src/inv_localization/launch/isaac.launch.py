"""`isaac` backend: Isaac ROS Visual SLAM on the canonical camera topics.

UNTESTED in this repo: needs CUDA, so it only runs in the Isaac ROS images on an RTX GPU or the
Jetson, never in GitHub-hosted CI (docs/onboarding/linux-nvidia.md has the manual GPU check).

TODO(phase-2): relay isaac_ros_visual_slam_interfaces/VisualSlamStatus (visual_slam/status) into
/localization/backend_status so the adapter sees "tracking lost" immediately instead of through
the odometry timeout. The message's field names are not documented for 4.6; read them off the
installed interface first.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _launch(context, *args, **kwargs):
    try:
        get_package_share_directory("isaac_ros_visual_slam")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "localization:=isaac needs Isaac ROS Visual SLAM, which needs CUDA. Run inside "
            "docker/isaac-x86.Dockerfile (RTX GPU) or the Jetson image, or use "
            "localization:=groundtruth in simulation."
        ) from exc
    config = os.path.join(get_package_share_directory("inv_localization"), "config", "isaac_vslam.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    vslam = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam_node",
        parameters=[config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("visual_slam/image_0", "/camera/left/image_raw"),
            ("visual_slam/camera_info_0", "/camera/left/camera_info"),
            ("visual_slam/image_1", "/camera/right/image_raw"),
            ("visual_slam/camera_info_1", "/camera/right/camera_info"),
            ("visual_slam/imu", "/camera/imu"),
            ("visual_slam/tracking/odometry", "/localization/odom"),
        ],
    )
    return [
        ComposableNodeContainer(
            name="visual_slam_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[vslam],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("use_sim_time", default_value="false"), OpaqueFunction(function=_launch)]
    )
