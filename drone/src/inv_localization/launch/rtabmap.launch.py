"""`rtabmap` backend: CPU stereo + IMU fallback, for a real-sensor path without a GPU.

TODO(phase-2): not implemented. When it is needed:
  * apt: ros-${ROS_DISTRO}-rtabmap-ros (add it to this package's package.xml)
  * run rtabmap_odom stereo_odometry + rtabmap on /camera/left|right/image_raw and /camera/imu,
    publishing map->odom->base_link and remapping odometry to /localization/odom
  * reuse the adapter unchanged
See docs/adr/0004-isaac-vslam-swappable-localization.md for why this beats ORB-SLAM3/Stella.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def _fail(context, *args, **kwargs):
    raise RuntimeError(
        "localization:=rtabmap is a phase-2 stub (TODO(phase-2) in inv_localization/launch/rtabmap.launch.py). "
        "Use localization:=groundtruth in simulation or localization:=isaac on NVIDIA hardware."
    )


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("use_sim_time", default_value="false"), OpaqueFunction(function=_fail)]
    )
