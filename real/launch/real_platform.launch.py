"""Real-vehicle platform: camera driver (remapped to canonical topics), micro-ROS agent on the
flight controller's serial port, and the EKF origin setter. SKELETON: the parts are chosen (real/BOM.md)
but nothing here has run on a vehicle. Mirrors sim/launch/sim_platform.launch.py; the autonomy stack is identical.

  camera:=oak_d_lite|none        OAK-D Lite (real/BOM.md), or no camera driver
  dds_device:=/dev/ttyTHS1       UART wired to the FC's DDS serial port (real/flight_controller/README.md)
  dds_baud:=921600               must match SERIALn_BAUD on the FC
  mavlink:=udpin:127.0.0.1:14551 mavlink-router endpoint for the EKF origin setter
  world:=<warehouse>             layout whose geodetic_origin becomes the EKF origin
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Canonical topics (docs/interfaces.md). Driver-specific names are remapped onto these here and
# only here, so a camera swap never touches the autonomy code.
OAK_REMAPS = [
    ("/oak/left/image_rect", "/camera/left/image_raw"),
    ("/oak/left/camera_info", "/camera/left/camera_info"),
    ("/oak/right/image_rect", "/camera/right/image_raw"),
    ("/oak/right/camera_info", "/camera/right/camera_info"),
    ("/oak/stereo/image_raw", "/camera/depth/image_raw"),
    ("/oak/stereo/camera_info", "/camera/depth/camera_info"),
    ("/oak/rgb/image_raw", "/camera/color/image_raw"),
    ("/oak/rgb/camera_info", "/camera/color/camera_info"),
    ("/oak/imu/data", "/camera/imu"),
]
# The driver stamps data with its own frame names. Tie them to the URDF's frames, which stay the
# only source of extrinsics. TODO(hardware): check these names with `ros2 topic echo`.
OAK_FRAMES = [
    ("camera_left_optical_frame", "oak_left_camera_optical_frame"),
    ("camera_right_optical_frame", "oak_right_camera_optical_frame"),
    ("camera_color_optical_frame", "oak_rgb_camera_optical_frame"),
    ("camera_imu_optical_frame", "oak_imu_frame"),
]


def _launch(context, *args, **kwargs):
    camera = LaunchConfiguration("camera").perform(context)
    actions = [
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            arguments=[
                "serial",
                "--dev",
                LaunchConfiguration("dds_device").perform(context),
                "-b",
                LaunchConfiguration("dds_baud").perform(context),
            ],
            output="log",
        ),
        # DDS link check + EKF origin. The FC usually boots before the Jetson, so we have not seen
        # its DDS start-up messages: reboot it once (only ever while disarmed) to observe them.
        Node(
            package="inv_tools",
            executable="fc_bootstrap",
            name="fc_bootstrap",
            output="screen",
            arguments=[
                "--connection",
                LaunchConfiguration("mavlink").perform(context),
                "--warehouse",
                LaunchConfiguration("world").perform(context),
                "--reboot-if-unknown",
            ],
        ),
    ]
    if camera == "oak_d_lite":
        # TODO(hardware): untested on a camera. Parameters live in real/sensors/oak_d_lite.yaml.
        actions.append(
            Node(
                package="depthai_ros_driver",
                executable="camera_node",
                name="oak",
                parameters=["/opt/inv/real/sensors/oak_d_lite.yaml"],
                remappings=OAK_REMAPS,
                output="screen",
            )
        )
        actions += [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{driver_frame}_alias",
                arguments=["--frame-id", urdf_frame, "--child-frame-id", driver_frame],
            )
            for urdf_frame, driver_frame in OAK_FRAMES
        ]
    elif camera != "none":
        raise ValueError(f"camera:={camera!r} not supported; use oak_d_lite or none")
    else:
        actions.insert(0, LogInfo(msg="real platform: camera:=none (no camera driver started)"))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera", default_value="oak_d_lite"),
            DeclareLaunchArgument("dds_device", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument("dds_baud", default_value="921600"),
            DeclareLaunchArgument("mavlink", default_value="udpin:127.0.0.1:14551"),
            DeclareLaunchArgument("world", default_value="sim_small"),
            OpaqueFunction(function=_launch),
        ]
    )
