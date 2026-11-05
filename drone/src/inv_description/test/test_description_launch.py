"""launch_testing: description.launch.py starts and publishes the sensor frames on /tf_static."""

import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("inv_description")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "description.launch.py"))
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDescriptionPublishes(unittest.TestCase):
    def test_tf_static_has_camera_frames(self):
        rclpy.init()
        try:
            node = rclpy.create_node("test_description_listener")
            frames = set()
            qos = QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
            )
            node.create_subscription(
                TFMessage, "/tf_static", lambda m: frames.update(t.child_frame_id for t in m.transforms), qos
            )
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline and "camera_color_optical_frame" not in frames:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertIn("camera_color_optical_frame", frames)
            self.assertIn("camera_left_optical_frame", frames)
            node.destroy_node()
        finally:
            rclpy.shutdown()
