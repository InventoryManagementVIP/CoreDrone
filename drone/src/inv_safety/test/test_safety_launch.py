"""launch_testing: the safety monitor starts and, with nothing else running, reports a problem."""

import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from inv_interfaces.msg import SafetyStatus


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("inv_safety")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "safety.launch.py"))
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSafetyStarts(unittest.TestCase):
    def test_publishes_status(self):
        rclpy.init()
        try:
            node = rclpy.create_node("test_safety_listener")
            msgs = []
            node.create_subscription(SafetyStatus, "/safety/status", msgs.append, 10)
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline and not msgs:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertTrue(msgs, "no /safety/status")
            # No flight interface running: flight state is missing, so the level cannot be OK.
            self.assertNotEqual(msgs[-1].level, SafetyStatus.LEVEL_OK)
            node.destroy_node()
        finally:
            rclpy.shutdown()
