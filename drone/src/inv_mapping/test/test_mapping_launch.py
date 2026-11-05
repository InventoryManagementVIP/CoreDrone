"""launch_testing: mapping:=none starts and publishes an UNKNOWN (not blocked) corridor status."""

import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from inv_interfaces.msg import CorridorStatus


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("inv_mapping")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "mapping.launch.py")),
                launch_arguments={"mapping": "none"}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestNoneBackendPublishes(unittest.TestCase):
    def test_unknown_not_blocked(self):
        rclpy.init()
        try:
            node = rclpy.create_node("test_corridor_listener")
            msgs = []
            node.create_subscription(CorridorStatus, "/mapping/blocked", msgs.append, 10)
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline and not msgs:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertTrue(msgs)
            self.assertEqual(msgs[-1].state, CorridorStatus.STATE_UNKNOWN)
            self.assertFalse(msgs[-1].blocked)
            node.destroy_node()
        finally:
            rclpy.shutdown()
