"""launch_testing: the node starts without a flight controller and reports a lost link."""

import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from inv_interfaces.msg import FlightState


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("inv_flight_interface")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "flight_interface.launch.py"))
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestFlightInterfaceStarts(unittest.TestCase):
    def test_publishes_state_without_fc(self):
        rclpy.init()
        try:
            node = rclpy.create_node("test_flight_state_listener")
            states = []
            node.create_subscription(FlightState, "/flight/state", states.append, 10)
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline and not states:
                rclpy.spin_once(node, timeout_sec=0.2)
            self.assertTrue(states, "no /flight/state message")
            self.assertFalse(states[-1].connected)
            self.assertFalse(states[-1].armed)
            node.destroy_node()
        finally:
            rclpy.shutdown()
