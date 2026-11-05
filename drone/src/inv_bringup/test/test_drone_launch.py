"""launch_testing: drone.launch.py composes the whole stack (without a platform) and every
package comes up and publishes its status topic."""

import os
import tempfile
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from inv_interfaces.msg import CorridorStatus, FlightState, LocalizationStatus, SafetyStatus

RUN_DIR = tempfile.mkdtemp(prefix="drone_launch_test_")


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("inv_bringup")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "drone.launch.py")),
                launch_arguments={
                    "platform": "sim",
                    "with_platform": "false",
                    "perception": "none",
                    "mission": "none",
                    "run_dir": RUN_DIR,
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestStackComposes(unittest.TestCase):
    def test_every_package_reports(self):
        rclpy.init()
        try:
            node = rclpy.create_node("test_stack_listener")
            got = {}
            for topic, msg_type in (
                ("/flight/state", FlightState),
                ("/localization/status", LocalizationStatus),
                ("/mapping/blocked", CorridorStatus),
                ("/safety/status", SafetyStatus),
            ):
                node.create_subscription(msg_type, topic, lambda m, t=topic: got.setdefault(t, m), 10)
            # use_sim_time is on and no simulator runs: publish a clock so timers fire.
            from rosgraph_msgs.msg import Clock

            clock_pub = node.create_publisher(Clock, "/clock", 10)
            t = 1.0
            deadline = time.monotonic() + 40.0
            while time.monotonic() < deadline and len(got) < 4:
                t += 0.05
                clock_pub.publish(Clock(clock=rclpy.time.Time(seconds=t).to_msg()))
                rclpy.spin_once(node, timeout_sec=0.05)
            self.assertEqual(
                sorted(got), sorted(["/flight/state", "/localization/status", "/mapping/blocked", "/safety/status"])
            )
            self.assertFalse(got["/flight/state"].connected)  # no SITL in this test
            node.destroy_node()
        finally:
            rclpy.shutdown()
