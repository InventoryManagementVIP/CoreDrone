"""launch_testing: groundtruth backend + adapter start and publish external nav from a fake sim pose."""

import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

from inv_interfaces.msg import LocalizationStatus


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("inv_localization")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "localization.launch.py")),
                launch_arguments={"localization": "groundtruth", "use_sim_time": "true"}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGroundTruthPipeline(unittest.TestCase):
    def test_truth_reaches_external_nav(self):
        rclpy.init()
        try:
            node = rclpy.create_node("fake_sim")
            clock_pub = node.create_publisher(Clock, "/clock", 10)
            truth_pub = node.create_publisher(Odometry, "/sim/ground_truth/odom", 10)
            nav, status = [], []
            # External nav is streamed best-effort; a reliable subscription would never match.
            node.create_subscription(Odometry, "/localization/external_nav", nav.append, qos_profile_sensor_data)
            node.create_subscription(LocalizationStatus, "/localization/status", status.append, 10)
            sim_t = 100.0
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                sim_t += 0.02
                stamp = rclpy.time.Time(seconds=sim_t).to_msg()
                clock_pub.publish(Clock(clock=stamp))
                truth = Odometry()
                truth.header.stamp = stamp
                truth.header.frame_id, truth.child_frame_id = "map", "base_link"
                truth.pose.pose.position.x = 1.0
                truth.pose.pose.position.z = 0.2
                truth.pose.pose.orientation.w = 1.0
                truth_pub.publish(truth)
                rclpy.spin_once(node, timeout_sec=0.02)
                tracking = [s for s in status if s.state == LocalizationStatus.STATE_TRACKING]
                if nav and tracking:
                    break
            self.assertTrue(nav, "nothing on /localization/external_nav")
            self.assertEqual(nav[-1].header.frame_id, "odom")
            self.assertEqual(nav[-1].child_frame_id, "base_link")
            self.assertAlmostEqual(nav[-1].pose.pose.position.x, 1.0, places=6)
            self.assertTrue(any(s.state == LocalizationStatus.STATE_TRACKING for s in status), "never TRACKING")
            node.destroy_node()
        finally:
            rclpy.shutdown()
