"""launch_testing: the inventory node attributes an item to the slot whose tag is in the same frame."""

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

from inv_interfaces.msg import InventoryItemArray, LabelDetection, LabelDetectionArray

DB = os.path.join(tempfile.mkdtemp(prefix="inv_test_"), "inventory.sqlite")


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("inv_inventory")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "inventory.launch.py")),
                launch_arguments={"warehouse": "sim_small", "db_path": DB}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def detection(code, symbology, x, y):
    d = LabelDetection()
    d.code, d.symbology, d.confidence = code, symbology, 1.0
    d.center_x_px, d.center_y_px = float(x), float(y)
    return d


class TestInventoryAttributes(unittest.TestCase):
    def test_item_gets_slot_from_tag(self):
        rclpy.init()
        try:
            node = rclpy.create_node("fake_perception")
            pub = node.create_publisher(LabelDetectionArray, "/perception/detections", 10)
            items = []
            node.create_subscription(InventoryItemArray, "/inventory/items", lambda m: items.extend(m.items), 10)
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline and not items:
                stamp = node.get_clock().now().to_msg()
                # Two detectors report on the same image, in separate messages.
                tags = LabelDetectionArray(
                    source="apriltag_ros", detections=[detection("tag36h11:4", "apriltag", 480, 380)]
                )
                codes = LabelDetectionArray(source="zxing", detections=[detection("SKU-77777", "qr", 470, 200)])
                for msg in (tags, codes):
                    msg.header.stamp, msg.header.frame_id = stamp, "camera_color_optical_frame"
                    pub.publish(msg)
                end = time.monotonic() + 1.0
                while time.monotonic() < end and not items:
                    rclpy.spin_once(node, timeout_sec=0.1)
            self.assertTrue(items, "no /inventory/items")
            self.assertEqual(items[0].code, "SKU-77777")
            # Tag 4 is the fifth slot in layout order: A1-R, level 1, bay 1.
            self.assertEqual(items[0].slot_code, "A1-R-L1-B1")
            self.assertEqual(items[0].attribution_method, "label")
            self.assertTrue(os.path.exists(DB))
            node.destroy_node()
        finally:
            rclpy.shutdown()
