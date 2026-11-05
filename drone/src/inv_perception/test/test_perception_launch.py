"""launch_testing: the CPU perception pipeline decodes a synthetic image with a QR code and an AprilTag."""

import os
import time
import unittest

import launch
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sensor_msgs.msg import CameraInfo, Image

from inv_interfaces.msg import LabelDetectionArray

WIDTH, HEIGHT, F = 960, 540, 700.0


def synthetic_image() -> np.ndarray:
    import cv2
    import qrcode

    img = np.full((HEIGHT, WIDTH), 230, dtype=np.uint8)
    qr = qrcode.QRCode(box_size=7, border=4)
    qr.add_data("SKU-04242")
    qr.make(fit=True)
    q = np.array(qr.make_image().convert("L"))
    img[40 : 40 + q.shape[0], 80 : 80 + q.shape[1]] = q
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    make = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    tag = make(dictionary, 7, 160, borderBits=1)
    tag = np.pad(tag, 20, constant_values=255)
    img[300 : 300 + tag.shape[0], 600 : 600 + tag.shape[1]] = tag
    return img


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory("inv_perception")
    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "perception.launch.py")),
                launch_arguments={"perception": "cpu", "tag_size_m": "0.09", "qr_symbol_size_m": "0.18"}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestPerceptionDecodes(unittest.TestCase):
    def test_qr_and_apriltag(self):
        rclpy.init()
        try:
            node = rclpy.create_node("fake_camera")
            image_pub = node.create_publisher(Image, "/camera/color/image_raw", 10)
            info_pub = node.create_publisher(CameraInfo, "/camera/color/camera_info", 10)
            seen = {}
            node.create_subscription(
                LabelDetectionArray,
                "/perception/detections",
                lambda m: seen.update({d.code: (m.source, d) for d in m.detections}),
                10,
            )
            frame = synthetic_image()
            deadline = time.monotonic() + 40.0
            while time.monotonic() < deadline and not {"SKU-04242", "tag36h11:7"} <= set(seen):
                stamp = node.get_clock().now().to_msg()
                img = Image(height=HEIGHT, width=WIDTH, encoding="mono8", step=WIDTH, data=frame.tobytes())
                img.header.stamp, img.header.frame_id = stamp, "camera_color_optical_frame"
                info = CameraInfo(height=HEIGHT, width=WIDTH, distortion_model="plumb_bob")
                info.header = img.header
                info.k = [F, 0.0, WIDTH / 2, 0.0, F, HEIGHT / 2, 0.0, 0.0, 1.0]
                info.p = [F, 0.0, WIDTH / 2, 0.0, 0.0, F, HEIGHT / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
                info.d = [0.0] * 5
                info_pub.publish(info)
                image_pub.publish(img)
                end = time.monotonic() + 0.3
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
            self.assertIn("SKU-04242", seen, f"QR not decoded; saw {sorted(seen)}")
            self.assertEqual(seen["SKU-04242"][0], "zxing")
            self.assertEqual(seen["SKU-04242"][1].symbology, "qr")
            self.assertTrue(seen["SKU-04242"][1].has_pose)
            self.assertIn("tag36h11:7", seen, f"AprilTag not decoded; saw {sorted(seen)}")
            self.assertEqual(seen["tag36h11:7"][0], "apriltag_ros")
            node.destroy_node()
        finally:
            rclpy.shutdown()
