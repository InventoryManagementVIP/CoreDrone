"""Barcode / QR reader (CPU): zxing-cpp on the label camera, published on /perception/detections.

AprilTags are not handled here: apriltag_ros detects them and tag_adapter republishes them on the
same topic, with the same image stamp, so inventory can group detections per frame.
Perception does not interpret payloads; inventory decides what is an item and what is a location.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from inv_interfaces.msg import LabelDetection, LabelDetectionArray
from inv_perception.geometry import solve_label_pose
from inv_perception.pose_msgs import pose_from_rt

DEFAULTS: dict[str, object] = {
    "image_topic": "/camera/color/image_raw",
    "camera_info_topic": "/camera/color/camera_info",
    "formats": ["QRCode", "DataMatrix", "Code128", "EAN13"],
    "qr_symbol_size_m": 0.0,  # 0 = unknown, no pose; drone.launch.py sets it from the layout
    "max_rate_hz": 5.0,
    "try_harder": True,
}

# zxing-cpp format name -> contract symbology (contracts/scan_record.schema.json)
SYMBOLOGY = {
    "QRCode": LabelDetection.SYMBOLOGY_QR,
    "MicroQRCode": LabelDetection.SYMBOLOGY_QR,
    "EAN13": LabelDetection.SYMBOLOGY_EAN13,
    "Code128": LabelDetection.SYMBOLOGY_CODE128,
}


def symbology_for(format_name: str) -> str:
    return SYMBOLOGY.get(format_name, LabelDetection.SYMBOLOGY_OTHER)


def format_name(fmt) -> str:
    """zxing-cpp BarcodeFormat -> 'QRCode' (works for the 2.x enum and its string form)."""
    name = getattr(fmt, "name", None) or str(fmt)
    return name.split(".")[-1]


def decode(image: np.ndarray, formats: list[str], try_harder: bool = True) -> list[dict]:
    """Decode every barcode in an image. Returns dicts with text, format, corners (4x2 px)."""
    import zxingcpp

    flags = None
    for name in formats:
        f = getattr(zxingcpp.BarcodeFormat, name)
        flags = f if flags is None else flags | f
    kwargs = {"try_rotate": try_harder, "try_downscale": try_harder}
    results = zxingcpp.read_barcodes(image, formats=flags, **kwargs) if flags is not None else []
    out = []
    for r in results:
        if not r.valid:
            continue
        p = r.position
        corners = np.array(
            [
                [p.top_left.x, p.top_left.y],
                [p.top_right.x, p.top_right.y],
                [p.bottom_right.x, p.bottom_right.y],
                [p.bottom_left.x, p.bottom_left.y],
            ],
            dtype=np.float64,
        )
        out.append({"text": r.text, "format": format_name(r.format), "corners": corners})
    return out


class CodeReader(Node):
    def __init__(self) -> None:
        super().__init__("code_reader")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.bridge = CvBridge()
        self._camera_matrix: np.ndarray | None = None
        self._dist: np.ndarray | None = None
        self._last_wall = 0.0
        self.pub = self.create_publisher(LabelDetectionArray, "/perception/detections", 10)
        self.create_subscription(CameraInfo, self.p["camera_info_topic"], self._on_info, qos_profile_sensor_data)
        # Reliable, like apriltag_ros: best effort lost whole hovers' worth of these large messages under
        # load (0 of 10 frames at some scan points), and with them the item reads.
        self.create_subscription(Image, self.p["image_topic"], self._on_image, 5)
        self.get_logger().info(f"reading {', '.join(self.p['formats'])} from {self.p['image_topic']}")

    def _on_info(self, msg: CameraInfo) -> None:
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist = np.array(msg.d, dtype=np.float64) if len(msg.d) else None

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self.p["max_rate_hz"] > 0 and now - self._last_wall < 1.0 / self.p["max_rate_hz"]:
            return
        self._last_wall = now
        encoding = "mono8" if msg.encoding in ("mono8", "8UC1") else "rgb8"
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        out = LabelDetectionArray()
        out.header = msg.header
        out.source = "zxing"
        out.image_width, out.image_height = msg.width, msg.height
        for result in decode(image, self.p["formats"], self.p["try_harder"]):
            det = LabelDetection()
            det.symbology = symbology_for(result["format"])
            det.code = result["text"]
            det.confidence = 1.0
            corners = result["corners"]
            det.corners_px = [float(v) for v in corners.flatten()]
            det.center_x_px, det.center_y_px = (float(v) for v in corners.mean(axis=0))
            size = self.p["qr_symbol_size_m"] if det.symbology == LabelDetection.SYMBOLOGY_QR else 0.0
            if size > 0.0 and self._camera_matrix is not None:
                solved = solve_label_pose(corners, size, self._camera_matrix, self._dist)
                if solved is not None:
                    det.has_pose, det.pose, det.size_m = True, pose_from_rt(*solved), float(size)
            out.detections.append(det)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CodeReader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
