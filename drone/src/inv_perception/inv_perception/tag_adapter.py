"""AprilTag adapter: apriltag_ros (CPU) detections -> LabelDetectionArray on /perception/detections.

apriltag_ros publishes apriltag_msgs/AprilTagDetectionArray and the tag poses as TF
(camera optical frame -> "<family>:<id>"). We publish them in the same message type as the
barcode reader, keeping the image stamp, so inventory can associate labels seen in one frame.

TODO(phase-2): GPU path (isaac_ros_apriltag, isaac_ros_apriltag_interfaces/AprilTagDetectionArray)
with perception:=gpu; same output.
"""

from __future__ import annotations

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from inv_interfaces.msg import LabelDetection, LabelDetectionArray

DEFAULTS: dict[str, object] = {
    "input_topic": "/perception/apriltag/detections",
    "family": "tag36h11",
    "tag_size_m": 0.09,
    "pose_wait_s": 0.1,
}


def tag_code(family: str, tag_id: int) -> str:
    family = family if family.startswith("tag") else f"tag{family}"
    return f"{family}:{tag_id}"


class TagAdapter(Node):
    def __init__(self) -> None:
        super().__init__("tag_adapter")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.tf_buffer = Buffer()
        # No spin_thread: that would add this node to a second executor, and rclpy nodes belong to
        # one executor at a time (the two fight over it: lost callbacks, tags without poses).
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LabelDetectionArray, "/perception/detections", 10)
        self.create_subscription(AprilTagDetectionArray, self.p["input_topic"], self._on_tags, 10)

    def _on_tags(self, msg: AprilTagDetectionArray) -> None:
        out = LabelDetectionArray()
        out.header = msg.header
        out.source = "apriltag_ros"
        for tag in msg.detections:
            det = LabelDetection()
            det.symbology = LabelDetection.SYMBOLOGY_APRILTAG
            det.code = tag_code(tag.family or self.p["family"], tag.id)
            det.confidence = float(min(1.0, max(0.0, tag.decision_margin / 100.0)))
            det.center_x_px, det.center_y_px = float(tag.centre.x), float(tag.centre.y)
            det.corners_px = [float(v) for c in tag.corners for v in (c.x, c.y)]
            try:
                t = self.tf_buffer.lookup_transform(
                    msg.header.frame_id, det.code, msg.header.stamp, timeout=Duration(seconds=self.p["pose_wait_s"])
                )
                det.has_pose = True
                det.pose.position.x = t.transform.translation.x
                det.pose.position.y = t.transform.translation.y
                det.pose.position.z = t.transform.translation.z
                det.pose.orientation = t.transform.rotation
                det.size_m = float(self.p["tag_size_m"])
            except TransformException:
                det.has_pose = False
            out.detections.append(det)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TagAdapter()
    # Multi-threaded so TF keeps arriving (on the listener's reentrant callback group) while a
    # callback waits for a transform. The node's own callbacks share one mutually exclusive group.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
