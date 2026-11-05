"""Save camera frames to PNG, e.g. to see what the label camera sees when a decode fails.

ros2 run inv_tools snapshot --topic /camera/color/image_raw --count 3 --out /ws/runs/snapshots
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--every", type=int, default=1, help="keep every Nth frame")
    parser.add_argument("--out", type=Path, default=Path("snapshots"))
    parser.add_argument("--timeout", type=float, default=30.0)
    argv = sys.argv[1:] if argv is None else argv
    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]
    args = parser.parse_args(argv)

    import cv2
    from cv_bridge import CvBridge

    rclpy.init()
    node = rclpy.create_node("snapshot")
    bridge = CvBridge()
    args.out.mkdir(parents=True, exist_ok=True)
    saved: list[Path] = []
    seen = [0]

    def on_image(msg: Image) -> None:
        seen[0] += 1
        if len(saved) >= args.count or (seen[0] - 1) % args.every:
            return
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8" if msg.encoding != "mono8" else "mono8")
        stamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
        path = args.out / f"{args.topic.strip('/').replace('/', '_')}_{stamp}.png"
        cv2.imwrite(str(path), image)
        saved.append(path)
        print(path, flush=True)

    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    try:
        while len(saved) < args.count and node.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if saved else 1


if __name__ == "__main__":
    sys.exit(main())
