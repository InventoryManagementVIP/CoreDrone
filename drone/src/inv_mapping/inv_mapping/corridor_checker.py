"""Corridor checker: publishes /mapping/blocked for the route segment ahead.

Phase 1 scope (docs/adr/0004-isaac-vslam-swappable-localization.md, brief section 3.4): no 3D
path planning. With mapping:=nvblox the job is a "stop and hold if the corridor ahead is blocked"
check against nvblox's ESDF; with mapping:=none we publish UNKNOWN so consumers always have a signal.

TODO(phase-2): nvblox backend. Query nvblox's ~/get_esdf_and_gradient service (nvblox_msgs) for an
axis-aligned box around the next route segment (from /mission/status + the route), and publish
BLOCKED when the minimum distance is below `min_clearance_m`.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from inv_interfaces.msg import CorridorStatus

DEFAULTS: dict[str, object] = {
    "backend": "none",
    "rate_hz": 5.0,
    "min_clearance_m": 0.6,
}


class CorridorChecker(Node):
    def __init__(self) -> None:
        super().__init__("corridor_checker")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        if self.p["backend"] not in ("none", "nvblox"):
            raise ValueError(f"unknown mapping backend {self.p['backend']!r}")
        if self.p["backend"] == "nvblox":
            self.get_logger().warning("nvblox corridor check is TODO(phase-2): reporting UNKNOWN")
        self.pub = self.create_publisher(CorridorStatus, "/mapping/blocked", 10)
        self.create_timer(1.0 / self.p["rate_hz"], self._tick)

    def _tick(self) -> None:
        msg = CorridorStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.backend = self.p["backend"]
        msg.state = CorridorStatus.STATE_UNKNOWN
        msg.blocked = False
        msg.clearance_m = math.nan
        msg.message = "no obstacle map (mapping:=none)" if self.p["backend"] == "none" else "not implemented"
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CorridorChecker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
