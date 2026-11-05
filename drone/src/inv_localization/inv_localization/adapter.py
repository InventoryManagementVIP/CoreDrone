"""Localization adapter: the same node for every backend (groundtruth, isaac, rtabmap).

* Takes the backend's odometry (/localization/odom) and turns it into odom->base_link, whatever
  frames the backend uses (camera-frame odometry is moved to base_link with the URDF's static TF).
* Publishes it on /localization/external_nav at a steady rate. inv_flight_interface forwards it
  to the flight controller's EKF; the adapter itself never talks to the FC.
* Monitors tracking health and publishes /localization/status. When tracking is lost it stops
  publishing (so the EKF times out instead of fusing stale data) and reports LOST, which
  inv_safety turns into an abort.
"""

from __future__ import annotations

import collections
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from inv_interfaces.msg import LocalizationStatus
from inv_localization.transforms import matrix_to_quat, pose_to_matrix, transform_to_matrix

DEFAULTS: dict[str, object] = {
    "backend": "groundtruth",
    "input_topic": "/localization/odom",
    "backend_status_topic": "/localization/backend_status",
    "odom_frame": "odom",
    "base_frame": "base_link",
    "publish_rate_hz": 30.0,
    "min_rate_hz": 15.0,
    "lost_timeout_s": 0.5,
    "status_rate_hz": 5.0,
    "backend_status_timeout_s": 2.0,
}


def classify(age_s: float, rate_hz: float, backend_state: int | None, lost_timeout_s: float, min_rate_hz: float) -> int:
    """Health state from sample age, input rate and what the backend says about itself."""
    if backend_state == LocalizationStatus.STATE_LOST or not math.isfinite(age_s) or age_s > lost_timeout_s:
        return LocalizationStatus.STATE_LOST
    if backend_state == LocalizationStatus.STATE_DEGRADED or rate_hz < min_rate_hz:
        return LocalizationStatus.STATE_DEGRADED
    return LocalizationStatus.STATE_TRACKING


class LocalizationAdapter(Node):
    def __init__(self) -> None:
        super().__init__("localization_adapter")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._latest: Odometry | None = None
        self._latest_stamp_s: float | None = None
        self._last_published_stamp_s: float | None = None
        self._last_publish_s: float | None = None
        self._arrivals: collections.deque[float] = collections.deque()
        self._backend_state: int | None = None
        self._backend_message = ""
        self._backend_status_s: float | None = None
        self._last_state: int | None = None

        self.out_pub = self.create_publisher(Odometry, "/localization/external_nav", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(LocalizationStatus, "/localization/status", 10)
        self.create_subscription(Odometry, self.p["input_topic"], self._on_odom, 20)
        self.create_subscription(LocalizationStatus, self.p["backend_status_topic"], self._on_backend_status, 10)
        self.create_timer(1.0 / self.p["publish_rate_hz"], self._publish)
        self.create_timer(1.0 / self.p["status_rate_hz"], self._publish_status)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs ---------------------------------------------------------------------------------
    def _on_backend_status(self, msg: LocalizationStatus) -> None:
        self._backend_state = msg.state
        self._backend_message = msg.message
        self._backend_status_s = self._now()

    def _on_odom(self, msg: Odometry) -> None:
        odom = self._to_odom_base_link(msg)
        if odom is None:
            return
        now = self._now()
        self._latest = odom
        self._latest_stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._arrivals.append(now)
        while self._arrivals and now - self._arrivals[0] > 2.0:
            self._arrivals.popleft()

    def _to_odom_base_link(self, msg: Odometry) -> Odometry | None:
        """Express the backend's pose as odom->base_link (REP-105)."""
        odom_frame, base_frame = self.p["odom_frame"], self.p["base_frame"]
        parent = msg.header.frame_id or odom_frame
        child = msg.child_frame_id or base_frame
        if parent == odom_frame and child == base_frame:
            return msg
        m = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        try:
            if child != base_frame:  # e.g. VSLAM reporting the camera pose
                m = m @ transform_to_matrix(self.tf_buffer.lookup_transform(child, base_frame, Time()).transform)
            if parent != odom_frame:
                m = transform_to_matrix(self.tf_buffer.lookup_transform(odom_frame, parent, Time()).transform) @ m
        except TransformException as exc:
            self.get_logger().warning(
                f"cannot express {parent}->{child} as {odom_frame}->{base_frame}: {exc}", throttle_duration_sec=5.0
            )
            return None
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id, out.child_frame_id = odom_frame, base_frame
        pos = out.pose.pose.position
        pos.x, pos.y, pos.z = (float(v) for v in m[:3, 3])
        q = out.pose.pose.orientation
        q.x, q.y, q.z, q.w = matrix_to_quat(m[:3, :3])
        out.pose.covariance = msg.pose.covariance
        return out

    # ---- outputs --------------------------------------------------------------------------------
    def _age(self, now: float) -> float:
        return math.inf if self._latest_stamp_s is None else now - self._latest_stamp_s

    def _rate(self) -> float:
        if len(self._arrivals) < 2:
            return 0.0
        span = self._arrivals[-1] - self._arrivals[0]
        return (len(self._arrivals) - 1) / span if span > 0 else 0.0

    def _backend_state_now(self, now: float) -> int | None:
        if self._backend_status_s is None or now - self._backend_status_s > self.p["backend_status_timeout_s"]:
            return None
        return self._backend_state

    def _publish(self) -> None:
        now = self._now()
        if self._latest is None or self._latest_stamp_s == self._last_published_stamp_s:
            return
        if self._age(now) > self.p["lost_timeout_s"]:
            return
        if self._backend_state_now(now) == LocalizationStatus.STATE_LOST:
            return
        self.out_pub.publish(self._latest)
        self._last_published_stamp_s = self._latest_stamp_s
        self._last_publish_s = now

    def _publish_status(self) -> None:
        now = self._now()
        age, rate = self._age(now), self._rate()
        state = classify(age, rate, self._backend_state_now(now), self.p["lost_timeout_s"], self.p["min_rate_hz"])
        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.backend = self.p["backend"]
        msg.state = state
        msg.rate_hz = float(rate)
        msg.age_s = float(age)
        msg.publishing_external_nav = (
            self._last_publish_s is not None and now - self._last_publish_s < self.p["lost_timeout_s"]
        )
        if state == LocalizationStatus.STATE_LOST:
            msg.message = self._backend_message or (
                "no odometry yet" if self._latest is None else f"odometry is {age:.2f} s old"
            )
        elif state == LocalizationStatus.STATE_DEGRADED:
            msg.message = self._backend_message or f"odometry rate {rate:.1f} Hz < {self.p['min_rate_hz']} Hz"
        self.status_pub.publish(msg)
        if state != self._last_state:
            names = {0: "UNKNOWN", 1: "TRACKING", 2: "DEGRADED", 3: "LOST"}
            # Two call sites: rclpy raises if one call site changes severity.
            text = f"localization {names.get(state, state)} ({msg.message or 'ok'})"
            if state == LocalizationStatus.STATE_TRACKING:
                self.get_logger().info(text)
            else:
                self.get_logger().warning(text)
            self._last_state = state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
