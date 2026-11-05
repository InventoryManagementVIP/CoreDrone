"""ArduPilot AP_DDS backend.

This is the only module in the repository that names /ap/* topics or services
(enforced by test/test_ap_topics_confined.py). Topic names, types and QoS are from
libraries/AP_DDS/AP_DDS_Topic_Table.h at Copter-4.7.2.
"""

from __future__ import annotations

import math
import threading

from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from builtin_interfaces.msg import Time
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from tf2_msgs.msg import TFMessage

from inv_flight_interface.backend import FcState, FlightBackend, Request
from inv_flight_interface.control import yaw_from_quaternion
from inv_flight_interface.time_sync import ClockOffsetEstimator

_FAILSAFE_NAMES = {
    Status.FS_RADIO: "radio",
    Status.FS_BATTERY: "battery",
    Status.FS_GCS: "gcs",
    Status.FS_EKF: "ekf",
}

# AP_DDS accepts external odometry only as parent "odom", child "base_link"
# (AP_DDS_External_Odom::is_odometry_frame).
EXTERNAL_NAV_PARENT = "odom"
EXTERNAL_NAV_CHILD = "base_link"
# Velocity commands in this frame are read as ENU in the FC's map frame
# (AP_DDS_External_Control::handle_velocity_control). We do not use "base_link": at Copter-4.7.2
# that path converts y-left as if it were y-right (docs/ardupilot.md).
CMD_VEL_FRAME = "map"


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _time_msg(seconds: float) -> Time:
    sec = math.floor(seconds)
    return Time(sec=int(sec), nanosec=round((seconds - sec) * 1e9) % 1_000_000_000)


class ApDdsBackend(FlightBackend):
    name = "ap_dds"

    def __init__(self, node: Node, callback_group, namespace: str = "/ap"):
        self._node = node
        self._ns = namespace.rstrip("/")
        self._lock = threading.Lock()
        self._state = FcState()
        self.clock = ClockOffsetEstimator()

        best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        def sub(msg_type, name, cb, qos):
            node.create_subscription(msg_type, f"{self._ns}/{name}", cb, qos, callback_group=callback_group)

        sub(Status, "status", self._on_status, latched)
        sub(Time, "time", self._on_time, best_effort)
        sub(BatteryState, "battery", self._on_battery, best_effort)
        sub(GeoPointStamped, "gps_global_origin/filtered", self._on_origin, best_effort)
        sub(PoseStamped, "pose/filtered", self._on_pose, best_effort)

        self._cmd_vel_pub = node.create_publisher(TwistStamped, f"{self._ns}/cmd_vel", best_effort)
        self._tf_pub = node.create_publisher(TFMessage, f"{self._ns}/tf", best_effort)

        def client(srv_type, name):
            return node.create_client(srv_type, f"{self._ns}/{name}", callback_group=callback_group)

        self._mode_client = client(ModeSwitch, "mode_switch")
        self._arm_client = client(ArmMotors, "arm_motors")
        self._takeoff_client = client(Takeoff, "experimental/takeoff")

    # ---- FC -> us --------------------------------------------------------------------------
    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _touch(self) -> None:
        self._state.last_message_s = self._now()

    def _on_status(self, msg: Status) -> None:
        with self._lock:
            self._touch()
            self._state.armed = bool(msg.armed)
            self._state.flying = bool(msg.flying)
            self._state.mode_id = int(msg.mode)
            self._state.failsafes = [_FAILSAFE_NAMES.get(int(f), f"fs_{int(f)}") for f in msg.failsafe]

    def _on_time(self, msg: Time) -> None:
        now = self._now()
        with self._lock:
            self._state.last_message_s = now
            self.clock.update(_seconds(msg), now)

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._touch()
            self._state.battery_voltage = float(msg.voltage)
            self._state.battery_percentage = float(msg.percentage)

    def _on_origin(self, msg: GeoPointStamped) -> None:
        with self._lock:
            self._touch()
            p = msg.position
            self._state.ekf_origin_set = abs(p.latitude) > 1e-9 or abs(p.longitude) > 1e-9

    def _on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        with self._lock:
            self._touch()
            self._state.fc_yaw_enu = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            self._state.fc_yaw_stamp_s = self._now()

    # ---- FlightBackend ---------------------------------------------------------------------
    def state(self) -> FcState:
        with self._lock:
            s = self._state
            return FcState(
                last_message_s=s.last_message_s,
                armed=s.armed,
                flying=s.flying,
                mode_id=s.mode_id,
                failsafes=list(s.failsafes),
                battery_voltage=s.battery_voltage,
                battery_percentage=s.battery_percentage,
                ekf_origin_set=s.ekf_origin_set,
                fc_yaw_enu=s.fc_yaw_enu,
                fc_yaw_stamp_s=s.fc_yaw_stamp_s,
            )

    def services_ready(self) -> bool:
        return all(c.service_is_ready() for c in (self._mode_client, self._arm_client, self._takeoff_client))

    def request_mode(self, mode_id: int) -> Request:
        return Request(self._mode_client.call_async(ModeSwitch.Request(mode=mode_id)), lambda r: r.status)

    def request_arm(self, arm: bool) -> Request:
        return Request(self._arm_client.call_async(ArmMotors.Request(arm=arm)), lambda r: r.result)

    def request_takeoff(self, altitude_m: float) -> Request:
        return Request(self._takeoff_client.call_async(Takeoff.Request(alt=float(altitude_m))), lambda r: r.status)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = CMD_VEL_FRAME
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.z = float(yaw_rate)
        self._cmd_vel_pub.publish(msg)

    def clock_ready(self) -> bool:
        with self._lock:
            return self.clock.offset_s is not None and not self.clock.fc_clock_is_utc

    def forward_external_nav(self, odom: Odometry) -> bool:
        with self._lock:
            if self.clock.fc_clock_is_utc:
                return False
            fc_stamp = self.clock.to_fc(_seconds(odom.header.stamp))
        if fc_stamp is None:
            return False
        t = TransformStamped()
        t.header.stamp = _time_msg(fc_stamp)
        t.header.frame_id = EXTERNAL_NAV_PARENT
        t.child_frame_id = EXTERNAL_NAV_CHILD
        p = odom.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = odom.pose.pose.orientation
        self._tf_pub.publish(TFMessage(transforms=[t]))
        return True
