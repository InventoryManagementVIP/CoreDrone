"""Mission node: a py_trees behaviour tree over the inv_flight_interface actions.

  mission:=smoke  wait for ready -> take off 1.5 m -> hover 10 s -> land
  mission:=scan   wait for ready -> take off -> visit every hover point of the generated route
                  (stop and hover at each) -> return home -> land
Publishes /mission/status at a steady rate; inv_flight_interface treats it as a heartbeat.
If the safety monitor requests an abort, the pilot takes over or a step fails, the tree stops;
the flight interface does the landing, which it starts itself when it sees a FAILED or ABORTED
status in the air. TODO(phase-2): retries, rescans, corridor-blocked hold branch.
"""

from __future__ import annotations

import datetime as dt

import py_trees
import py_trees_ros
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from inv_interfaces.msg import FlightState, LocalizationStatus, MissionStatus, SafetyStatus
from inv_mission import behaviours as bt
from inv_mission.route import Waypoint, generate_route

DEFAULTS: dict[str, object] = {
    "mission": "scan",
    "warehouse": "sim_small",
    "mission_id": "",
    "takeoff_altitude_m": 1.0,
    "smoke_altitude_m": 1.5,
    "smoke_hover_s": 10.0,
    "tick_hz": 5.0,
    "status_rate_hz": 2.0,
    "camera_frame": "camera_color",
    "base_frame": "base_link",
    "ready_timeout_s": 300.0,
}


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}
        if self.p["mission"] not in ("scan", "smoke"):
            raise ValueError(f"mission:={self.p['mission']!r}; choose scan or smoke")
        self.mission_id = self.p["mission_id"] or f"{self.p['warehouse']}-{dt.datetime.now():%Y%m%d-%H%M%S}"
        self.flight: FlightState | None = None
        self.localization: LocalizationStatus | None = None
        self.state = MissionStatus.STATE_IDLE
        self.message = ""
        self.route: list[Waypoint] = []
        self.tree: py_trees_ros.trees.BehaviourTree | None = None
        self.ready_check: bt.WaitForReady | None = None
        self.step_names: list[str] = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(FlightState, "/flight/state", self._on_flight, 10)
        self.create_subscription(LocalizationStatus, "/localization/status", self._on_loc, 10)
        self.create_subscription(SafetyStatus, "/safety/status", self._on_safety, 10)
        self.status_pub = self.create_publisher(MissionStatus, "/mission/status", 10)
        self.create_timer(1.0 / self.p["status_rate_hz"], self._publish_status)
        self._tick_timer = self.create_timer(1.0 / self.p["tick_hz"], self._tick)
        self._started_s = self._now()
        self.get_logger().info(f"mission {self.mission_id} ({self.p['mission']}) on {self.p['warehouse']}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_flight(self, msg: FlightState) -> None:
        self.flight = msg
        if msg.pilot_override and self.state in (MissionStatus.STATE_TAKING_OFF, MissionStatus.STATE_RUNNING):
            self._stop(MissionStatus.STATE_ABORTED, "pilot took control")

    def _on_loc(self, msg: LocalizationStatus) -> None:
        self.localization = msg

    def _on_safety(self, msg: SafetyStatus) -> None:
        if msg.abort_requested and self.state in (MissionStatus.STATE_TAKING_OFF, MissionStatus.STATE_RUNNING):
            self._stop(MissionStatus.STATE_ABORTED, "safety abort: " + "; ".join(msg.reasons))

    # ---- tree ---------------------------------------------------------------------------------
    def _camera_offset(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform(self.p["base_frame"], self.p["camera_frame"], Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, tr.z)

    def _build_tree(self) -> bool:
        if self.p["mission"] == "scan":
            offset = self._camera_offset()
            if offset is None:
                if self._now() - self._started_s < 15.0:
                    return False  # wait for robot_state_publisher's static TF
                self.get_logger().warning("no TF base_link->camera: hover points assume the camera at base_link")
                offset = (0.0, 0.0, 0.0)
            from inv_layout import load_layout

            layout = load_layout(self.p["warehouse"])
            self.route = generate_route(layout, offset)
            steps = [bt.takeoff("takeoff", self.p["takeoff_altitude_m"])]
            for wp in self.route:
                steps.append(bt.goto(f"goto {wp.name}", wp.pose.x, wp.pose.y, wp.pose.z, wp.pose.yaw, wp.speed_mps))
                if wp.kind == "scan" and wp.hover_s > 0:
                    steps.append(bt.hover(f"scan {wp.slot_code}", wp.hover_s))
            steps.append(bt.land("land"))
            scans = sum(1 for wp in self.route if wp.kind == "scan")
            self.get_logger().info(f"route: {len(self.route)} waypoints, {scans} hover points")
        else:
            steps = [
                bt.takeoff("takeoff", self.p["smoke_altitude_m"]),
                bt.hover("hover", self.p["smoke_hover_s"]),
                bt.land("land"),
            ]
        self.ready_check = bt.WaitForReady(
            "wait for ready", lambda: self.flight, lambda: self.localization, self._now, self.p["ready_timeout_s"]
        )
        self.step_names = [s.name for s in steps]
        root = py_trees.composites.Sequence("mission", memory=True, children=[self.ready_check, *steps])
        self.tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
        self.tree.setup(node=self, timeout=15.0)
        self.state = MissionStatus.STATE_WAITING_FOR_READY
        return True

    def _tick(self) -> None:
        if self.state in (MissionStatus.STATE_COMPLETED, MissionStatus.STATE_ABORTED, MissionStatus.STATE_FAILED):
            return
        if self.tree is None and not self._build_tree():
            return
        self.tree.tick()
        root = self.tree.root
        running = [b for b in root.children if b.status == py_trees.common.Status.RUNNING]
        current = running[0].name if running else ""
        if current == "wait for ready":
            self.state, self.message = MissionStatus.STATE_WAITING_FOR_READY, self.ready_check.feedback_message
        elif current == "takeoff":
            self.state, self.message = MissionStatus.STATE_TAKING_OFF, ""
        elif current == "land":
            self.state, self.message = MissionStatus.STATE_LANDING, ""
        elif current:
            self.state, self.message = MissionStatus.STATE_RUNNING, current
        if root.status == py_trees.common.Status.SUCCESS:
            self._stop(MissionStatus.STATE_COMPLETED, "mission complete")
        elif root.status == py_trees.common.Status.FAILURE:
            failed = [b for b in root.children if b.status == py_trees.common.Status.FAILURE]
            reason = f"{failed[0].name}: {failed[0].feedback_message}" if failed else "unknown"
            self._stop(MissionStatus.STATE_FAILED, reason)

    def _stop(self, state: int, message: str) -> None:
        self.state, self.message = state, message
        if self.tree is not None and state != MissionStatus.STATE_COMPLETED:
            self.tree.interrupt()
        # Two call sites: rclpy raises if one call site changes severity.
        text = f"mission {self.mission_id}: {message}"
        if state == MissionStatus.STATE_COMPLETED:
            self.get_logger().info(text)
        else:
            self.get_logger().error(text)

    def _current_waypoint(self) -> int:
        if self.tree is None or not self.route:
            return 0
        done = sum(
            1
            for b in self.tree.root.children
            if b.name.startswith("goto") and b.status == py_trees.common.Status.SUCCESS
        )
        return min(done, len(self.route))

    def _publish_status(self) -> None:
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = self.mission_id
        msg.warehouse_id = self.p["warehouse"]
        msg.state = self.state
        msg.waypoint_index = self._current_waypoint()
        msg.waypoint_count = len(self.route)
        msg.current_task = self.message if self.state == MissionStatus.STATE_RUNNING else ""
        msg.message = self.message
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
