"""py_trees behaviours for the mission. Flight actions go through inv_flight_interface only."""

from __future__ import annotations

import math
from collections.abc import Callable

import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped

from inv_interfaces.action import GoTo, Hover, Land, Takeoff
from inv_interfaces.msg import FlightState, LocalizationStatus

ACTION_SERVER_WAIT_S = 30.0


class WaitForReady(py_trees.behaviour.Behaviour):
    """RUNNING until the flight controller link, EKF origin, external nav and localization are good."""

    def __init__(
        self,
        name: str,
        flight: Callable[[], FlightState | None],
        localization: Callable[[], LocalizationStatus | None],
        now: Callable[[], float],
        timeout_s: float,
    ):
        super().__init__(name)
        self._flight, self._loc, self._now, self._timeout_s = flight, localization, now, timeout_s
        self._started: float | None = None
        self.waiting_for = ""

    def initialise(self) -> None:
        self._started = self._now()

    def update(self) -> py_trees.common.Status:
        f, loc = self._flight(), self._loc()
        missing = []
        if f is None or not f.connected:
            missing.append("flight controller")
        else:
            if not f.ekf_origin_set:
                missing.append("EKF origin")
            if not f.external_nav_ok:
                missing.append("external nav")
            if f.pilot_override:
                missing.append("pilot has control")
        if loc is None or loc.state != LocalizationStatus.STATE_TRACKING:
            missing.append("localization tracking")
        self.waiting_for = ", ".join(missing)
        if not missing:
            return py_trees.common.Status.SUCCESS
        if self._now() - self._started > self._timeout_s:
            self.feedback_message = f"not ready after {self._timeout_s:.0f} s: {self.waiting_for}"
            return py_trees.common.Status.FAILURE
        self.feedback_message = f"waiting for {self.waiting_for}"
        return py_trees.common.Status.RUNNING


def takeoff(name: str, altitude_m: float) -> py_trees.behaviour.Behaviour:
    return py_trees_ros.action_clients.FromConstant(
        name=name,
        action_type=Takeoff,
        action_name="/flight/takeoff",
        action_goal=Takeoff.Goal(altitude_m=float(altitude_m)),
        wait_for_server_timeout_sec=ACTION_SERVER_WAIT_S,
    )


def goto(
    name: str, x: float, y: float, z: float, yaw: float, max_speed_mps: float = 0.0
) -> py_trees.behaviour.Behaviour:
    target = PoseStamped()
    target.header.frame_id = "map"
    target.pose.position.x, target.pose.position.y, target.pose.position.z = float(x), float(y), float(z)
    target.pose.orientation.z, target.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return py_trees_ros.action_clients.FromConstant(
        name=name,
        action_type=GoTo,
        action_name="/flight/goto",
        action_goal=GoTo.Goal(target=target, max_speed_mps=float(max_speed_mps)),
        wait_for_server_timeout_sec=ACTION_SERVER_WAIT_S,
    )


def hover(name: str, duration_s: float) -> py_trees.behaviour.Behaviour:
    return py_trees_ros.action_clients.FromConstant(
        name=name,
        action_type=Hover,
        action_name="/flight/hover",
        action_goal=Hover.Goal(duration_s=float(duration_s)),
        wait_for_server_timeout_sec=ACTION_SERVER_WAIT_S,
    )


def land(name: str) -> py_trees.behaviour.Behaviour:
    return py_trees_ros.action_clients.FromConstant(
        name=name,
        action_type=Land,
        action_name="/flight/land",
        action_goal=Land.Goal(),
        wait_for_server_timeout_sec=ACTION_SERVER_WAIT_S,
    )
