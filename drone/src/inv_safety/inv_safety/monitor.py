"""Safety monitor: aggregates battery, localization, FC link and corridor health into
/safety/status, and asks inv_flight_interface to abort (hold, then land) when needed."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from inv_interfaces.msg import CorridorStatus, FlightState, LocalizationStatus, SafetyStatus
from inv_interfaces.srv import Abort
from inv_safety.policy import Inputs, SafetyConfig, evaluate, should_request_abort

DEFAULTS: dict[str, object] = {
    "battery_warn_fraction": 0.30,
    "battery_abort_fraction": 0.20,
    "localization_lost_grace_s": 1.0,
    "fc_link_grace_s": 2.0,
    "state_stale_s": 2.0,
    "rate_hz": 5.0,
}


class SafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.cfg = SafetyConfig(
            battery_warn_fraction=p["battery_warn_fraction"],
            battery_abort_fraction=p["battery_abort_fraction"],
            localization_lost_grace_s=p["localization_lost_grace_s"],
            fc_link_grace_s=p["fc_link_grace_s"],
            state_stale_s=p["state_stale_s"],
        )
        self._flight: FlightState | None = None
        self._flight_s: float | None = None
        self._loc: LocalizationStatus | None = None
        self._loc_lost_since: float | None = None
        self._link_lost_since: float | None = None
        self._corridor: CorridorStatus | None = None
        self._abort_requested = False
        self._last_level = 0

        self.create_subscription(FlightState, "/flight/state", self._on_flight, 10)
        self.create_subscription(LocalizationStatus, "/localization/status", self._on_loc, 10)
        self.create_subscription(CorridorStatus, "/mapping/blocked", self._on_corridor, 10)
        self.status_pub = self.create_publisher(SafetyStatus, "/safety/status", 10)
        self.abort_client = self.create_client(Abort, "/flight/abort")
        self.create_timer(1.0 / p["rate_hz"], self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_flight(self, msg: FlightState) -> None:
        now = self._now()
        self._flight, self._flight_s = msg, now
        if msg.connected:
            self._link_lost_since = None
        elif self._link_lost_since is None:
            self._link_lost_since = now
        if not msg.armed:
            self._abort_requested = False

    def _on_loc(self, msg: LocalizationStatus) -> None:
        self._loc = msg
        if msg.state == LocalizationStatus.STATE_LOST:
            if self._loc_lost_since is None:
                self._loc_lost_since = self._now()
        else:
            self._loc_lost_since = None

    def _on_corridor(self, msg: CorridorStatus) -> None:
        self._corridor = msg

    def _tick(self) -> None:
        now = self._now()
        if self._loc is None and self._loc_lost_since is None:
            self._loc_lost_since = now  # no localization status yet: count it as lost from now on
        f = self._flight
        x = Inputs(
            now_s=now,
            flight_state_age_s=math.inf if self._flight_s is None else now - self._flight_s,
            connected=bool(f and f.connected),
            armed=bool(f and f.armed),
            flying=bool(f and f.flying),
            pilot_override=bool(f and f.pilot_override),
            battery_fraction=float(f.battery_percentage) if f else math.nan,
            localization_lost_since_s=self._loc_lost_since,
            localization_degraded=bool(self._loc and self._loc.state == LocalizationStatus.STATE_DEGRADED),
            corridor_blocked=bool(self._corridor and self._corridor.blocked),
            fc_link_lost_since_s=self._link_lost_since,
        )
        verdict = evaluate(self.cfg, x)
        if should_request_abort(verdict, x, self._abort_requested):
            self._request_abort("; ".join(verdict.reasons))
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.level = verdict.level
        msg.reasons = verdict.reasons
        msg.abort_requested = self._abort_requested
        self.status_pub.publish(msg)
        if verdict.level != self._last_level:
            names = {0: "OK", 1: "WARN", 2: "ABORT"}
            # Two call sites: rclpy raises if one call site changes severity.
            text = f"safety {names[verdict.level]}: {'; '.join(verdict.reasons) or 'all good'}"
            if verdict.level == 0:
                self.get_logger().info(text)
            else:
                self.get_logger().warning(text)
            self._last_level = verdict.level

    def _request_abort(self, reason: str) -> None:
        if not self.abort_client.service_is_ready():
            self.get_logger().error(f"want to abort ({reason}) but /flight/abort is not available")
            return
        self.get_logger().error(f"requesting abort: {reason}")
        self._abort_requested = True
        self.abort_client.call_async(Abort.Request(reason=f"safety: {reason}"))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
