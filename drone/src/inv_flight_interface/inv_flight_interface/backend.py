"""Flight-controller backend interface.

The node talks to the flight controller only through this interface. Today the one
implementation is AP_DDS (ap_dds_backend.py). Switching to MAVROS means writing a second
implementation here; nothing outside this package changes (docs/adr/0003-ardupilot-dds.md).
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass, field

from nav_msgs.msg import Odometry


@dataclass
class FcState:
    """What the backend currently knows about the flight controller."""

    last_message_s: float | None = None  # ROS time of the latest FC message
    armed: bool = False
    flying: bool = False
    mode_id: int | None = None
    failsafes: list[str] = field(default_factory=list)
    battery_voltage: float = math.nan
    battery_percentage: float = math.nan
    ekf_origin_set: bool = False
    # Yaw the FC reports for itself, in its own ENU map frame (None until known).
    fc_yaw_enu: float | None = None
    fc_yaw_stamp_s: float | None = None


class Request:
    """A pending request to the flight controller (mode change, arm, takeoff)."""

    def __init__(self, future, interpret: Callable[[object], bool]):
        self._future = future
        self._interpret = interpret

    def done(self) -> bool:
        return self._future is None or self._future.done()

    def succeeded(self) -> bool:
        if self._future is None or not self._future.done() or self._future.exception() is not None:
            return False
        return bool(self._interpret(self._future.result()))


class FlightBackend(ABC):
    name = "abstract"

    @abstractmethod
    def state(self) -> FcState: ...

    @abstractmethod
    def services_ready(self) -> bool: ...

    @abstractmethod
    def request_mode(self, mode_id: int) -> Request: ...

    @abstractmethod
    def request_arm(self, arm: bool) -> Request: ...

    @abstractmethod
    def request_takeoff(self, altitude_m: float) -> Request: ...

    @abstractmethod
    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """Velocity in the FC's ENU map frame, m/s, and yaw rate in rad/s (CCW positive)."""

    @abstractmethod
    def forward_external_nav(self, odom: Odometry) -> bool:
        """Send odom->base_link to the FC. Returns False if it could not be sent."""

    @abstractmethod
    def clock_ready(self) -> bool:
        """True when external-nav timestamps can be converted to the FC clock."""
